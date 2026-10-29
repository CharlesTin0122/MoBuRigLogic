/**
 * RigLogicHeadConstraint 实现。
 *
 * 求值流程（每 EvaluationID 一次）：
 *   1. 读 252 个表情属性（GetData + evalInfo，支持K帧动画）→ setRawControl
 *   2. 读 neck_01/neck_02/head 及父级全局旋转 → q_rel = q中立⁻¹∘q父全局⁻¹∘q全局 → 12 个四元数 raw
 *   3. rig->calculate → 输出节点按通知写：关节 T(中立+增量)/R(q中立∘q增量 合成)；BS ×100
 */

#include "riglogichead_constraint.h"
#include "riglogic_common.h"
#include "riglogic_notice.h"
#include "riglogic_scene.h"

#include <fbsdk/fbundomanager.h>

#include <dna/BinaryStreamReader.h>
#include <dna/Configuration.h>
#include <riglogic/riglogic/RigLogic.h>
#include <riglogic/riglogic/RigInstance.h>
#include <status/Status.h>
#include <trio/streams/FileStream.h>

#include <chrono>
#include <cmath>
#include <map>
#include <mutex>

#define RIGLOGICHEAD__CLASS   RIGLOGICHEAD__CLASSNAME
#define RIGLOGICHEAD__NAME    "RigLogic Head Expression"
#define RIGLOGICHEAD__LABEL   "RigLogic Head Expression"
#define RIGLOGICHEAD__DESC    "MetaHuman facial joints & blendshapes driven by RigLogic (head.dna)"

FBConstraintImplementation( RIGLOGICHEAD__CLASS );
FBRegisterConstraint( RIGLOGICHEAD__NAME,
                      RIGLOGICHEAD__CLASS,
                      RIGLOGICHEAD__LABEL,
                      RIGLOGICHEAD__DESC,
                      FB_DEFAULT_SDK_ICON );

using moburiglogic::CollectModelsNs;
using moburiglogic::ComposeJointRotation;
using moburiglogic::DriverRelativeQuat;
using moburiglogic::EulerDegToQuat;
using moburiglogic::ExtractNamespace;
using moburiglogic::NeedsNeutralCompose;
using moburiglogic::QuatConj;

// 绑定 FBX 同目录下的固定文件名：公司路径不可用时用来兜底（棚内/外拍常用）
static const char* kSceneFolderDnaName = "head.dna";

// FaceBoard 眼睛摇杆：左右眼在 DNA 的 GUI 通道里，中间那只不在（需手动叠回）
static const char* kLeftEyeStick   = "CTRL_L_eye";
static const char* kRightEyeStick  = "CTRL_R_eye";
static const char* kCentreEyeStick = "CTRL_C_eye";

bool RigLogicHeadConstraint::FBCreate()
{
    FBPropertyPublish( this, DnaPath,     "DNA Path",      nullptr, nullptr );
    FBPropertyPublish( this, LodLevel,    "LOD Level",     nullptr, nullptr );
    FBPropertyPublish( this, InputMode,   "Input Mode",    nullptr, nullptr );
    FBPropertyPublish( this, LastSolveMs, "Last Solve Ms", nullptr, nullptr );
    FBPropertyPublish( this, LockPanelFrames, "Lock Panel Frames", nullptr, nullptr );
    FBPropertyPublish( this, EyeLinkage,   "Eye Linkage",   nullptr, nullptr );
    LodLevel = 0;
    InputMode = 0;   // 0=表情属性 1=FaceBoard面板
    LastSolveMs = 0.0;
    LockPanelFrames = false;
    EyeLinkage = true;

    mGroupSkeleton = ReferenceGroupAdd( "Skeleton Root", 1 );
    Deformer = false;
    HasLayout = true;
    FBSystem().OnUIIdle.Add( this, (FBCallback)&RigLogicHeadConstraint::EventUIIdle );
    return true;
}

// 求值线程只写 mLastSolveMs 原子量；属性写入会触发通知，必须回到主线程
void RigLogicHeadConstraint::EventUIIdle( HISender, HKEvent )
{
    const double ms = mLastSolveMs.load();
    if ((double)LastSolveMs != ms) LastSolveMs = ms;

    // LOD 改动即时生效：驱动集/BS 映射随 LOD 变化，需重建绑定（不止 setLOD）。
    // 先记下目标 LOD 再重建，重建失败也不会每个空闲周期反复重试。
    // 播放中不重建，也不记 mAppliedLod，停止后自然会再触发一次
    const bool playing = moburiglogic::IsTransportPlaying();
    mStopIdleTicks = playing ? 0 : (mStopIdleTicks + 1);   // 停播后累计空闲周期，等在本飞行求值排空
    const int lod = static_cast<int>( LodLevel );
    if (mRig && mAppliedLod >= 0 && lod != mAppliedLod
        && !playing && mStopIdleTicks >= 10)
    {
        mAppliedLod = lod;
        RebuildBindings();
    }

    // 播放中挂起的重建：等停播且空闲稳定后再补做，避开“停播瞬间求值未完全结束”
    if (mRebuildPending && !playing && mStopIdleTicks >= 10)
    {
        mRebuildPending = false;
        RebuildBindings();
    }

    // Lock Panel Frames 改动（Python/属性面板设置）即时生效。首个空闲周期只记录当前值：
    // FRM_* 的可选状态本身随 FBX 保存，打开场景时不覆盖用户手动改过的状态
    const int frameLock = static_cast<bool>( LockPanelFrames ) ? 1 : 0;
    if (mAppliedFrameLock < 0)
    {
        mAppliedFrameLock = frameLock;
    }
    else if (frameLock != mAppliedFrameLock)
    {
        mAppliedFrameLock = frameLock;
        ApplyPanelFramesLock( frameLock == 1 );
    }

    // DNA 加载失败时的兜底懒重试。FbxRetrieve(kCleanup) 先于场景文件名就绪的情况存在
    // （FBXFileName 为空 → 找不到 FBX 同目录的 head.dna），所以每遇到一组新的
    // （场景路径, DNA 属性路径）组合就重试一次，避免每个空闲周期反复读 DNA
    if (!mRig && !moburiglogic::IsTransportPlaying())
    {
        const char* prop = DnaPath.AsString();
        const std::string key = moburiglogic::SceneFilePath() + "|" + (prop ? prop : "");
        if (key != mDnaRetryKey)
        {
            mDnaRetryKey = key;
            if (LoadDna()) RebuildBindings();
        }
    }

    // DNA 解析通知（自动改写 / 未找到）：由主线程空闲统一弹一条汇总消息，
    // 不在加载与重建回调里弹模态框
    moburiglogic::DnaNoticeQueue::DrainIfReady( moburiglogic::IsTransportPlaying() );

    // 旧版本文件恢复出来的 "Source DNA Path" 属性：MoBu 在 kCleanup 之后才恢复动态属性，
    // 所以在空闲里清（PropertyRemove 是官方移除属性的接口）；已删除时 Find 返回空
    // 注意：放在通知排空之后，避免属性表变动影响弹窗
    if (FBProperty* legacy = PropertyList.Find( "Source DNA Path" ))
        PropertyRemove( legacy );
}

void RigLogicHeadConstraint::SetPanelFramesLocked( bool locked )
{
    LockPanelFrames = locked;
    mAppliedFrameLock = locked ? 1 : 0;   // 已在此应用，空闲回调不再重复
    ApplyPanelFramesLock( locked );
}

std::map<std::string, FBModel*> RigLogicHeadConstraint::CollectCharacterModels()
{
    // 与绑定一致：按骨架根所属 namespace 查找，多角色场景只处理本角色的面板
    FBModel* skelRoot = (FBModel*)ReferenceGet( mGroupSkeleton, 0 );
    const std::string ns = skelRoot ? ExtractNamespace( skelRoot ) : std::string();
    std::map<std::string, FBModel*> models;
    FBSystem lSystem;
    if (lSystem.Scene && lSystem.Scene->RootModel)
        CollectModelsNs( lSystem.Scene->RootModel, ns, models );
    return models;
}

std::size_t RigLogicHeadConstraint::ApplyPanelFramesLock( bool locked )
{
    std::size_t count = 0;
    for (const auto& kv : CollectCharacterModels())
    {
        if (!moburiglogic::IsPanelFrameName( kv.first )) continue;
        kv.second->Pickable = !locked;
        ++count;
    }
    FBTrace( "[RigLogicHead] Lock Panel Frames=%d: %zu FRM_* models\n", locked ? 1 : 0, count );
    return count;
}

// 面部表情回到中立（Zero Expressions 按钮）：
//   - FaceBoard 面板：DNA 读取的全部 GUI 控制器（CTRL_C_jaw 等）位移归零。
//     控制器集合取自 DNA，面板框（CTRL_faceGUI / CTRL_faceAndEyesAimFollowHeadGUI /
//     CTRL_faceTweakersGUI）及开关类 CTRL_* 不被 DNA 读取，因此不会被移动
//   - 表情属性模式：约束上的表情属性归零
// 两种模式都处理，切换输入模式后也能一键回中立；整体为一次可撤销操作
void RigLogicHeadConstraint::ZeroFaceControls()
{
    std::vector<FBModel*> controls;
    if (mReader)
    {
        std::vector<std::string> guiNames;
        const std::uint16_t guiCount = mReader->getGUIControlCount();
        guiNames.reserve( guiCount );
        for (std::uint16_t g = 0; g < guiCount; ++g)
        {
            auto sv = mReader->getGUIControlName( g );
            guiNames.emplace_back( sv.data(), sv.size() );
        }

        // 与绑定一致：按骨架根所属 namespace 查找，多角色场景只动本角色的面板
        const auto models = CollectCharacterModels();

        for (const auto& name : moburiglogic::GuiControlModelNames( guiNames ))
        {
            auto it = models.find( name );
            if (it != models.end()) controls.push_back( it->second );
        }
    }

    FBUndoManager& undo = FBUndoManager::TheOne();
    const bool ownTransaction = !undo.TransactionIsOpen()
                             && undo.TransactionBegin( "RigLogic Zero Expressions" );
    for (FBModel* control : controls)
    {
        undo.TransactionAddModelTRS( control );
        control->Translation = FBVector3d( 0.0, 0.0, 0.0 );
    }
    for (const auto& ei : mExprInputs)
    {
        undo.TransactionAddProperty( ei.prop );
        double v = 0.0;
        ei.prop->SetData( &v );
    }
    if (ownTransaction) undo.TransactionEnd();

    FBTrace( "[RigLogicHead] Zero Expressions: %zu panel controls, %zu expression properties\n",
             controls.size(), mExprInputs.size() );
}

void RigLogicHeadConstraint::FBDestroy()
{
    FBSystem().OnUIIdle.Remove( this, (FBCallback)&RigLogicHeadConstraint::EventUIIdle );
    ReleaseDna();
}

bool RigLogicHeadConstraint::LoadDna()
{
    const char* prop = DnaPath.AsString();
    const std::string propertyPath = prop ? prop : "";

    // 候选顺序：DNA Path 属性 → 绑定 FBX 同目录的 head.dna。
    // 回退命中后把实际路径写回 DnaPath，之后这个约束就一直带着可用路径。
    // 注意：不能用“文件是否存在”预判，统一交给 LoadRigRuntime，
    // 路径编码、相对路径等行为与原来的 DnaPath 完全一致
    const auto candidates = moburiglogic::DnaCandidates(
        propertyPath, moburiglogic::SceneFilePath(), kSceneFolderDnaName );

    moburiglogic::RigRuntime fresh;
    std::string resolvedPath;
    for (const auto& candidate : candidates)
    {
        if (moburiglogic::LoadRigRuntime( candidate.c_str(), fresh ))
        {
            resolvedPath = candidate;
            break;
        }
    }
    const bool loaded = !resolvedPath.empty();
    const bool relocated = loaded && resolvedPath != propertyPath;
    const auto source = relocated ? moburiglogic::DnaSource::SceneFolder
                                  : moburiglogic::DnaSource::Property;

    // 耗时的读取/构建在锁外完成，锁内只交换指针：求值线程看不到半成品，
    // 也不会在旧运行时被销毁时还在用它
    moburiglogic::RigRuntime old;
    {
        std::lock_guard<std::mutex> lock( mSolveMutex );
        old = { mStream, mReader, mRig, mInst };
        mStream = fresh.stream;
        mReader = fresh.reader;
        mRig    = fresh.rig;
        mInst   = fresh.inst;
        mBindingsReady = false;   // 旧绑定的关节/通道下标属于旧 DNA
        mLastEvalId = -1;
    }
    moburiglogic::DestroyRigRuntime( old );
    if (!loaded)
    {
        mLoadedDnaPath.clear();
        mLoadedDnaPropertyPath.clear();
        mDnaSource = moburiglogic::DnaSource::Property;
        // 通知（弹窗）委托给空闲队列：解析回调里不能弹模态框。
        // 只在整个候选集都试过（即场景同目录也试过）时才报“未找到”，
        // 否则加载早期场景路径尚未就绪会造成假告警
        if (ReferenceGet( mGroupSkeleton, 0 ) && candidates.size() > 1u)
        {
            moburiglogic::DnaNotice notice;
            notice.relocated = false;
            notice.node   = std::string( Name.AsString() );
            notice.ns     = CharacterNamespace();
            notice.oldPath = propertyPath;
            notice.tried  = candidates;
            moburiglogic::DnaNoticeQueue::Push( notice );
        }
        return false;
    }

    const auto lodCount = mReader->getLODCount();
    LodLevel.SetMinMax( 0.0, static_cast<double>( lodCount - 1u ), true, true );
    const auto validLod = moburiglogic::ClampLod( static_cast<int>( LodLevel ), lodCount );
    LodLevel = static_cast<int>( validLod );
    mInst->setLOD( validLod );
    mLoadedDnaPath = resolvedPath;
    mLoadedDnaPropertyPath = resolvedPath;   // 写回后属性值就等于实际路径
    mDnaSource = source;

    // 属性写入放到最后：它可能触发 MoBu 通知/重建绑定，此时内部状态已一致，不会递归重载
    if (relocated)
    {
        DnaPath = resolvedPath.c_str();             // DNA Path 始终显示实际使用的文件
        FBTrace( "[RigLogicHead] DNA Path unavailable ('%s'), using scene folder DNA: %s\n",
                 propertyPath.c_str(), resolvedPath.c_str() );

        // 提示用户“已自动改写，请保存”：同样交给空闲队列，不在这里弹窗
        moburiglogic::DnaNotice notice;
        notice.relocated = true;
        notice.node    = std::string( Name.AsString() );
        notice.ns      = CharacterNamespace();
        notice.oldPath = propertyPath;
        notice.newPath = resolvedPath;
        moburiglogic::DnaNoticeQueue::Push( notice );
    }
    return true;
}

std::string RigLogicHeadConstraint::CharacterNamespace() const
{
    FBModel* root = static_cast<FBModel*>(
        const_cast<RigLogicHeadConstraint*>( this )->ReferenceGet( mGroupSkeleton, 0 ) );
    return root ? moburiglogic::ExtractNamespace( root ) : std::string();
}

void RigLogicHeadConstraint::ReleaseDna()
{
    moburiglogic::RigRuntime old;
    {
        std::lock_guard<std::mutex> lock( mSolveMutex );
        old = { mStream, mReader, mRig, mInst };
        mStream = nullptr; mReader = nullptr; mRig = nullptr; mInst = nullptr;
        mBindingsReady = false;
    }
    moburiglogic::DestroyRigRuntime( old );
    mLoadedDnaPath.clear();
    mLoadedDnaPropertyPath.clear();
    mEyeLinkNode = nullptr;
    mAppliedLod = -1;
}

std::uint16_t RigLogicHeadConstraint::ResolveLod() const
{
    if (!mReader) return 0;
    return moburiglogic::ClampLod(
        static_cast<int>( LodLevel ), mReader->getLODCount() );
}

bool RigLogicHeadConstraint::BuildBindings( std::uint16_t lod )
{
    mExprInputs.clear();
    mGuiInputs.clear();
    mNeckInputs.clear();
    mJointOutputs.clear();
    mBsOutputs.clear();
    mOutputRoutes.clear();
    mEyeLinkNode = nullptr;
    mBindingsReady = false;
    mMissingBindings = 0;

    FBModel* skelRoot = (FBModel*)ReferenceGet( mGroupSkeleton, 0 );
    if (!skelRoot || !mReader || !mRig || !mInst) return false;

    FBModel* top = skelRoot;
    while (top->Parent) top = top->Parent;

    // namespace 感知：以引用对象的 namespace 为准，只认本角色的对象
    const std::string ns = ExtractNamespace( skelRoot );
    std::map<std::string, FBModel*> models;
    CollectModelsNs( top, ns, models );

    // 网格（BS 宿主）与面板控制器通常不在骨架层级下 —— 场景根补充收集（同样按 ns 过滤）
    FBSystem lSystem;
    if (lSystem.Scene && lSystem.Scene->RootModel)
        CollectModelsNs( lSystem.Scene->RootModel, ns, models );

    std::map<std::string, std::uint16_t> jointIndex;
    const std::uint16_t jointCount = mReader->getJointCount();
    for (std::uint16_t j = 0; j < jointCount; ++j)
    {
        auto sv = mReader->getJointName( j );
        jointIndex[ std::string( sv.data(), sv.size() ) ] = j;
    }

    // ---- 输入：按模式二选一（表情属性 / FaceBoard 面板），颈部四元数两模式共用 ----
    const bool faceboardMode = ( (int)InputMode == 1 );

    if (faceboardMode)
    {
        // FaceBoard 模式：DNA GUI control 名 "CTRL_C_jaw.ty" → 场景控制器 Lcl Translation 轴
        const std::uint16_t guiCount = mReader->getGUIControlCount();
        for (std::uint16_t g = 0; g < guiCount; ++g)
        {
            auto sv = mReader->getGUIControlName( g );
            std::string name( sv.data(), sv.size() );
            const auto dot = name.rfind('.');
            if (dot == std::string::npos || dot + 2 > name.size()) continue;
            const std::string ctrlName = name.substr( 0, dot );
            const char axisChar = name[ dot + 2 ];   // t"x"/t"y"/t"z"
            const int axis = axisChar == 'x' ? 0 : (axisChar == 'y' ? 1 : 2);

            auto itModel = models.find( ctrlName );
            if (itModel == models.end()) continue;   // 面板未合入场景则跳过该通道

            GuiInput gi;
            gi.guiIndex = g;
            gi.axis = axis;
            gi.eyeStick = ( ctrlName == kLeftEyeStick || ctrlName == kRightEyeStick );
            gi.node = AnimationNodeOutCreate( 500 + g, itModel->second,
                                              ANIMATIONNODE_TYPE_LOCAL_TRANSLATION );
            if (!gi.node) continue;   // 创建失败：跳过该通道（否则求值时空指针）
            mGuiInputs.push_back( gi );
        }

        // 双眼联动：CTRL_C_eye 不在 DNA 的 GUI 通道里（只有 CTRL_C_eye_parallelLook），
        // 单独读它的位移，求值时叠加到左右眼摇杆上
        auto itCEye = models.find( kCentreEyeStick );
        if (itCEye != models.end())
            mEyeLinkNode = AnimationNodeOutCreate( 900, itCEye->second,
                                                   ANIMATIONNODE_TYPE_LOCAL_TRANSLATION );
    }

    const std::uint16_t rawCount = mReader->getRawControlCount();
    for (std::uint16_t i = 0; i < rawCount; ++i)
    {
        auto sv = mReader->getRawControlName( i );
        std::string name( sv.data(), sv.size() );

        if (!faceboardMode && name.rfind( "CTRL_expressions.", 0 ) == 0)
        {
            // 表情控制 → 约束上的可K帧动态属性（短名，如 jawOpen）
            const std::string shortName = name.substr( name.find('.') + 1 );
            FBProperty* p = PropertyList.Find( shortName.c_str() );
            if (!p)
            {
                p = PropertyCreate( shortName.c_str(), kFBPT_double, "Number",
                                    /*animatable=*/true, /*user=*/true );
            }
            if (p)
                mExprInputs.push_back( { p, i } );
        }
        else if (name.size() > 3 && name.compare( name.size()-3, 3, ".qx" ) == 0)
        {
            // 颈部四元数四连组（qx 起始）
            const std::string jname = name.substr( 0, name.size()-3 );
            auto itModel = models.find( jname );
            auto itJoint = jointIndex.find( jname );
            // 场景里找不到该关节：统计为缺失（头部 DNA 对不上绑定时只告警，不阻断）
            if (itModel == models.end()) { ++mMissingBindings; continue; }
            if (itJoint == jointIndex.end()) continue;

            NeckInput ni;
            ni.rawBase = i;
            ni.node = AnimationNodeOutCreate( 1000 + i, itModel->second,
                                              ANIMATIONNODE_TYPE_ROTATION );
            if (!ni.node) continue;   // 创建失败：跳过该关节
            ni.parentNode = itModel->second->Parent
                ? AnimationNodeOutCreate( 2000 + i, itModel->second->Parent,
                                          ANIMATIONNODE_TYPE_ROTATION )
                : nullptr;
            auto nr = mReader->getNeutralJointRotation( itJoint->second );
            const double neutralE[3] = { nr.x, nr.y, nr.z };
            double qNeutral[4];
            EulerDegToQuat( neutralE, qNeutral );
            QuatConj( qNeutral, ni.qNeutralInv );
            mNeckInputs.push_back( ni );
        }
    }

    // ---- 输出①：面部关节（驱动集过滤，跳过根与输入关节）----
    auto varAttrs = mRig->getJointVariableAttributeIndices( lod );
    std::map<std::uint32_t, bool> driven;
    for (auto a : varAttrs) driven[ a / 9 ] = true;

    for (auto& kv : driven)
    {
        const std::uint32_t j = kv.first;
        const auto j16 = static_cast<std::uint16_t>( j );
        if (mReader->getJointParentIndex( j16 ) == j) continue;
        auto sv = mReader->getJointName( j16 );
        std::string jname( sv.data(), sv.size() );
        auto itModel = models.find( jname );
        if (itModel == models.end()) { ++mMissingBindings; continue; }
        FBModel* m = itModel->second;

        JointOutput jo;
        jo.jointIndex = j;
        auto nt = mReader->getNeutralJointTranslation( j16 );
        jo.neutralT[0] = nt.x; jo.neutralT[1] = nt.y; jo.neutralT[2] = nt.z;

        // 旋转策略：Pre-Rotation≈0 → 中立烘在 Lcl → 需合成（FaceMesh 资产惯例）
        FBVector3d pre = m->PreRotation;
        const double preE[3] = { pre[0], pre[1], pre[2] };
        auto nr = mReader->getNeutralJointRotation( j16 );
        const double neutralE[3] = { nr.x, nr.y, nr.z };
        jo.composeRot = NeedsNeutralCompose( preE, neutralE );
        EulerDegToQuat( neutralE, jo.qNeutral );

        jo.nodeT = AnimationNodeInCreate( 3000 + j*3 + 0, m, ANIMATIONNODE_TYPE_LOCAL_TRANSLATION );
        jo.nodeR = AnimationNodeInCreate( 3000 + j*3 + 1, m, ANIMATIONNODE_TYPE_LOCAL_ROTATION );
        jo.nodeS = nullptr;   // head.dna 无缩放输出（实测 s:0）
        mJointOutputs.push_back( jo );
        const auto bindingIndex =
            static_cast<std::uint32_t>( mJointOutputs.size() - 1u );
        if (jo.nodeT) mOutputRoutes.emplace(
            jo.nodeT, moburiglogic::OutputRoute {
                moburiglogic::OutputKind::JointTranslation, bindingIndex } );
        if (jo.nodeR) mOutputRoutes.emplace(
            jo.nodeR, moburiglogic::OutputRoute {
                moburiglogic::OutputKind::JointRotation, bindingIndex } );
        if (jo.nodeS) mOutputRoutes.emplace(
            jo.nodeS, moburiglogic::OutputRoute {
                moburiglogic::OutputKind::JointScaling, bindingIndex } );
    }

    // ---- 输出②：按 DNA mesh-channel mapping 创建独立 BlendShape 输出 ----
    const auto mappingIndices =
        mReader->getMeshBlendShapeChannelMappingIndicesForLOD( lod );
    const auto mappings = moburiglogic::CollectBlendShapeMappings(
        mappingIndices,
        [this]( std::uint16_t mappingIndex ) {
            const auto mapping =
                mReader->getMeshBlendShapeChannelMapping( mappingIndex );
            return moburiglogic::BlendShapeMappingRef {
                mapping.meshIndex,
                mapping.blendShapeChannelIndex
            };
        } );

    // 每网格缓存宿主与命名风格（首个通道解析，后续通道复用，避免反复全场景扫描）
    std::map<std::uint16_t, moburiglogic::BlendShapePropertyMatch<FBModel*>> blendShapeHosts;
    const auto hasAnimatableProperty =
        []( FBModel* model, const std::string& candidateProperty ) {
            FBProperty* property =
                model->PropertyList.Find( candidateProperty.c_str() );
            return property && property->IsAnimatable();
        };
    std::size_t mappingOrdinal = 0;
    for (const auto& mapping : mappings)
    {
        auto meshNameView = mReader->getMeshName( mapping.meshIndex );
        const std::string meshName( meshNameView.data(), meshNameView.size() );
        auto channelNameView =
            mReader->getBlendShapeChannelName( mapping.channelIndex );
        const std::string channelName(
            channelNameView.data(), channelNameView.size() );

        moburiglogic::BlendShapePropertyMatch<FBModel*> match;
        const auto cachedHost = blendShapeHosts.find( mapping.meshIndex );
        if (cachedHost != blendShapeHosts.end()) {
            match = cachedHost->second;
            match.propertyName = match.prefixed
                ? meshName + "__" + channelName
                : channelName;
        } else {
            match = moburiglogic::ResolveBlendShapeProperty(
                models, meshName, channelName, hasAnimatableProperty );
            if (match.owner) {
                blendShapeHosts.emplace( mapping.meshIndex, match );
            }
        }

        FBProperty* property = match.owner
            ? match.owner->PropertyList.Find( match.propertyName.c_str() )
            : nullptr;
        if (!property || !property->IsAnimatable()) {
            ++mappingOrdinal;
            continue;
        }

        const auto userId = static_cast<int>( 6000u + mappingOrdinal );
        FBAnimationNode* node = AnimationNodeInCreate( userId, property );
        if (node) {
            mBsOutputs.push_back( { node, mapping.channelIndex } );
            const auto bindingIndex =
                static_cast<std::uint32_t>( mBsOutputs.size() - 1u );
            mOutputRoutes.emplace(
                node, moburiglogic::OutputRoute {
                    moburiglogic::OutputKind::BlendShape, bindingIndex } );
        }
        ++mappingOrdinal;
    }

    const auto requestedMappings = mappings.size();
    const auto boundMappings = mBsOutputs.size();
    FBTrace( "[RigLogicHead] BlendShape mappings: requested=%zu bound=%zu missing=%zu\n",
             requestedMappings,
             boundMappings,
             requestedMappings - boundMappings );
    if (mMissingBindings > 0)
        FBTrace( "[RigLogicHead] WARNING: %d DNA joints not found in scene (DNA may not match this rig)\n",
                 mMissingBindings );

    mBindingsReady = ( !mExprInputs.empty() || !mGuiInputs.empty() )
                  && !mJointOutputs.empty();
    return mBindingsReady;
}

void RigLogicHeadConstraint::SetupAllAnimationNodes()
{
    if (!ReferenceGet( mGroupSkeleton, 0 )) return;
    // 播放中不重建绑定/不重读 DNA：先立即失效旧绑定（节点可能已被 MoBu 销毁），
    // 停止后由 UI 空闲补做。继续用旧绑定会解引用已释放的 connector（实测崩溃点）
    if (moburiglogic::IsTransportPlaying())
    {
        InvalidateBindings();
        mRebuildPending = true;
        return;
    }
    // 属性路径变化或未加载时（重新）加载 DNA —— 修复换 DNA 后仍用旧数据
    const char* p = DnaPath.AsString();
    const std::string want = p ? p : "";
    if ((!mRig || want != mLoadedDnaPropertyPath) && !LoadDna()) return;

    const auto validLod = ResolveLod();
    LodLevel = static_cast<int>( validLod );
    mInst->setLOD( validLod );
    BuildBindings( validLod );
    mAppliedLod = static_cast<int>( validLod );
    mLastEvalId = -1;
}

bool RigLogicHeadConstraint::RebuildBindings()
{
    FBModel* root = static_cast<FBModel*>( ReferenceGet( mGroupSkeleton, 0 ) );
    if (!root) return false;

    // 播放/求值中重建会销毁并重建动画节点，与求值线程并发时可能崩——延后到停止后由 UI 空闲补做
    if (moburiglogic::IsTransportPlaying())
    {
        InvalidateBindings();
        mRebuildPending = true;
        return false;
    }
    mRebuildPending = false;

    const bool wasActive = static_cast<bool>( Active );
    Active = false;

    if (!ReferenceRemove( mGroupSkeleton, root ))
    {
        Active = wasActive;
        return false;
    }
    if (!ReferenceAdd( mGroupSkeleton, root ))
    {
        return false;
    }

    if (wasActive && mBindingsReady) Active = true;
    return mBindingsReady;
}

void RigLogicHeadConstraint::RemoveAllAnimationNodes()
{
    // MoBu 正在拆我们的节点：必须在锁内清空，否则求值线程会继续解引用已释放节点
    InvalidateBindings();
}

void RigLogicHeadConstraint::InvalidateBindings()
{
    std::lock_guard<std::mutex> lock( mSolveMutex );
    mExprInputs.clear();
    mGuiInputs.clear();
    mNeckInputs.clear();
    mJointOutputs.clear();
    mBsOutputs.clear();
    mOutputRoutes.clear();
    mEyeLinkNode = nullptr;
    mBindingsReady = false;
    mLastEvalId = -1;
}

bool RigLogicHeadConstraint::AnimationNodeNotify( FBAnimationNode* pConnector,
                                                  FBEvaluateInfo* pEvaluateInfo,
                                                  FBConstraintInfo* pConstraintInfo )
{
    // 锁的边界（并发安全的关键）：
    //   * 绑定表（mGuiInputs/mJointOutputs/mBsOutputs/mOutputRoutes）只允许在锁内访问：
    //     主线程重建会 clear() 这些容器，求值线程无锁迭代/取引用会造成悬垂与迭代器失效
    //   * ReadData/GetData/WriteData 一律在锁外调用：读可能触发上游（如 HIK）求值
    //     并回到本约束，持锁读取会自锁死
    // 做法：锁内“拷出输入节点快照 + 取绑定信息 + 拷结果”，锁外做实际读写
    const long evalId = pEvaluateInfo->GetEvaluationID();

    std::vector<GuiInput>  guiInputs;
    std::vector<ExprInput> exprInputs;
    std::vector<NeckInput> neckInputs;
    FBAnimationNode* eyeLinkNode = nullptr;
    bool needSolve;
    {
        std::lock_guard<std::mutex> lock( mSolveMutex );
        if (!mBindingsReady || !mRig || !mInst) return false;
        needSolve = ( evalId != mLastEvalId );
        if (needSolve)
        {
            guiInputs  = mGuiInputs;    // 元素是 POD + 节点指针，拷贝开销很小
            exprInputs = mExprInputs;
            neckInputs = mNeckInputs;
            eyeLinkNode = mEyeLinkNode;
        }
    }

    if (needSolve)
    {
        const auto t0 = std::chrono::steady_clock::now();

        // FaceBoard 模式：面板位移；表情属性模式：属性值（支持K帧动画取值）
        std::vector<float> guiValues( guiInputs.size() );
        // 眼睛联动：CTRL_C_eye 的位移与左右眼摇杆在 Maya 里是角度相加（blendWeighted），
        // DNA 的摇杆→激活值映射在 [-1,1] 内是线性的，所以直接在摇杆域相加等价
        double eyeLink[3] = { 0.0, 0.0, 0.0 };
        const bool eyeLinkOn = eyeLinkNode && static_cast<bool>( EyeLinkage );
        if (eyeLinkOn) eyeLinkNode->ReadData( eyeLink, pEvaluateInfo );
        for (std::size_t i = 0; i < guiInputs.size(); ++i)
        {
            if (!guiInputs[i].node) continue;   // 节点创建失败：跳过该通道，不空指针崩溃
            double t[3] = { 0, 0, 0 };
            guiInputs[i].node->ReadData( t, pEvaluateInfo );
            double v = t[ guiInputs[i].axis ];
            if (guiInputs[i].eyeStick)
            {
                if (eyeLinkOn) v += eyeLink[ guiInputs[i].axis ];
                v = moburiglogic::ClampEyeStick( v );   // DNA 域是 [-1,1]，越界会被外插成 0
            }
            guiValues[i] = static_cast<float>( v );
        }
        std::vector<float> exprValues;
        if (guiInputs.empty())
        {
            exprValues.resize( exprInputs.size() );
            for (std::size_t i = 0; i < exprInputs.size(); ++i)
            {
                if (!exprInputs[i].prop) continue;
                double v = 0.0;
                exprInputs[i].prop->GetData( &v, sizeof(v), pEvaluateInfo );
                exprValues[i] = static_cast<float>( v );
            }
        }
        std::vector<double> neckQuats( neckInputs.size() * 4u );
        for (std::size_t i = 0; i < neckInputs.size(); ++i)
        {
            const auto& ni = neckInputs[i];
            double globalE[3] = { 0, 0, 0 }, parentE[3] = { 0, 0, 0 };
            if (!ni.node) continue;
            ni.node->ReadData( globalE, pEvaluateInfo );
            if (ni.parentNode) ni.parentNode->ReadData( parentE, pEvaluateInfo );
            DriverRelativeQuat( ni.qNeutralInv, parentE, globalE, &neckQuats[ i * 4u ] );
        }

        std::lock_guard<std::mutex> lock( mSolveMutex );
        if (!mBindingsReady || !mRig || !mInst) return false;
        if (evalId != mLastEvalId)   // 并发时可能已被其他线程解完
        {
            if (!mGuiInputs.empty())
            {
                for (std::size_t i = 0; i < guiInputs.size(); ++i)
                    mInst->setGUIControl( guiInputs[i].guiIndex, guiValues[i] );
                mRig->mapGUIToRawControls( mInst );   // 双向拆分/相位/量程全在 DNA 里
            }
            else
            {
                for (std::size_t i = 0; i < exprInputs.size(); ++i)
                    mInst->setRawControl( exprInputs[i].rawIndex, exprValues[i] );
            }
            // 颈部四元数在 GUI 映射之后写入，不被覆盖
            for (std::size_t i = 0; i < neckInputs.size(); ++i)
                for (std::uint16_t k = 0; k < 4u; ++k)
                    mInst->setRawControl( neckInputs[i].rawBase + k,
                                          static_cast<float>( neckQuats[ i * 4u + k ] ) );
            mRig->calculate( mInst );

            mLastSolveMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0 ).count();
            mLastEvalId = evalId;
        }
    }

    // ---- 输出：绑定信息 + 求解结果全在锁内取出，WriteData 在锁外 ----
    moburiglogic::OutputKind outKind = moburiglogic::OutputKind::BlendShape;
    FBAnimationNode* outNode = nullptr;
    FBAnimationNode* outNodeS = nullptr;
    double bsValue = 0.0;
    double jointDelta[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    double outNeutralT[3] = { 0, 0, 0 };
    double outQNeutral[4] = { 0, 0, 0, 1 };
    bool outComposeRot = false;
    {
        std::lock_guard<std::mutex> lock( mSolveMutex );
        if (!mBindingsReady || !mInst) return false;
        const auto routeIt = mOutputRoutes.find( pConnector );
        if (routeIt == mOutputRoutes.end()) return false;
        const auto route = routeIt->second;
        outKind = route.kind;

        if (route.kind == moburiglogic::OutputKind::BlendShape)
        {
            if (route.bindingIndex >= mBsOutputs.size()) return false;
            const auto& out = mBsOutputs[ route.bindingIndex ];
            outNode = out.node;
            const auto bs = mInst->getBlendShapeOutputs();
            bsValue = out.channel < bs.size() ? bs[ out.channel ] * 100.0 : 0.0;
        }
        else
        {
            if (route.bindingIndex >= mJointOutputs.size()) return false;
            const auto& out = mJointOutputs[ route.bindingIndex ];
            outNode   = out.nodeT;
            outNodeS  = out.nodeS;
            outComposeRot = out.composeRot;
            for (int k = 0; k < 3; ++k) outNeutralT[k] = out.neutralT[k];
            for (int k = 0; k < 4; ++k) outQNeutral[k] = out.qNeutral[k];
            const auto jointOutputs = mInst->getJointOutputs();
            const std::size_t base = static_cast<std::size_t>( out.jointIndex ) * 9u;
            if (base + 9u > jointOutputs.size()) return false;
            for (std::size_t k = 0; k < 9u; ++k) jointDelta[k] = jointOutputs[base + k];
            if (route.kind == moburiglogic::OutputKind::JointRotation) outNode = out.nodeR;
        }
    }

    if (outKind == moburiglogic::OutputKind::BlendShape)
    {
        if (!outNode) return false;
        outNode->WriteData( &bsValue, pEvaluateInfo );
        return true;
    }

    // 节点可能因重建而失效（重建受“播放中不允许”守卫，理论上不会），判空兜底
    if (!outNode) return false;

    switch (outKind)
    {
    case moburiglogic::OutputKind::JointTranslation:
    {
        double value[3] = {
            outNeutralT[0] + jointDelta[0],
            outNeutralT[1] + jointDelta[1],
            outNeutralT[2] + jointDelta[2]
        };
        outNode->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::JointRotation:
    {
        const double deltaEuler[3] = { jointDelta[3], jointDelta[4], jointDelta[5] };
        double value[3];
        ComposeJointRotation( outComposeRot, outQNeutral, deltaEuler, value );
        outNode->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::JointScaling:
    {
        if (!outNodeS) return false;
        double value[3] = { 1.0 + jointDelta[6], 1.0 + jointDelta[7], 1.0 + jointDelta[8] };
        outNodeS->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::BlendShape:
        break;
    }
    return false;
}

bool RigLogicHeadConstraint::FbxStore( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat )
{
    return true;
}

bool RigLogicHeadConstraint::FbxRetrieve( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat )
{
    if (pStoreWhat == kCleanup)
    {
        // 打开新文件：清空通知去重表（待发队列不动，加载与文件事件的先后顺序不可靠）
        moburiglogic::DnaNoticeQueue::OnSceneLoaded();
        // 场景加载完成后重建运行时；若 SetupAllAnimationNodes 已按同一属性路径加载过则跳过，
        // 避免重复读取 DNA 并在求值可能已开始时替换运行时。
        // 属性路径打不开时会回退到 FBX 同目录的 head.dna（棚内/外拍场景）
        const char* path = DnaPath.AsString();
        const std::string want = path ? path : "";
        if ((!mRig || mLoadedDnaPropertyPath != want) && !LoadDna())
            FBTrace( "[RigLogicHead] DNA not loaded from '%s' or scene folder %s\n",
                     want.c_str(), kSceneFolderDnaName );
    }
    return true;
}
