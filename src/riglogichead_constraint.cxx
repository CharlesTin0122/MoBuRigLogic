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

bool RigLogicHeadConstraint::FBCreate()
{
    FBPropertyPublish( this, DnaPath,     "DNA Path",      nullptr, nullptr );
    FBPropertyPublish( this, LodLevel,    "LOD Level",     nullptr, nullptr );
    FBPropertyPublish( this, InputMode,   "Input Mode",    nullptr, nullptr );
    FBPropertyPublish( this, LastSolveMs, "Last Solve Ms", nullptr, nullptr );
    FBPropertyPublish( this, LockPanelFrames, "Lock Panel Frames", nullptr, nullptr );
    LodLevel = 0;
    InputMode = 0;   // 0=表情属性 1=FaceBoard面板
    LastSolveMs = 0.0;
    LockPanelFrames = false;

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
    // 先记下目标 LOD 再重建，重建失败也不会每个空闲周期反复重试
    const int lod = static_cast<int>( LodLevel );
    if (mRig && mAppliedLod >= 0 && lod != mAppliedLod)
    {
        mAppliedLod = lod;
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
    const char* path = DnaPath.AsString();
    moburiglogic::RigRuntime fresh;
    const bool loaded = path && *path && moburiglogic::LoadRigRuntime( path, fresh );

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
        return false;
    }

    const auto lodCount = mReader->getLODCount();
    LodLevel.SetMinMax( 0.0, static_cast<double>( lodCount - 1u ), true, true );
    const auto validLod = moburiglogic::ClampLod( static_cast<int>( LodLevel ), lodCount );
    LodLevel = static_cast<int>( validLod );
    mInst->setLOD( validLod );
    mLoadedDnaPath = path;
    return true;
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
    mBindingsReady = false;

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
            gi.node = AnimationNodeOutCreate( 500 + g, itModel->second,
                                              ANIMATIONNODE_TYPE_LOCAL_TRANSLATION );
            mGuiInputs.push_back( gi );
        }
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
            if (itModel == models.end() || itJoint == jointIndex.end()) continue;

            NeckInput ni;
            ni.rawBase = i;
            ni.node = AnimationNodeOutCreate( 1000 + i, itModel->second,
                                              ANIMATIONNODE_TYPE_ROTATION );
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
        if (itModel == models.end()) continue;
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

    mBindingsReady = ( !mExprInputs.empty() || !mGuiInputs.empty() )
                  && !mJointOutputs.empty();
    return mBindingsReady;
}

void RigLogicHeadConstraint::SetupAllAnimationNodes()
{
    if (!ReferenceGet( mGroupSkeleton, 0 )) return;
    // 路径变化或未加载时（重新）加载 DNA —— 修复换 DNA 后仍用旧数据
    const char* p = DnaPath.AsString();
    const std::string want = p ? p : "";
    if ((!mRig || want != mLoadedDnaPath) && !LoadDna()) return;

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
    mExprInputs.clear();
    mGuiInputs.clear();
    mNeckInputs.clear();
    mJointOutputs.clear();
    mBsOutputs.clear();
    mOutputRoutes.clear();
    mLastEvalId = -1;
    mBindingsReady = false;
}

bool RigLogicHeadConstraint::AnimationNodeNotify( FBAnimationNode* pConnector,
                                                  FBEvaluateInfo* pEvaluateInfo,
                                                  FBConstraintInfo* pConstraintInfo )
{
    // 锁只包住纯 RigLogic 计算与结果拷贝，不包 ReadData/GetData/WriteData：
    // 读取可能触发上游（如 HIK 解算）求值并回到本约束，持锁读取会自锁死
    const long evalId = pEvaluateInfo->GetEvaluationID();
    bool needSolve;
    {
        std::lock_guard<std::mutex> lock( mSolveMutex );
        if (!mBindingsReady || !mRig || !mInst) return false;
        needSolve = ( evalId != mLastEvalId );
    }

    if (needSolve)
    {
        const auto t0 = std::chrono::steady_clock::now();

        // FaceBoard 模式：面板位移；表情属性模式：属性值（支持K帧动画取值）
        std::vector<float> guiValues( mGuiInputs.size() );
        for (std::size_t i = 0; i < mGuiInputs.size(); ++i)
        {
            double t[3] = { 0, 0, 0 };
            mGuiInputs[i].node->ReadData( t, pEvaluateInfo );
            guiValues[i] = static_cast<float>( t[ mGuiInputs[i].axis ] );
        }
        std::vector<float> exprValues;
        if (mGuiInputs.empty())
        {
            exprValues.resize( mExprInputs.size() );
            for (std::size_t i = 0; i < mExprInputs.size(); ++i)
            {
                double v = 0.0;
                mExprInputs[i].prop->GetData( &v, sizeof(v), pEvaluateInfo );
                exprValues[i] = static_cast<float>( v );
            }
        }
        std::vector<double> neckQuats( mNeckInputs.size() * 4u );
        for (std::size_t i = 0; i < mNeckInputs.size(); ++i)
        {
            const auto& ni = mNeckInputs[i];
            double globalE[3] = { 0, 0, 0 }, parentE[3] = { 0, 0, 0 };
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
                for (std::size_t i = 0; i < mGuiInputs.size(); ++i)
                    mInst->setGUIControl( mGuiInputs[i].guiIndex, guiValues[i] );
                mRig->mapGUIToRawControls( mInst );   // 双向拆分/相位/量程全在 DNA 里
            }
            else
            {
                for (std::size_t i = 0; i < mExprInputs.size(); ++i)
                    mInst->setRawControl( mExprInputs[i].rawIndex, exprValues[i] );
            }
            // 颈部四元数在 GUI 映射之后写入，不被覆盖
            for (std::size_t i = 0; i < mNeckInputs.size(); ++i)
                for (std::uint16_t k = 0; k < 4u; ++k)
                    mInst->setRawControl( mNeckInputs[i].rawBase + k,
                                          static_cast<float>( neckQuats[ i * 4u + k ] ) );
            mRig->calculate( mInst );

            mLastSolveMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0 ).count();
            mLastEvalId = evalId;
        }
    }

    const auto routeIt = mOutputRoutes.find( pConnector );
    if (routeIt == mOutputRoutes.end()) return false;
    const auto route = routeIt->second;

    if (route.kind == moburiglogic::OutputKind::BlendShape)
    {
        if (route.bindingIndex >= mBsOutputs.size()) return false;
        const auto& out = mBsOutputs[ route.bindingIndex ];
        double value = 0.0;
        {
            std::lock_guard<std::mutex> lock( mSolveMutex );
            if (!mInst) return false;
            const auto bs = mInst->getBlendShapeOutputs();
            value = out.channel < bs.size() ? bs[ out.channel ] * 100.0 : 0.0;
        }
        out.node->WriteData( &value, pEvaluateInfo );
        return true;
    }

    if (route.bindingIndex >= mJointOutputs.size()) return false;
    const auto& out = mJointOutputs[ route.bindingIndex ];
    float delta[9];
    {
        std::lock_guard<std::mutex> lock( mSolveMutex );
        if (!mInst) return false;
        const auto jointOutputs = mInst->getJointOutputs();
        const std::size_t base = static_cast<std::size_t>( out.jointIndex ) * 9u;
        if (base + 9u > jointOutputs.size()) return false;
        for (std::size_t k = 0; k < 9u; ++k) delta[k] = jointOutputs[base + k];
    }

    switch (route.kind)
    {
    case moburiglogic::OutputKind::JointTranslation:
    {
        double value[3] = {
            out.neutralT[0] + delta[0],
            out.neutralT[1] + delta[1],
            out.neutralT[2] + delta[2]
        };
        out.nodeT->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::JointRotation:
    {
        const double deltaEuler[3] = { delta[3], delta[4], delta[5] };
        double value[3];
        ComposeJointRotation( out.composeRot, out.qNeutral, deltaEuler, value );
        out.nodeR->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::JointScaling:
    {
        if (!out.nodeS) return false;
        double value[3] = { 1.0 + delta[6], 1.0 + delta[7], 1.0 + delta[8] };
        out.nodeS->WriteData( value, pEvaluateInfo );
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
        // 场景加载完成后重建运行时；若 SetupAllAnimationNodes 已按同一路径加载过则跳过，
        // 避免重复读取 DNA 并在求值可能已开始时替换运行时
        const char* path = DnaPath.AsString();
        if (path && *path && (!mRig || mLoadedDnaPath != path))
            LoadDna();
    }
    return true;
}
