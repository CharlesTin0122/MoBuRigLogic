/**
 * RigLogicBodyConstraint 实现（MotionBuilder 2024 + OpenRigLogic 静态库）
 *
 * 求值流程（每个 EvaluateInfo ID 只求解一次）：
 *   1. 读驱动关节及父级全局旋转 → q_rel = q中立⁻¹ * q父全局⁻¹ * q全局（锁外读取）
 *   2. setRawControl × 176 → rig->calculate(inst)
 *   3. 通知到的每个输出节点从 getJointOutputs() 取对应关节增量写出
 */

//--- Class declaration
#include "riglogicbody_constraint.h"
#include "riglogic_common.h"
#include "riglogic_scene.h"

//--- OpenRigLogic
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

//--- Registration defines
#define RIGLOGICBODY__CLASS   RIGLOGICBODY__CLASSNAME
#define RIGLOGICBODY__NAME    "RigLogic Body Corrective"
#define RIGLOGICBODY__LABEL   "RigLogic Body Corrective"
#define RIGLOGICBODY__DESC    "MetaHuman body corrective joints driven by RigLogic (body.dna)"

FBConstraintImplementation( RIGLOGICBODY__CLASS );
FBRegisterConstraint( RIGLOGICBODY__NAME,
                      RIGLOGICBODY__CLASS,
                      RIGLOGICBODY__LABEL,
                      RIGLOGICBODY__DESC,
                      FB_DEFAULT_SDK_ICON );

using moburiglogic::CollectModelsNs;
using moburiglogic::ComposeJointRotation;
using moburiglogic::DriverRelativeQuat;
using moburiglogic::EulerDegToQuat;
using moburiglogic::ExtractNamespace;
using moburiglogic::NeedsNeutralCompose;
using moburiglogic::QuatConj;

// 绑定 FBX 同目录下的固定文件名：公司路径不可用时用来兜底（棚内/外拍常用）
static const char* kSceneFolderDnaName = "body.dna";

/************************************************
 *  Creation
 ************************************************/
bool RigLogicBodyConstraint::FBCreate()
{
    // 属性（FBPropertyPublish 注册到 PropertyList，UI 可见且随 FBX 序列化）
    FBPropertyPublish( this, DnaPath,     "DNA Path",      nullptr, nullptr );
    FBPropertyPublish( this, SourceDnaPath, "Source DNA Path", nullptr, nullptr );
    FBPropertyPublish( this, LodLevel,    "LOD Level",     nullptr, nullptr );
    FBPropertyPublish( this, LastSolveMs, "Last Solve Ms", nullptr, nullptr );
    LodLevel = 0;
    LastSolveMs = 0.0;

    // 引用组：把骨架根（root/pelvis 所在层级任意节点）拖进来
    mGroupSkeleton = ReferenceGroupAdd( "Skeleton Root", 1 );

    Deformer = false;
    HasLayout = true;
    FBSystem().OnUIIdle.Add( this, (FBCallback)&RigLogicBodyConstraint::EventUIIdle );
    return true;
}

// 求值线程只写 mLastSolveMs 原子量；属性写入会触发通知，必须回到主线程
void RigLogicBodyConstraint::EventUIIdle( HISender, HKEvent )
{
    const double ms = mLastSolveMs.load();
    if ((double)LastSolveMs != ms) LastSolveMs = ms;

    // LOD 改动即时生效：驱动集/BS 映射随 LOD 变化，需重建绑定（不止 setLOD）。
    // 先记下目标 LOD 再重建，重建失败也不会每个空闲周期反复重试。
    // 播放中不重建，也不记 mAppliedLod，停止后自然会再触发一次
    const int lod = static_cast<int>( LodLevel );
    if (mRig && mAppliedLod >= 0 && lod != mAppliedLod
        && !moburiglogic::IsTransportPlaying())
    {
        mAppliedLod = lod;
        RebuildBindings();
    }

    // 播放中挂起的重建：停止后补做一次
    if (mRebuildPending && !moburiglogic::IsTransportPlaying())
    {
        mRebuildPending = false;
        RebuildBindings();
    }

    // DNA 加载失败时的兜底懒重试。FbxRetrieve(kCleanup) 先于场景文件名就绪的情况存在
    // （FBXFileName 为空 → 找不到 FBX 同目录的 body.dna），所以每遇到一组新的
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
}

void RigLogicBodyConstraint::FBDestroy()
{
    FBSystem().OnUIIdle.Remove( this, (FBCallback)&RigLogicBodyConstraint::EventUIIdle );
    ReleaseDna();
}

/************************************************
 *  DNA / RigLogic 生命周期
 ************************************************/
bool RigLogicBodyConstraint::LoadDna()
{
    const char* prop = DnaPath.AsString();
    const std::string propertyPath = prop ? prop : "";
    const char* prev = SourceDnaPath.AsString();
    const std::string previousPath = prev ? prev : "";

    // 候选顺序：DNA Path 属性 → 上次被替换的路径 → 绑定 FBX 同目录的 body.dna。
    // 回退命中后会把实际路径写回 DnaPath，公司原路径留在 SourceDnaPath
    // （随 FBX 保存），拿回公司时第二级自动把它恢复回来。
    // 注意：不能用“文件是否存在”预判，统一交给 LoadRigRuntime，
    // 路径编码、相对路径等行为与原来的 DnaPath 完全一致
    const auto candidates = moburiglogic::DnaCandidates(
        propertyPath, previousPath, moburiglogic::SceneFilePath(), kSceneFolderDnaName );

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
    const auto source =
        !loaded                ? moburiglogic::DnaSource::Property
        : resolvedPath == propertyPath ? moburiglogic::DnaSource::Property
        : (!previousPath.empty() && resolvedPath == previousPath)
                                     ? moburiglogic::DnaSource::Previous
                                     : moburiglogic::DnaSource::SceneFolder;

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
        mBindingsReady = false;   // 旧绑定的关节下标属于旧 DNA
        mLastEvalId = -1;
    }
    moburiglogic::DestroyRigRuntime( old );
    if (!loaded)
    {
        mLoadedDnaPath.clear();
        mLoadedDnaPropertyPath.clear();
        mDnaSource = moburiglogic::DnaSource::Property;
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
    if (source != moburiglogic::DnaSource::Property)
    {
        if (!propertyPath.empty() && propertyPath != resolvedPath)
            SourceDnaPath = propertyPath.c_str();   // 记住“刚从哪条路退回来”
        DnaPath = resolvedPath.c_str();             // DNA Path 始终显示实际使用的文件
        FBTrace( "[RigLogicBody] DNA Path unavailable ('%s'), using %s: %s\n",
                 propertyPath.c_str(),
                 source == moburiglogic::DnaSource::Previous
                     ? "Source DNA Path" : "scene folder DNA",
                 resolvedPath.c_str() );
    }
    return true;
}

void RigLogicBodyConstraint::ReleaseDna()
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
    mAppliedLod = -1;
}

std::uint16_t RigLogicBodyConstraint::ResolveLod() const
{
    if (!mReader) return 0;
    return moburiglogic::ClampLod(
        static_cast<int>( LodLevel ), mReader->getLODCount() );
}

/************************************************
 *  绑定构建：DNA 关节名 ↔ 场景模型
 ************************************************/
bool RigLogicBodyConstraint::BuildBindings( std::uint16_t lod )
{
    mInputs.clear();
    mOutputs.clear();
    mOutputRoutes.clear();
    mBindingsReady = false;
    mMissingBindings = 0;

    FBModel* skelRoot = (FBModel*)ReferenceGet( mGroupSkeleton, 0 );
    if (!skelRoot || !mReader || !mRig || !mInst) return false;

    // 从引用对象最顶层父级开始收集（允许用户拖任意骨骼进组）；
    // namespace 感知：多角色场景只认引用对象所属 namespace 的骨骼
    FBModel* top = skelRoot;
    while (top->Parent) top = top->Parent;
    const std::string ns = ExtractNamespace( skelRoot );
    std::map<std::string, FBModel*> models;
    CollectModelsNs( top, ns, models );

    // DNA 关节名 → 下标
    std::map<std::string, std::uint32_t> jointIndex;
    const std::uint16_t jointCount = mReader->getJointCount();
    for (std::uint16_t j = 0; j < jointCount; ++j)
        jointIndex[ std::string( mReader->getJointName(j).data(),
                                 mReader->getJointName(j).size() ) ] = j;

    // ---- 输入：raw controls 按 <joint>.qx qy qz qw 四连排布 ----
    const std::uint16_t rawCount = mReader->getRawControlCount();
    std::vector<std::string> inputJointNames;
    std::map<FBModel*, FBAnimationNode*> parentNodes;   // 多个驱动关节共用同一父级时只建一个节点
    for (std::uint16_t i = 0; i < rawCount; i += 4)
    {
        auto sv = mReader->getRawControlName( i );
        std::string name( sv.data(), sv.size() );
        const auto dot = name.find('.');
        if (dot == std::string::npos) continue;
        const std::string jname = name.substr( 0, dot );

        auto itModel = models.find( jname );
        auto itJoint = jointIndex.find( jname );
        // 场景里找不到该驱动关节：统计为缺失（DNA 与绑定不匹配时只告警，不阻断）
        if (itModel == models.end()) { ++mMissingBindings; continue; }
        if (itJoint == jointIndex.end()) continue;

        FBModel* m = itModel->second;
        InputBinding ib;
        ib.rawBase = i;
        // 输入节点：驱动关节与父级的全局 Rotation，局部在求值时反推。
        // 不用 Lcl Rotation：HIK 激活时其输出节点读到的是全局旋转
        ib.node = AnimationNodeOutCreate( 1000 + i, m, ANIMATIONNODE_TYPE_ROTATION );
        if (!ib.node) continue;   // 创建失败：跳过该驱动关节（否则求值时空指针）
        ib.parentNode = nullptr;
        if (FBModel* parent = m->Parent)
        {
            auto itParent = parentNodes.find( parent );
            if (itParent == parentNodes.end())
            {
                const int parentUserId = 2000 + static_cast<int>( parentNodes.size() );
                itParent = parentNodes.emplace(
                    parent, AnimationNodeOutCreate( parentUserId, parent, ANIMATIONNODE_TYPE_ROTATION ) ).first;
            }
            ib.parentNode = itParent->second;
        }

        auto nr = mReader->getNeutralJointRotation( itJoint->second );
        const double neutralE[3] = { nr.x, nr.y, nr.z };
        double qNeutral[4];
        EulerDegToQuat( neutralE, qNeutral );
        QuatConj( qNeutral, ib.qNeutralInv );

        mInputs.push_back( ib );
        inputJointNames.push_back( jname );
    }

    // ---- 输出：驱动集过滤（LOD），排除输入关节与根关节 ----
    auto varAttrs = mRig->getJointVariableAttributeIndices( lod );
    std::map<std::uint32_t, bool> drivenJoints;
    for (auto a : varAttrs) drivenJoints[ a / 9 ] = true;

    for (auto& kv : drivenJoints)
    {
        const std::uint32_t j = kv.first;
        auto sv = mReader->getJointName( static_cast<std::uint16_t>(j) );
        std::string jname( sv.data(), sv.size() );
        if (mReader->getJointParentIndex( static_cast<std::uint16_t>(j) ) == j) continue;
        bool isInput = false;
        for (auto& n : inputJointNames) if (n == jname) { isInput = true; break; }
        if (isInput) continue;

        auto itModel = models.find( jname );
        if (itModel == models.end()) { ++mMissingBindings; continue; }
        FBModel* m = itModel->second;

        OutputBinding ob;
        ob.jointIndex = j;
        auto nt = mReader->getNeutralJointTranslation( static_cast<std::uint16_t>(j) );
        ob.neutralT[0] = nt.x; ob.neutralT[1] = nt.y; ob.neutralT[2] = nt.z;

        // 旋转策略同 Head：Pre-Rotation≈0 且中立非零 → 中立烘在 Lcl，输出需合成
        FBVector3d pre = m->PreRotation;
        const double preE[3] = { pre[0], pre[1], pre[2] };
        auto nr = mReader->getNeutralJointRotation( static_cast<std::uint16_t>(j) );
        const double neutralE[3] = { nr.x, nr.y, nr.z };
        ob.composeRot = NeedsNeutralCompose( preE, neutralE );
        EulerDegToQuat( neutralE, ob.qNeutral );
        ob.nodeT = AnimationNodeInCreate( 3000 + j * 3 + 0, m, ANIMATIONNODE_TYPE_LOCAL_TRANSLATION );
        ob.nodeR = AnimationNodeInCreate( 3000 + j * 3 + 1, m, ANIMATIONNODE_TYPE_LOCAL_ROTATION );
        ob.nodeS = AnimationNodeInCreate( 3000 + j * 3 + 2, m, ANIMATIONNODE_TYPE_LOCAL_SCALING );
        mOutputs.push_back( ob );
        const auto bindingIndex =
            static_cast<std::uint32_t>( mOutputs.size() - 1u );
        if (ob.nodeT) mOutputRoutes.emplace(
            ob.nodeT, moburiglogic::OutputRoute {
                moburiglogic::OutputKind::JointTranslation, bindingIndex } );
        if (ob.nodeR) mOutputRoutes.emplace(
            ob.nodeR, moburiglogic::OutputRoute {
                moburiglogic::OutputKind::JointRotation, bindingIndex } );
        if (ob.nodeS) mOutputRoutes.emplace(
            ob.nodeS, moburiglogic::OutputRoute {
                moburiglogic::OutputKind::JointScaling, bindingIndex } );
    }

    mBindingsReady = !mInputs.empty() && !mOutputs.empty();
    if (mMissingBindings > 0)
        FBTrace( "[RigLogicBody] WARNING: %d DNA joints not found in scene (DNA may not match this rig)\n",
                 mMissingBindings );
    return mBindingsReady;
}

/************************************************
 *  Animation node setup
 ************************************************/
void RigLogicBodyConstraint::SetupAllAnimationNodes()
{
    if (!ReferenceGet( mGroupSkeleton, 0 )) return;
    // 播放中不重建绑定/不重读 DNA：交给 UI 空闲，停止后补做
    if (moburiglogic::IsTransportPlaying())
    {
        mRebuildPending = true;
        return;
    }
    // 路径变化或未加载时（重新）加载 DNA —— 修复换 DNA 后仍用旧数据
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

bool RigLogicBodyConstraint::RebuildBindings()
{
    FBModel* root = static_cast<FBModel*>( ReferenceGet( mGroupSkeleton, 0 ) );
    if (!root) return false;

    // 播放/求值中重建会销毁并重建动画节点，与求值线程并发时可能崩——延后到停止后由 UI 空闲补做
    if (moburiglogic::IsTransportPlaying())
    {
        mRebuildPending = true;
        return mBindingsReady;
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

void RigLogicBodyConstraint::RemoveAllAnimationNodes()
{
    mInputs.clear();
    mOutputs.clear();
    mOutputRoutes.clear();
    mLastEvalId = -1;
    mBindingsReady = false;
}

/************************************************
 *  求值引擎回调（求值图内，任意求值线程）
 ************************************************/
bool RigLogicBodyConstraint::AnimationNodeNotify( FBAnimationNode* pConnector,
                                                  FBEvaluateInfo* pEvaluateInfo,
                                                  FBConstraintInfo* pConstraintInfo )
{
    // 锁的边界（并发安全的关键）：
    //   * 绑定表（mInputs/mOutputs/mOutputRoutes）只允许在锁内访问：
    //     主线程重建会 clear() 这些容器，求值线程无锁迭代/取引用会造悬垂指针
    //   * ReadData/WriteData 一律在锁外：读可能触发上游（如 HIK）求值并回到本约束，
    //     持锁读取会自锁死
    const long evalId = pEvaluateInfo->GetEvaluationID();

    std::vector<InputBinding> inputs;
    bool needSolve;
    {
        std::lock_guard<std::mutex> lock( mSolveMutex );
        if (!mBindingsReady || !mRig || !mInst) return false;
        needSolve = ( evalId != mLastEvalId );
        if (needSolve) inputs = mInputs;   // 元素是 POD + 节点指针，拷贝开销很小
    }

    // 每个求值 ID 只跑一次完整求解；后续输出节点直接取缓存结果
    if (needSolve)
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<double> rawQuats( inputs.size() * 4u );
        for (std::size_t inputIndex = 0; inputIndex < inputs.size(); ++inputIndex)
        {
            const auto& ib = inputs[ inputIndex ];
            double globalE[3] = { 0, 0, 0 }, parentE[3] = { 0, 0, 0 };
            if (!ib.node) continue;   // 节点创建失败：跳过，不空指针崩溃
            ib.node->ReadData( globalE, pEvaluateInfo );
            if (ib.parentNode) ib.parentNode->ReadData( parentE, pEvaluateInfo );
            DriverRelativeQuat( ib.qNeutralInv, parentE, globalE, &rawQuats[ inputIndex * 4u ] );
        }

        std::lock_guard<std::mutex> lock( mSolveMutex );
        if (!mBindingsReady || !mRig || !mInst) return false;
        if (evalId != mLastEvalId)   // 并发时可能已被其他线程解完
        {
            for (std::size_t inputIndex = 0; inputIndex < inputs.size(); ++inputIndex)
            {
                const std::uint16_t rawBase = inputs[ inputIndex ].rawBase;
                for (std::uint16_t k = 0; k < 4u; ++k)
                    mInst->setRawControl( rawBase + k,
                        static_cast<float>( rawQuats[ inputIndex * 4u + k ] ) );
            }
            mRig->calculate( mInst );
            mLastSolveMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0 ).count();
            mLastEvalId = evalId;
        }
    }

    // ---- 输出：绑定信息 + 求解结果全在锁内取出，WriteData 在锁外 ----
    moburiglogic::OutputKind outKind = moburiglogic::OutputKind::BlendShape;
    FBAnimationNode* outNode = nullptr;
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
        if (route.bindingIndex >= mOutputs.size()) return false;
        const auto& output = mOutputs[ route.bindingIndex ];
        outComposeRot = output.composeRot;
        for (int k = 0; k < 3; ++k) outNeutralT[k] = output.neutralT[k];
        for (int k = 0; k < 4; ++k) outQNeutral[k] = output.qNeutral[k];
        outNode = route.kind == moburiglogic::OutputKind::JointRotation ? output.nodeR
                : route.kind == moburiglogic::OutputKind::JointScaling   ? output.nodeS
                                                                         : output.nodeT;
        const auto jointOutputs = mInst->getJointOutputs();
        const std::size_t base = static_cast<std::size_t>( output.jointIndex ) * 9u;
        if (base + 9u > jointOutputs.size()) return false;
        for (std::size_t k = 0; k < 9u; ++k) jointDelta[k] = jointOutputs[base + k];
    }

    if (!outNode) return false;   // 节点可能因重建而失效，判空兜底

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
        double value[3] = { 1.0 + jointDelta[6], 1.0 + jointDelta[7], 1.0 + jointDelta[8] };
        outNode->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::BlendShape:
        return false;
    }
    return false;
}

/************************************************
 *  FBX 存取（保存 DNA 路径与 LOD）
 ************************************************/
bool RigLogicBodyConstraint::FbxStore( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat )
{
    return true;  // FBPropertyString/Int 自动序列化
}

bool RigLogicBodyConstraint::FbxRetrieve( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat )
{
    if (pStoreWhat == kCleanup)
    {
        // 场景加载完成后重建运行时；若 SetupAllAnimationNodes 已按同一属性路径加载过则跳过，
        // 避免重复读取 DNA 并在求值可能已开始时替换运行时。
        // 属性路径打不开时会回退到 FBX 同目录的 body.dna（棚内/外拍场景）
        const char* path = DnaPath.AsString();
        const std::string want = path ? path : "";
        if ((!mRig || mLoadedDnaPropertyPath != want) && !LoadDna())
            FBTrace( "[RigLogicBody] DNA not loaded from '%s' or scene folder %s\n",
                     want.c_str(), kSceneFolderDnaName );
    }
    return true;
}
