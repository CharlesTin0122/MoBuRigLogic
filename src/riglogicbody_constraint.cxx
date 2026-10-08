/**
 * RigLogicBodyConstraint 实现（MotionBuilder 2024 + OpenRigLogic 静态库）
 *
 * 求值流程（每个 EvaluateInfo ID 只求解一次）：
 *   1. 读全部输入节点（驱动关节 Lcl 欧拉）→ 转四元数 → q_rel = qCorr * q_lcl
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
#include <cstdio>
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
using moburiglogic::EulerDegToQuat;
using moburiglogic::ExtractNamespace;
using moburiglogic::NeedsNeutralCompose;
using moburiglogic::QuatConj;
using moburiglogic::QuatMul;

/************************************************
 *  Creation
 ************************************************/
bool RigLogicBodyConstraint::FBCreate()
{
    // 属性（FBPropertyPublish 注册到 PropertyList，UI 可见且随 FBX 序列化）
    FBPropertyPublish( this, DnaPath,     "DNA Path",      nullptr, nullptr );
    FBPropertyPublish( this, LodLevel,    "LOD Level",     nullptr, nullptr );
    FBPropertyPublish( this, LastSolveMs, "Last Solve Ms", nullptr, nullptr );
    FBPropertyPublish( this, DebugInfo,    "Debug Info",     nullptr, nullptr );
    FBPropertyPublish( this, DebugNoCache, "Debug No Cache", nullptr, nullptr );
    LodLevel = 0;
    LastSolveMs = 0.0;
    DebugNoCache = false;

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

    // [诊断·临时] 汇总上个空闲窗口的求值情况
    {
        std::string info;
        {
            std::lock_guard<std::mutex> lock( mSolveMutex );
            mDbgNoCache = static_cast<bool>( DebugNoCache );
            if (mDbgNotifies > 0)
            {
                char head[160];
                std::snprintf( head, sizeof(head), "notifies=%zu solves=%zu evalIds=%zu lastEval=%ld|",
                               mDbgNotifies, mDbgSolves, mDbgEvalIds.size(), mLastEvalId );
                info = head;
                for (std::size_t i = 0; i < mDbgReads.size() && i < mInputs.size(); ++i)
                {
                    char item[96];
                    const auto& d = mDbgReads[i];
                    std::snprintf( item, sizeof(item), "%s:%.4f,%.4f,%.4f,%d;",
                                   mInputs[i].name.c_str(), d.e[0], d.e[1], d.e[2], d.ok ? 1 : 0 );
                    info += item;
                }
                mDbgNotifies = 0;
                mDbgSolves = 0;
                mDbgEvalIds.clear();
            }
        }
        if (!info.empty()) DebugInfo = info.c_str();
    }

    // LOD 改动即时生效：驱动集/BS 映射随 LOD 变化，需重建绑定（不止 setLOD）。
    // 先记下目标 LOD 再重建，重建失败也不会每个空闲周期反复重试
    const int lod = static_cast<int>( LodLevel );
    if (mRig && mAppliedLod >= 0 && lod != mAppliedLod)
    {
        mAppliedLod = lod;
        RebuildBindings();
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
        mBindingsReady = false;   // 旧绑定的关节下标属于旧 DNA
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
    for (std::uint16_t i = 0; i < rawCount; i += 4)
    {
        auto sv = mReader->getRawControlName( i );
        std::string name( sv.data(), sv.size() );
        const auto dot = name.find('.');
        if (dot == std::string::npos) continue;
        const std::string jname = name.substr( 0, dot );

        auto itModel = models.find( jname );
        auto itJoint = jointIndex.find( jname );
        if (itModel == models.end() || itJoint == jointIndex.end()) continue;

        FBModel* m = itModel->second;
        InputBinding ib;
        ib.rawBase = i;
        ib.name = jname;
        // 输入节点：驱动关节的 Lcl Rotation（"Rotation" 是全局空间，实测坑）
        ib.node = AnimationNodeOutCreate( 1000 + i, m, ANIMATIONNODE_TYPE_LOCAL_ROTATION );

        // qCorr = q(DNA中立)^-1 * q(PreRotation)
        auto nr = mReader->getNeutralJointRotation( itJoint->second );
        const double neutralE[3] = { nr.x, nr.y, nr.z };
        double qNeutral[4], qNeutralInv[4];
        EulerDegToQuat( neutralE, qNeutral );
        QuatConj( qNeutral, qNeutralInv );

        FBVector3d pre = m->PreRotation;
        const double preE[3] = { pre[0], pre[1], pre[2] };
        double qPre[4];
        EulerDegToQuat( preE, qPre );
        QuatMul( qNeutralInv, qPre, ib.qCorr );

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
        if (itModel == models.end()) continue;
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
    return mBindingsReady;
}

/************************************************
 *  Animation node setup
 ************************************************/
void RigLogicBodyConstraint::SetupAllAnimationNodes()
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

bool RigLogicBodyConstraint::RebuildBindings()
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
    std::lock_guard<std::mutex> lock( mSolveMutex );
    if (!mBindingsReady || !mRig || !mInst) return false;

    // 每个求值 ID 只跑一次完整求解；后续输出节点直接取缓存结果
    const long evalId = pEvaluateInfo->GetEvaluationID();
    ++mDbgNotifies;                                   // [诊断]
    mDbgEvalIds.insert( evalId );                     // [诊断]
    if (evalId != mLastEvalId || mDbgNoCache)
    {
        const auto t0 = std::chrono::steady_clock::now();
        ++mDbgSolves;                                 // [诊断]
        mDbgReads.resize( mInputs.size() );           // [诊断]

        for (std::size_t inputIndex = 0; inputIndex < mInputs.size(); ++inputIndex)
        {
            const auto& ib = mInputs[ inputIndex ];
            double e[3] = { 0, 0, 0 };
            const bool readOk = ib.node->ReadData( e, pEvaluateInfo );
            mDbgReads[ inputIndex ] = { { e[0], e[1], e[2] }, readOk };   // [诊断]
            double qLcl[4], qRel[4];
            EulerDegToQuat( e, qLcl );
            QuatMul( ib.qCorr, qLcl, qRel );
            mInst->setRawControl( ib.rawBase + 0, static_cast<float>( qRel[0] ) );
            mInst->setRawControl( ib.rawBase + 1, static_cast<float>( qRel[1] ) );
            mInst->setRawControl( ib.rawBase + 2, static_cast<float>( qRel[2] ) );
            mInst->setRawControl( ib.rawBase + 3, static_cast<float>( qRel[3] ) );
        }
        mRig->calculate( mInst );

        const auto t1 = std::chrono::steady_clock::now();
        mLastSolveMs = std::chrono::duration<double, std::milli>( t1 - t0 ).count();
        mLastEvalId = evalId;
    }

    const auto routeIt = mOutputRoutes.find( pConnector );
    if (routeIt == mOutputRoutes.end()) return false;
    const auto& route = routeIt->second;
    if (route.bindingIndex >= mOutputs.size()) return false;

    const auto& output = mOutputs[ route.bindingIndex ];
    const auto jointOutputs = mInst->getJointOutputs();
    const std::size_t base = static_cast<std::size_t>( output.jointIndex ) * 9u;
    if (base + 9u > jointOutputs.size()) return false;

    switch (route.kind)
    {
    case moburiglogic::OutputKind::JointTranslation:
    {
        double value[3] = {
            output.neutralT[0] + jointOutputs[base + 0u],
            output.neutralT[1] + jointOutputs[base + 1u],
            output.neutralT[2] + jointOutputs[base + 2u]
        };
        output.nodeT->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::JointRotation:
    {
        const double deltaEuler[3] = {
            jointOutputs[base + 3u],
            jointOutputs[base + 4u],
            jointOutputs[base + 5u]
        };
        double value[3];
        ComposeJointRotation( output.composeRot, output.qNeutral, deltaEuler, value );
        output.nodeR->WriteData( value, pEvaluateInfo );
        return true;
    }
    case moburiglogic::OutputKind::JointScaling:
    {
        double value[3] = {
            1.0 + jointOutputs[base + 6u],
            1.0 + jointOutputs[base + 7u],
            1.0 + jointOutputs[base + 8u]
        };
        output.nodeS->WriteData( value, pEvaluateInfo );
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
        // 场景加载完成后重建运行时；若 SetupAllAnimationNodes 已按同一路径加载过则跳过，
        // 避免重复读取 DNA 并在求值可能已开始时替换运行时
        const char* path = DnaPath.AsString();
        if (path && *path && (!mRig || mLoadedDnaPath != path))
            LoadDna();
    }
    return true;
}
