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

//--- OpenRigLogic
#include <dna/BinaryStreamReader.h>
#include <dna/Configuration.h>
#include <riglogic/riglogic/RigLogic.h>
#include <riglogic/riglogic/RigInstance.h>
#include <status/Status.h>
#include <trio/streams/FileStream.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <map>

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

namespace {

// XYZ 旋转序欧拉角（度）→ 四元数 (x,y,z,w)。与 Python 版 _euler_deg_to_quat 一致。
void EulerDegToQuat( const double e[3], double q[4] )
{
    const double hx = e[0] * 3.14159265358979323846 / 360.0;
    const double hy = e[1] * 3.14159265358979323846 / 360.0;
    const double hz = e[2] * 3.14159265358979323846 / 360.0;
    const double cx = std::cos(hx), sx = std::sin(hx);
    const double cy = std::cos(hy), sy = std::sin(hy);
    const double cz = std::cos(hz), sz = std::sin(hz);
    // q = qz * qy * qx
    const double qz[4] = { 0, 0, sz, cz };
    const double qy[4] = { 0, sy, 0, cy };
    const double qx[4] = { sx, 0, 0, cx };
    double t[4];
    // t = qz * qy
    t[0] = qz[3]*qy[0] + qz[0]*qy[3] + qz[1]*qy[2] - qz[2]*qy[1];
    t[1] = qz[3]*qy[1] - qz[0]*qy[2] + qz[1]*qy[3] + qz[2]*qy[0];
    t[2] = qz[3]*qy[2] + qz[0]*qy[1] - qz[1]*qy[0] + qz[2]*qy[3];
    t[3] = qz[3]*qy[3] - qz[0]*qy[0] - qz[1]*qy[1] - qz[2]*qy[2];
    // q = t * qx
    q[0] = t[3]*qx[0] + t[0]*qx[3] + t[1]*qx[2] - t[2]*qx[1];
    q[1] = t[3]*qx[1] - t[0]*qx[2] + t[1]*qx[3] + t[2]*qx[0];
    q[2] = t[3]*qx[2] + t[0]*qx[1] - t[1]*qx[0] + t[2]*qx[3];
    q[3] = t[3]*qx[3] - t[0]*qx[0] - t[1]*qx[1] - t[2]*qx[2];
}

void QuatMul( const double a[4], const double b[4], double out[4] )
{
    out[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    out[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    out[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    out[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
}

void QuatConj( const double q[4], double out[4] )
{
    out[0] = -q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = q[3];
}

// 递归收集模型树中全部骨骼（含普通 FBModel，按名字匹配 DNA）
void CollectModels( FBModel* root, std::map<std::string, FBModel*>& out )
{
    if (!root) return;
    out[ std::string( root->Name.AsString() ) ] = root;
    for (int i = 0; i < root->Children.GetCount(); ++i)
        CollectModels( root->Children[i], out );
}

// 提取模型的 namespace 前缀（"Char01:pelvis" → "Char01:"；无则空串）
std::string ExtractNamespace( FBModel* m )
{
    std::string longName( m->LongName.AsString() );
    std::string shortName( m->Name.AsString() );
    if (longName.size() > shortName.size()
        && longName.compare( longName.size() - shortName.size(),
                             shortName.size(), shortName ) == 0)
        return longName.substr( 0, longName.size() - shortName.size() );
    return std::string();
}

// namespace 感知收集：只收 LongName 以 ns 开头的模型，key=剥掉 ns 的短名
void CollectModelsNs( FBModel* root, const std::string& ns,
                      std::map<std::string, FBModel*>& out )
{
    if (!root) return;
    std::string longName( root->LongName.AsString() );
    if (ns.empty())
        out[ std::string( root->Name.AsString() ) ] = root;
    else if (longName.rfind( ns, 0 ) == 0)
        out[ longName.substr( ns.size() ) ] = root;
    for (int i = 0; i < root->Children.GetCount(); ++i)
        CollectModelsNs( root->Children[i], ns, out );
}

} // namespace

/************************************************
 *  Creation
 ************************************************/
bool RigLogicBodyConstraint::FBCreate()
{
    // 属性（FBPropertyPublish 注册到 PropertyList，UI 可见且随 FBX 序列化）
    FBPropertyPublish( this, DnaPath,     "DNA Path",      nullptr, nullptr );
    FBPropertyPublish( this, LodLevel,    "LOD Level",     nullptr, nullptr );
    FBPropertyPublish( this, LastSolveMs, "Last Solve Ms", nullptr, nullptr );
    LodLevel = 0;
    LastSolveMs = 0.0;

    // 引用组：把骨架根（root/pelvis 所在层级任意节点）拖进来
    mGroupSkeleton = ReferenceGroupAdd( "Skeleton Root", 1 );

    Deformer = false;
    HasLayout = true;
    return true;
}

void RigLogicBodyConstraint::FBDestroy()
{
    ReleaseDna();
}

/************************************************
 *  DNA / RigLogic 生命周期
 ************************************************/
bool RigLogicBodyConstraint::LoadDna()
{
    ReleaseDna();
    const char* path = DnaPath.AsString();
    if (!path || !*path) return false;

    mStream = trio::FileStream::create( path,
                                        trio::FileStream::AccessMode::Read,
                                        trio::FileStream::OpenMode::Binary );
    if (!mStream) return false;

    mReader = dna::BinaryStreamReader::create( mStream, dna::DataLayer::All );
    mReader->read();
    if (!sc::Status::isOk()) { ReleaseDna(); return false; }

    mRig = rl4::RigLogic::create( mReader );
    if (!mRig) { ReleaseDna(); return false; }
    mInst = rl4::RigInstance::create( mRig );
    if (!mInst) { ReleaseDna(); return false; }

    const auto lod = static_cast<std::uint16_t>( (int)LodLevel );
    mInst->setLOD( lod );
    mLoadedDnaPath = path;
    return true;
}

void RigLogicBodyConstraint::ReleaseDna()
{
    if (mInst)   { rl4::RigInstance::destroy( mInst );        mInst = nullptr; }
    if (mRig)    { rl4::RigLogic::destroy( mRig );            mRig = nullptr; }
    if (mReader) { dna::BinaryStreamReader::destroy( mReader ); mReader = nullptr; }
    if (mStream) { trio::FileStream::destroy( mStream );      mStream = nullptr; }
}

/************************************************
 *  绑定构建：DNA 关节名 ↔ 场景模型
 ************************************************/
bool RigLogicBodyConstraint::BuildBindings()
{
    mInputs.clear();
    mOutputs.clear();
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
    const auto lod = static_cast<std::uint16_t>( (int)LodLevel );
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
        ob.nodeT = AnimationNodeInCreate( 3000 + j * 3 + 0, m, ANIMATIONNODE_TYPE_LOCAL_TRANSLATION );
        ob.nodeR = AnimationNodeInCreate( 3000 + j * 3 + 1, m, ANIMATIONNODE_TYPE_LOCAL_ROTATION );
        ob.nodeS = AnimationNodeInCreate( 3000 + j * 3 + 2, m, ANIMATIONNODE_TYPE_LOCAL_SCALING );
        mOutputs.push_back( ob );
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
    BuildBindings();
    mLastEvalId = -1;
}

void RigLogicBodyConstraint::RemoveAllAnimationNodes()
{
    mInputs.clear();
    mOutputs.clear();
    mBindingsReady = false;
}

/************************************************
 *  求值引擎回调（求值图内，任意求值线程）
 ************************************************/
bool RigLogicBodyConstraint::AnimationNodeNotify( FBAnimationNode* pConnector,
                                                  FBEvaluateInfo* pEvaluateInfo,
                                                  FBConstraintInfo* pConstraintInfo )
{
    if (!mBindingsReady || !mRig || !mInst) return false;

    // 每个求值 ID 只跑一次完整求解；后续输出节点直接取缓存结果
    const long evalId = pEvaluateInfo->GetEvaluationID();
    if (evalId != mLastEvalId)
    {
        const auto t0 = std::chrono::steady_clock::now();

        for (const auto& ib : mInputs)
        {
            double e[3] = { 0, 0, 0 };
            ib.node->ReadData( e, pEvaluateInfo );
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
        LastSolveMs = std::chrono::duration<double, std::milli>( t1 - t0 ).count();
        mLastEvalId = evalId;
    }

    // 写当前被通知的输出节点（逐节点通知，全部走同一份求解缓存）
    auto jo = mInst->getJointOutputs();
    for (const auto& ob : mOutputs)
    {
        const std::size_t b = static_cast<std::size_t>( ob.jointIndex ) * 9;
        if (b + 9 > jo.size()) continue;

        if (pConnector == ob.nodeT)
        {
            double v[3] = { ob.neutralT[0] + jo[b+0], ob.neutralT[1] + jo[b+1], ob.neutralT[2] + jo[b+2] };
            ob.nodeT->WriteData( v, pEvaluateInfo );
            return true;
        }
        if (pConnector == ob.nodeR)
        {
            double v[3] = { jo[b+3], jo[b+4], jo[b+5] };
            ob.nodeR->WriteData( v, pEvaluateInfo );
            return true;
        }
        if (pConnector == ob.nodeS)
        {
            double v[3] = { 1.0 + jo[b+6], 1.0 + jo[b+7], 1.0 + jo[b+8] };
            ob.nodeS->WriteData( v, pEvaluateInfo );
            return true;
        }
    }
    return true;
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
        // 场景加载完成后重建运行时
        if (DnaPath.AsString() && *DnaPath.AsString())
            LoadDna();
    }
    return true;
}
