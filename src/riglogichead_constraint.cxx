/**
 * RigLogicHeadConstraint 实现。
 *
 * 求值流程（每 EvaluationID 一次）：
 *   1. 读 252 个表情属性（GetData + evalInfo，支持K帧动画）→ setRawControl
 *   2. 读 neck_01/neck_02/head Lcl Rotation → q_rel = qCorr∘q_lcl → 12 个四元数 raw
 *   3. rig->calculate → 输出节点按通知写：关节 T(中立+增量)/R(q中立∘q增量 合成)；BS ×100
 */

#include "riglogichead_constraint.h"

#include <dna/BinaryStreamReader.h>
#include <dna/Configuration.h>
#include <riglogic/riglogic/RigLogic.h>
#include <riglogic/riglogic/RigInstance.h>
#include <status/Status.h>
#include <trio/streams/FileStream.h>

#include <chrono>
#include <cmath>
#include <map>

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

namespace {

constexpr double kPi = 3.14159265358979323846;

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

// XYZ 旋转序欧拉角（度）→ 四元数 (x,y,z,w)：q = qz∘qy∘qx
void EulerDegToQuat( const double e[3], double q[4] )
{
    const double hx = e[0]*kPi/360.0, hy = e[1]*kPi/360.0, hz = e[2]*kPi/360.0;
    const double qz[4] = { 0, 0, std::sin(hz), std::cos(hz) };
    const double qy[4] = { 0, std::sin(hy), 0, std::cos(hy) };
    const double qx[4] = { std::sin(hx), 0, 0, std::cos(hx) };
    double t[4];
    QuatMul( qz, qy, t );
    QuatMul( t, qx, q );
}

// 四元数 → XYZ 旋转序欧拉角（度）
void QuatToEulerDeg( const double q[4], double e[3] )
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    double sy = 2.0 * (w*y - z*x);
    sy = sy > 1.0 ? 1.0 : (sy < -1.0 ? -1.0 : sy);
    e[0] = std::atan2( 2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y) ) * 180.0 / kPi;
    e[1] = std::asin( sy ) * 180.0 / kPi;
    e[2] = std::atan2( 2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z) ) * 180.0 / kPi;
}

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

// namespace 感知收集：只收 LongName 以 ns 开头的模型，key=剥掉 ns 的短名。
// 多角色场景下每个约束只认自己 namespace 的对象（ns 为空=收全部，向后兼容）。
void CollectModelsNs( FBModel* root, const std::string& ns,
                      std::map<std::string, FBModel*>& out )
{
    if (!root) return;
    std::string longName( root->LongName.AsString() );
    if (ns.empty())
    {
        out[ std::string( root->Name.AsString() ) ] = root;
    }
    else if (longName.rfind( ns, 0 ) == 0)
    {
        out[ longName.substr( ns.size() ) ] = root;
    }
    for (int i = 0; i < root->Children.GetCount(); ++i)
        CollectModelsNs( root->Children[i], ns, out );
}

} // namespace

bool RigLogicHeadConstraint::FBCreate()
{
    FBPropertyPublish( this, DnaPath,     "DNA Path",      nullptr, nullptr );
    FBPropertyPublish( this, LodLevel,    "LOD Level",     nullptr, nullptr );
    FBPropertyPublish( this, InputMode,   "Input Mode",    nullptr, nullptr );
    FBPropertyPublish( this, LastSolveMs, "Last Solve Ms", nullptr, nullptr );
    LodLevel = 0;
    InputMode = 0;   // 0=表情属性 1=FaceBoard面板
    LastSolveMs = 0.0;

    mGroupSkeleton = ReferenceGroupAdd( "Skeleton Root", 1 );
    Deformer = false;
    HasLayout = false;
    return true;
}

void RigLogicHeadConstraint::FBDestroy()
{
    ReleaseDna();
}

bool RigLogicHeadConstraint::LoadDna()
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
    mInst->setLOD( static_cast<std::uint16_t>( (int)LodLevel ) );
    return true;
}

void RigLogicHeadConstraint::ReleaseDna()
{
    if (mInst)   { rl4::RigInstance::destroy( mInst );          mInst = nullptr; }
    if (mRig)    { rl4::RigLogic::destroy( mRig );              mRig = nullptr; }
    if (mReader) { dna::BinaryStreamReader::destroy( mReader ); mReader = nullptr; }
    if (mStream) { trio::FileStream::destroy( mStream );        mStream = nullptr; }
}

bool RigLogicHeadConstraint::BuildBindings()
{
    mExprInputs.clear();
    mGuiInputs.clear();
    mNeckInputs.clear();
    mJointOutputs.clear();
    mBsOutputs.clear();
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
                                              ANIMATIONNODE_TYPE_LOCAL_ROTATION );
            auto nr = mReader->getNeutralJointRotation( itJoint->second );
            const double neutralE[3] = { nr.x, nr.y, nr.z };
            double qNeutral[4], qNeutralInv[4];
            EulerDegToQuat( neutralE, qNeutral );
            QuatConj( qNeutral, qNeutralInv );

            FBVector3d pre = itModel->second->PreRotation;
            const double preE[3] = { pre[0], pre[1], pre[2] };
            double qPre[4];
            EulerDegToQuat( preE, qPre );
            QuatMul( qNeutralInv, qPre, ni.qCorr );
            mNeckInputs.push_back( ni );
        }
    }

    // ---- 输出①：面部关节（驱动集过滤，跳过根与输入关节）----
    const auto lod = static_cast<std::uint16_t>( (int)LodLevel );
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
        jo.composeRot = std::fabs(pre[0]) < 1e-6 && std::fabs(pre[1]) < 1e-6
                     && std::fabs(pre[2]) < 1e-6;
        auto nr = mReader->getNeutralJointRotation( j16 );
        const double neutralE[3] = { nr.x, nr.y, nr.z };
        EulerDegToQuat( neutralE, jo.qNeutral );

        jo.nodeT = AnimationNodeInCreate( 3000 + j*3 + 0, m, ANIMATIONNODE_TYPE_LOCAL_TRANSLATION );
        jo.nodeR = AnimationNodeInCreate( 3000 + j*3 + 1, m, ANIMATIONNODE_TYPE_LOCAL_ROTATION );
        jo.nodeS = nullptr;   // head.dna 无缩放输出（实测 s:0）
        mJointOutputs.push_back( jo );
    }

    // ---- 输出②：BS 通道 → LOD0 网格 shape 属性 ----
    // 网格命名 <mesh>__<channel>，所有 LOD 共用 LOD0 前缀（实测）
    FBModel* bsMesh = nullptr;
    for (auto& kv : models)
    {
        if (kv.first.find( "LOD0" ) == std::string::npos) continue;
        FBModel* m = kv.second;
        // 找一个带 shape 属性的 LOD0 网格（用首个 BS 通道名探测）
        if (mReader->getBlendShapeChannelCount() > 0)
        {
            auto ch0 = mReader->getBlendShapeChannelName( 0 );
            std::string probe;
            for (std::uint16_t mi = 0; mi < mReader->getMeshCount(); ++mi)
            {
                auto ms = mReader->getMeshName( mi );
                probe = std::string( ms.data(), ms.size() ) + "__" + std::string( ch0.data(), ch0.size() );
                if (m->PropertyList.Find( probe.c_str() )) { bsMesh = m; break; }
            }
        }
        if (bsMesh) break;
    }
    if (bsMesh)
    {
        std::vector<std::string> prefixes;
        for (std::uint16_t mi = 0; mi < mReader->getMeshCount(); ++mi)
        {
            auto ms = mReader->getMeshName( mi );
            prefixes.push_back( std::string( ms.data(), ms.size() ) + "__" );
        }
        auto bsChannels = mRig->getBlendShapeChannelIndicesForLOD( lod );
        for (auto ci : bsChannels)
        {
            auto cs = mReader->getBlendShapeChannelName( ci );
            const std::string ch( cs.data(), cs.size() );
            for (const auto& pre : prefixes)
            {
                FBProperty* p = bsMesh->PropertyList.Find( (pre + ch).c_str() );
                if (p && p->IsAnimatable())
                {
                    FBAnimationNode* node = AnimationNodeInCreate( 6000 + ci, p );
                    if (node)
                        mBsOutputs.push_back( { node, ci } );
                    break;
                }
            }
        }
    }

    mBindingsReady = ( !mExprInputs.empty() || !mGuiInputs.empty() )
                  && !mJointOutputs.empty();
    return mBindingsReady;
}

void RigLogicHeadConstraint::SetupAllAnimationNodes()
{
    if (!ReferenceGet( mGroupSkeleton, 0 )) return;
    if (!mRig && !LoadDna()) return;
    BuildBindings();
    mLastEvalId = -1;
}

void RigLogicHeadConstraint::RemoveAllAnimationNodes()
{
    mExprInputs.clear();
    mGuiInputs.clear();
    mNeckInputs.clear();
    mJointOutputs.clear();
    mBsOutputs.clear();
    mBindingsReady = false;
}

bool RigLogicHeadConstraint::AnimationNodeNotify( FBAnimationNode* pConnector,
                                                  FBEvaluateInfo* pEvaluateInfo,
                                                  FBConstraintInfo* pConstraintInfo )
{
    if (!mBindingsReady || !mRig || !mInst) return false;

    const long evalId = pEvaluateInfo->GetEvaluationID();
    if (evalId != mLastEvalId)
    {
        const auto t0 = std::chrono::steady_clock::now();

        if (!mGuiInputs.empty())
        {
            // FaceBoard 模式：面板位移 → setGUIControl → DNA 内置翻译层
            for (const auto& gi : mGuiInputs)
            {
                double t[3] = { 0, 0, 0 };
                gi.node->ReadData( t, pEvaluateInfo );
                mInst->setGUIControl( gi.guiIndex, static_cast<float>( t[ gi.axis ] ) );
            }
            mRig->mapGUIToRawControls( mInst );   // 双向拆分/相位/量程全在 DNA 里
        }
        else
        {
            for (const auto& ei : mExprInputs)
            {
                double v = 0.0;
                ei.prop->GetData( &v, sizeof(v), pEvaluateInfo );  // 支持K帧动画取值
                mInst->setRawControl( ei.rawIndex, static_cast<float>( v ) );
            }
        }
        for (const auto& ni : mNeckInputs)
        {
            double e[3] = { 0, 0, 0 };
            ni.node->ReadData( e, pEvaluateInfo );
            double qLcl[4], qRel[4];
            EulerDegToQuat( e, qLcl );
            QuatMul( ni.qCorr, qLcl, qRel );
            mInst->setRawControl( ni.rawBase + 0, static_cast<float>( qRel[0] ) );
            mInst->setRawControl( ni.rawBase + 1, static_cast<float>( qRel[1] ) );
            mInst->setRawControl( ni.rawBase + 2, static_cast<float>( qRel[2] ) );
            mInst->setRawControl( ni.rawBase + 3, static_cast<float>( qRel[3] ) );
        }
        mRig->calculate( mInst );

        LastSolveMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0 ).count();
        mLastEvalId = evalId;
    }

    auto jo = mInst->getJointOutputs();
    for (const auto& out : mJointOutputs)
    {
        const std::size_t b = static_cast<std::size_t>( out.jointIndex ) * 9;
        if (b + 9 > jo.size()) continue;
        if (pConnector == out.nodeT)
        {
            double v[3] = { out.neutralT[0] + jo[b+0], out.neutralT[1] + jo[b+1],
                            out.neutralT[2] + jo[b+2] };
            out.nodeT->WriteData( v, pEvaluateInfo );
            return true;
        }
        if (pConnector == out.nodeR)
        {
            double v[3];
            if (out.composeRot)
            {
                const double deltaE[3] = { jo[b+3], jo[b+4], jo[b+5] };
                double qDelta[4], qFinal[4];
                EulerDegToQuat( deltaE, qDelta );
                QuatMul( out.qNeutral, qDelta, qFinal );
                QuatToEulerDeg( qFinal, v );
            }
            else
            {
                v[0] = jo[b+3]; v[1] = jo[b+4]; v[2] = jo[b+5];
            }
            out.nodeR->WriteData( v, pEvaluateInfo );
            return true;
        }
    }

    auto bs = mInst->getBlendShapeOutputs();
    for (const auto& out : mBsOutputs)
    {
        if (pConnector == out.node)
        {
            double v = out.channel < bs.size() ? bs[ out.channel ] * 100.0 : 0.0;
            out.node->WriteData( &v, pEvaluateInfo );
            return true;
        }
    }
    return true;
}

bool RigLogicHeadConstraint::FbxStore( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat )
{
    return true;
}

bool RigLogicHeadConstraint::FbxRetrieve( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat )
{
    if (pStoreWhat == kCleanup)
    {
        if (DnaPath.AsString() && *DnaPath.AsString())
            LoadDna();
    }
    return true;
}
