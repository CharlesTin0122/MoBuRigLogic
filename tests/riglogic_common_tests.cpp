#include "riglogic_common.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (false)

int main()
{
    using namespace moburiglogic;

    CHECK( ClampLod( -1, 8 ) == 0 );
    CHECK( ClampLod( 0, 8 ) == 0 );
    CHECK( ClampLod( 7, 8 ) == 7 );
    CHECK( ClampLod( 8, 8 ) == 7 );
    CHECK( ClampLod( 100, 8 ) == 7 );
    CHECK( ClampLod( 4, 0 ) == 0 );
    CHECK( ClampLod( -100, 1 ) == 0 );
    CHECK( ClampLod( 1, 1 ) == 0 );
    CHECK( ClampLod( 65535, 8 ) == 7 );

    const std::vector<std::uint16_t> indices { 10, 11, 12 };
    const auto mappings = CollectBlendShapeMappings(
        indices,
        []( std::uint16_t index ) {
            if (index == 10) return BlendShapeMappingRef { 0, 7 };
            if (index == 11) return BlendShapeMappingRef { 1, 7 };
            return BlendShapeMappingRef { 2, 9 };
        } );
    CHECK( mappings.size() == 3 );
    CHECK( mappings[0].channelIndex == 7 );
    CHECK( mappings[1].channelIndex == 7 );
    CHECK( mappings[0].meshIndex != mappings[1].meshIndex );

    const std::map<std::string, int> hostModels {
        { "SKM_H10100_FaceMesh_LOD0", 10 },
        { "SKM_H10100_FaceMesh_LOD1", 11 }
    };
    const auto fallbackOwner = FindBlendShapePropertyOwner(
        hostModels,
        std::string( "head_lod0_mesh" ),
        std::string( "head_lod0_mesh__brow_down_L" ),
        []( int host, const std::string& propertyName ) {
            return host == 10 && propertyName == "head_lod0_mesh__brow_down_L";
        } );
    CHECK( fallbackOwner == 10 );

    // ResolveBlendShapeProperty：两种 FBX 导出命名风格
    using PropSet = std::map<int, std::vector<std::string>>;
    const auto hasIn = []( const PropSet& props ) {
        return [&props]( int host, const std::string& propertyName ) {
            const auto it = props.find( host );
            if (it == props.end()) return false;
            for (const auto& p : it->second)
                if (p == propertyName) return true;
            return false;
        };
    };
    const std::map<std::string, int> meshHosts {
        { "head_lod0_mesh", 1 },
        { "teeth_lod0_mesh", 2 }
    };

    // 纯通道名（MBRig_H10100_RigLogic_001.fbx 风格）
    const PropSet plainProps { { 1, { "jaw_open" } }, { 2, { "jaw_open" } } };
    const auto plain = ResolveBlendShapeProperty(
        meshHosts, "teeth_lod0_mesh", "jaw_open", hasIn( plainProps ) );
    CHECK( plain.owner == 2 );                 // 必须落在同名网格，不能串到 head
    CHECK( plain.propertyName == "jaw_open" );
    CHECK( !plain.prefixed );

    // 纯通道名不跨网格兜底：同名网格没有该属性 → 未找到
    const PropSet otherMeshOnly { { 1, { "jaw_open" } } };
    const auto noCross = ResolveBlendShapeProperty(
        meshHosts, "teeth_lod0_mesh", "jaw_open", hasIn( otherMeshOnly ) );
    CHECK( noCross.owner == 0 );
    CHECK( noCross.propertyName.empty() );

    // mesh__channel 风格：允许宿主与网格不同名
    const PropSet prefixedProps { { 1, { "teeth_lod0_mesh__jaw_open" } } };
    const auto prefixed = ResolveBlendShapeProperty(
        meshHosts, "teeth_lod0_mesh", "jaw_open", hasIn( prefixedProps ) );
    CHECK( prefixed.owner == 1 );
    CHECK( prefixed.propertyName == "teeth_lod0_mesh__jaw_open" );
    CHECK( prefixed.prefixed );

    // 两种命名并存时优先 mesh__channel
    const PropSet bothProps { { 2, { "jaw_open", "teeth_lod0_mesh__jaw_open" } } };
    const auto both = ResolveBlendShapeProperty(
        meshHosts, "teeth_lod0_mesh", "jaw_open", hasIn( bothProps ) );
    CHECK( both.owner == 2 );
    CHECK( both.prefixed );

    // 网格不在场景中
    const auto missing = ResolveBlendShapeProperty(
        meshHosts, "eyeLeft_lod0_mesh", "jaw_open", hasIn( plainProps ) );
    CHECK( missing.owner == 0 );
    const OutputRoute translation { OutputKind::JointTranslation, 11 };
    const OutputRoute rotation { OutputKind::JointRotation, 22 };
    const OutputRoute scaling { OutputKind::JointScaling, 33 };
    const OutputRoute blendShape { OutputKind::BlendShape, 44 };
    CHECK( translation.kind == OutputKind::JointTranslation );
    CHECK( translation.bindingIndex == 11 );
    CHECK( rotation.kind == OutputKind::JointRotation );
    CHECK( rotation.bindingIndex == 22 );
    CHECK( scaling.kind == OutputKind::JointScaling );
    CHECK( scaling.bindingIndex == 33 );
    CHECK( blendShape.kind == OutputKind::BlendShape );
    CHECK( blendShape.bindingIndex == 44 );

    // ---- 四元数/欧拉（Head/Body 共用，原先两份各自实现）----
    const auto near = []( double a, double b ) { return std::fabs( a - b ) < 1e-9; };
    const auto nearDeg = []( double a, double b ) { return std::fabs( a - b ) < 1e-6; };

    const double zeroE[3] = { 0, 0, 0 };
    double qi[4];
    EulerDegToQuat( zeroE, qi );
    CHECK( near( qi[0], 0 ) && near( qi[1], 0 ) && near( qi[2], 0 ) && near( qi[3], 1 ) );

    // 单轴 90°：q = (sin45°, 0, 0, cos45°)
    const double x90[3] = { 90, 0, 0 };
    double qx90[4];
    EulerDegToQuat( x90, qx90 );
    CHECK( near( qx90[0], std::sqrt( 0.5 ) ) && near( qx90[3], std::sqrt( 0.5 ) ) );

    // 欧拉 → 四元数 → 欧拉 往返（含 FACIAL_C_Jaw 中立 [40,0,0] 与三轴混合）
    const double samples[][3] = {
        { 40, 0, 0 }, { 10, -20, 30 }, { -170, 45, 89 }, { 0.68, 164.28 - 180.0, 95.11 }
    };
    for (const auto& e : samples)
    {
        double q[4], back[3];
        EulerDegToQuat( e, q );
        QuatToEulerDeg( q, back );
        CHECK( nearDeg( back[0], e[0] ) && nearDeg( back[1], e[1] ) && nearDeg( back[2], e[2] ) );
    }

    // q∘q⁻¹ = 单位
    double qa[4], qaInv[4], qid[4];
    EulerDegToQuat( samples[1], qa );
    QuatConj( qa, qaInv );
    QuatMul( qa, qaInv, qid );
    CHECK( near( qid[0], 0 ) && near( qid[1], 0 ) && near( qid[2], 0 ) && near( qid[3], 1 ) );

    // XYZ 旋转序：q = qz∘qy∘qx（与 MoBu kFBEulerXYZ 一致）
    const double y90[3] = { 0, 90, 0 }, xy[3] = { 90, 90, 0 };
    double qy90[4], qxy[4], qcomp[4];
    EulerDegToQuat( y90, qy90 );
    EulerDegToQuat( xy, qxy );
    QuatMul( qy90, qx90, qcomp );
    CHECK( near( qxy[0], qcomp[0] ) && near( qxy[1], qcomp[1] )
        && near( qxy[2], qcomp[2] ) && near( qxy[3], qcomp[3] ) );

    // ---- 关节旋转输出策略 ----
    const double preZero[3] = { 0, 0, 0 }, preSet[3] = { 40, 0, 0 };
    const double neutralJaw[3] = { 40, 0, 0 };
    CHECK( NeedsNeutralCompose( preZero, neutralJaw ) );     // 中立烘在 Lcl → 合成
    CHECK( !NeedsNeutralCompose( preSet, neutralJaw ) );     // 中立在 Pre-Rotation → 直写
    CHECK( !NeedsNeutralCompose( preZero, zeroE ) );         // 中立为零 → 直写（合成等价）

    double qJaw[4], outE[3];
    EulerDegToQuat( neutralJaw, qJaw );
    const double delta[3] = { 22.0, -0.5, 0.7 };
    ComposeJointRotation( false, qJaw, delta, outE );
    CHECK( outE[0] == delta[0] && outE[1] == delta[1] && outE[2] == delta[2] );
    ComposeJointRotation( true, qJaw, zeroE, outE );         // 零增量 → 恰为中立
    CHECK( nearDeg( outE[0], 40 ) && nearDeg( outE[1], 0 ) && nearDeg( outE[2], 0 ) );
    const double deltaX[3] = { 22, 0, 0 };
    ComposeJointRotation( true, qJaw, deltaX, outE );        // 同轴增量 → 角度相加
    CHECK( nearDeg( outE[0], 62 ) && nearDeg( outE[1], 0 ) && nearDeg( outE[2], 0 ) );

    // ---- 驱动输入：由全局反推局部（HIK 激活时 Lcl 节点读到全局，改用此公式）----
    // 与旧公式 q中立⁻¹∘qPre∘qLcl 等价：构造 父全局∘Pre∘Lcl = 全局
    {
        const double parentG[3] = { 10, -35, 80 };
        const double pre[3] = { -4.6, 45.2, -3.3 };     // upperarm_l 的 PreRotation
        const double lcl[3] = { -0.61, -45.35, 4.75 };   // upperarm_l 的 Lcl
        const double neutral[3] = { 5, 40, -2 };
        double qP[4], qPre[4], qL[4], qPL[4], qG[4];
        EulerDegToQuat( parentG, qP ); EulerDegToQuat( pre, qPre ); EulerDegToQuat( lcl, qL );
        QuatMul( qPre, qL, qPL ); QuatMul( qP, qPL, qG );
        double globalE[3];
        QuatToEulerDeg( qG, globalE );   // 模拟 MoBu 全局 Rotation 节点给出的欧拉

        double qN[4], qNInv[4], qOld[4], qNewRel[4], tmp[4];
        EulerDegToQuat( neutral, qN ); QuatConj( qN, qNInv );
        QuatMul( qNInv, qPre, tmp ); QuatMul( tmp, qL, qOld );          // 旧公式（读 Lcl）
        DriverRelativeQuat( qNInv, parentG, globalE, qNewRel );        // 新公式（读全局）
        const double dot = qOld[0]*qNewRel[0] + qOld[1]*qNewRel[1]
                         + qOld[2]*qNewRel[2] + qOld[3]*qNewRel[3];
        CHECK( std::fabs( std::fabs( dot ) - 1.0 ) < 1e-9 );

        // 无父级（父全局=单位）：q_rel = q中立⁻¹∘q全局
        double qRoot[4], expect[4];
        DriverRelativeQuat( qNInv, zeroE, globalE, qRoot );
        QuatMul( qNInv, qG, expect );
        const double dotRoot = qRoot[0]*expect[0] + qRoot[1]*expect[1]
                             + qRoot[2]*expect[2] + qRoot[3]*expect[3];
        CHECK( std::fabs( std::fabs( dotRoot ) - 1.0 ) < 1e-9 );
    }

    // ---- namespace 前缀 ----
    CHECK( ExtractNamespacePrefix( "Char01:pelvis", "pelvis" ) == "Char01:" );
    CHECK( ExtractNamespacePrefix( "A:B:head", "head" ) == "A:B:" );
    CHECK( ExtractNamespacePrefix( "pelvis", "pelvis" ).empty() );
    CHECK( ExtractNamespacePrefix( "Char01:pelvis_x", "pelvis" ).empty() );
    return 0;
}
