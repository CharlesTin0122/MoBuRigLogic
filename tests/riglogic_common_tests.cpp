#include "riglogic_common.h"

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
    return 0;
}
