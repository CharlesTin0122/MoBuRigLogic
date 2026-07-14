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
