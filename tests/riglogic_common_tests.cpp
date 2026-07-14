#include "riglogic_common.h"

#include <cstdint>
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

    const OutputRoute route { OutputKind::BlendShape, 42 };
    CHECK( route.kind == OutputKind::BlendShape );
    CHECK( route.bindingIndex == 42 );
    return 0;
}
