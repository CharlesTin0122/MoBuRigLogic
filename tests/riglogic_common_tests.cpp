#include "riglogic_common.h"

#include <cassert>
#include <cstdint>
#include <vector>

int main()
{
    using namespace moburiglogic;

    assert( ClampLod( -1, 8 ) == 0 );
    assert( ClampLod( 0, 8 ) == 0 );
    assert( ClampLod( 7, 8 ) == 7 );
    assert( ClampLod( 8, 8 ) == 7 );
    assert( ClampLod( 100, 8 ) == 7 );
    assert( ClampLod( 4, 0 ) == 0 );

    const std::vector<std::uint16_t> indices { 10, 11, 12 };
    const auto mappings = CollectBlendShapeMappings(
        indices,
        []( std::uint16_t index ) {
            if (index == 10) return BlendShapeMappingRef { 0, 7 };
            if (index == 11) return BlendShapeMappingRef { 1, 7 };
            return BlendShapeMappingRef { 2, 9 };
        } );
    assert( mappings.size() == 3 );
    assert( mappings[0].channelIndex == 7 );
    assert( mappings[1].channelIndex == 7 );
    assert( mappings[0].meshIndex != mappings[1].meshIndex );

    const OutputRoute route { OutputKind::BlendShape, 42 };
    assert( route.kind == OutputKind::BlendShape );
    assert( route.bindingIndex == 42 );
    return 0;
}
