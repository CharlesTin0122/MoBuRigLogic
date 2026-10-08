#ifndef MOBURIGLOGIC_RIGLOGIC_COMMON_H
#define MOBURIGLOGIC_RIGLOGIC_COMMON_H

#include <cstdint>
#include <string>
#include <vector>

namespace moburiglogic {

inline std::uint16_t ClampLod( int requestedLod, std::uint16_t lodCount )
{
    if (lodCount == 0 || requestedLod <= 0) {
        return 0;
    }

    const auto requested = static_cast<std::uint32_t>( requestedLod );
    return static_cast<std::uint16_t>(
        requested >= lodCount ? lodCount - 1u : requested );
}

struct BlendShapeMappingRef {
    std::uint16_t meshIndex;
    std::uint16_t channelIndex;
};

template <typename IndexRange, typename Lookup>
std::vector<BlendShapeMappingRef> CollectBlendShapeMappings(
    const IndexRange& indices,
    Lookup&& lookup )
{
    std::vector<BlendShapeMappingRef> result;
    result.reserve( indices.size() );
    for (const auto index : indices) {
        result.push_back( lookup( index ) );
    }
    return result;
}
template <typename Owners, typename OwnerName, typename PropertyName,
          typename HasProperty>
typename Owners::mapped_type FindBlendShapePropertyOwner(
    const Owners& owners,
    const OwnerName& preferredOwnerName,
    const PropertyName& propertyName,
    HasProperty&& hasProperty )
{
    const auto preferred = owners.find( preferredOwnerName );
    if (preferred != owners.end() &&
        hasProperty( preferred->second, propertyName )) {
        return preferred->second;
    }

    for (const auto& owner : owners) {
        if (hasProperty( owner.second, propertyName )) {
            return owner.second;
        }
    }
    return typename Owners::mapped_type {};
}

template <typename Owner>
struct BlendShapePropertyMatch {
    Owner       owner;
    std::string propertyName;
    bool        prefixed;     // true=mesh__channel 命名，false=纯通道名
};

// BS 属性命名兼容两种 FBX 导出风格：
//   1. "mesh__channel"：任意宿主均可（优先同名网格）
//   2. "channel"：仅限同名网格 —— 纯通道名在 teeth/cartilage 等网格上可能重名
template <typename Owners, typename HasProperty>
BlendShapePropertyMatch<typename Owners::mapped_type> ResolveBlendShapeProperty(
    const Owners& owners,
    const std::string& meshName,
    const std::string& channelName,
    HasProperty&& hasProperty )
{
    const std::string prefixedName = meshName + "__" + channelName;
    const auto prefixedOwner = FindBlendShapePropertyOwner(
        owners, meshName, prefixedName, hasProperty );
    if (prefixedOwner) {
        return { prefixedOwner, prefixedName, true };
    }

    const auto mesh = owners.find( meshName );
    if (mesh != owners.end() && hasProperty( mesh->second, channelName )) {
        return { mesh->second, channelName, false };
    }
    return { typename Owners::mapped_type {}, std::string(), false };
}

enum class OutputKind : std::uint8_t {
    JointTranslation,
    JointRotation,
    JointScaling,
    BlendShape
};

struct OutputRoute {
    OutputKind kind;
    std::uint32_t bindingIndex;
};

} // namespace moburiglogic

#endif // MOBURIGLOGIC_RIGLOGIC_COMMON_H
