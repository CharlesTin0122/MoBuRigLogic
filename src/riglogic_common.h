#ifndef MOBURIGLOGIC_RIGLOGIC_COMMON_H
#define MOBURIGLOGIC_RIGLOGIC_COMMON_H

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace moburiglogic {

constexpr double kPi = 3.14159265358979323846;

// 四元数一律 (x,y,z,w) 排布
inline void QuatMul( const double a[4], const double b[4], double out[4] )
{
    out[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    out[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    out[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    out[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
}

inline void QuatConj( const double q[4], double out[4] )
{
    out[0] = -q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = q[3];
}

// XYZ 旋转序欧拉角（度）→ 四元数：q = qz∘qy∘qx。与 Python 版 _euler_deg_to_quat 一致
inline void EulerDegToQuat( const double e[3], double q[4] )
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
inline void QuatToEulerDeg( const double q[4], double e[3] )
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    double sy = 2.0 * (w*y - z*x);
    sy = sy > 1.0 ? 1.0 : (sy < -1.0 ? -1.0 : sy);
    e[0] = std::atan2( 2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y) ) * 180.0 / kPi;
    e[1] = std::asin( sy ) * 180.0 / kPi;
    e[2] = std::atan2( 2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z) ) * 180.0 / kPi;
}

inline bool IsZeroRotation( const double e[3] )
{
    return std::fabs(e[0]) < 1e-6 && std::fabs(e[1]) < 1e-6 && std::fabs(e[2]) < 1e-6;
}

// 驱动关节输入：由全局旋转反推局部（已含 Pre-Rotation），再相对 DNA 中立：
//   q_rel = q(DNA中立)⁻¹ ∘ q(父全局)⁻¹ ∘ q(全局)
// 不直接读 Lcl Rotation：HIK 角色激活（Source=Stance/Control Rig）时，
// 约束从 Lcl Rotation 输出节点 ReadData 拿到的是全局旋转（MoBu 2019 实测）
inline void DriverRelativeQuat( const double qNeutralInv[4],
                                const double parentGlobalEuler[3],
                                const double globalEuler[3],
                                double qRel[4] )
{
    double qParent[4], qParentInv[4], qGlobal[4], qLocal[4];
    EulerDegToQuat( parentGlobalEuler, qParent );
    QuatConj( qParent, qParentInv );
    EulerDegToQuat( globalEuler, qGlobal );
    QuatMul( qParentInv, qGlobal, qLocal );
    QuatMul( qNeutralInv, qLocal, qRel );
}

// 中立是否烘在 Lcl：Pre-Rotation≈0 且 DNA 中立非零 → 输出需与中立合成。
// 中立本身为零时合成等价于直写，省掉欧拉↔四元数往返
inline bool NeedsNeutralCompose( const double preRotation[3], const double neutralRotation[3] )
{
    return IsZeroRotation( preRotation ) && !IsZeroRotation( neutralRotation );
}

// 关节旋转输出：
//   composeRot=true （Pre-Rotation≈0，中立烘在 Lcl）→ Lcl = q中立∘q增量
//   composeRot=false（中立在 Pre-Rotation 里）      → Lcl = 增量直写
inline void ComposeJointRotation( bool composeRot, const double qNeutral[4],
                                  const double deltaEuler[3], double outEuler[3] )
{
    if (!composeRot) {
        outEuler[0] = deltaEuler[0];
        outEuler[1] = deltaEuler[1];
        outEuler[2] = deltaEuler[2];
        return;
    }
    double deltaQuat[4], finalQuat[4];
    EulerDegToQuat( deltaEuler, deltaQuat );
    QuatMul( qNeutral, deltaQuat, finalQuat );
    QuatToEulerDeg( finalQuat, outEuler );
}

// "Char01:pelvis" + "pelvis" → "Char01:"；LongName 不以短名结尾或无前缀 → 空串
inline std::string ExtractNamespacePrefix( const std::string& longName,
                                           const std::string& shortName )
{
    if (longName.size() > shortName.size()
        && longName.compare( longName.size() - shortName.size(),
                             shortName.size(), shortName ) == 0)
        return longName.substr( 0, longName.size() - shortName.size() );
    return std::string();
}

// DNA GUI 控制名 "CTRL_C_jaw.ty" → 控制器模型名 "CTRL_C_jaw"（去重、保持首次出现顺序）。
// 只含 DNA 实际读取的表情控制器：面板框（CTRL_faceGUI 等）、开关类控制器天然不在其中
template <typename NameRange>
std::vector<std::string> GuiControlModelNames( const NameRange& guiControlNames )
{
    std::vector<std::string> result;
    for (const auto& guiName : guiControlNames)
    {
        const std::string name( guiName );
        const auto dot = name.rfind( '.' );
        if (dot == std::string::npos || dot == 0) continue;
        std::string model = name.substr( 0, dot );
        bool seen = false;
        for (const auto& existing : result)
            if (existing == model) { seen = true; break; }
        if (!seen) result.push_back( std::move( model ) );
    }
    return result;
}

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
