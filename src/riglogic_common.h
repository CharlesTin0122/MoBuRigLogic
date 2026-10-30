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

// ---- DNA 路径解析 ----
// 只做字符串处理，不碰文件系统：能不能用由 LoadRigRuntime 决定。
// 这样不引入第二套路径/编码规则，中文目录等边界与现有 DnaPath 行为完全一致

// "D:/shot/hero.fbx" → "D:/shot"；无目录部分（"hero.fbx"）→ 空串
inline std::string DirectoryOf( const std::string& filePath )
{
    const auto pos = filePath.find_last_of( "/\\" );
    return pos == std::string::npos ? std::string() : filePath.substr( 0, pos );
}

// 路径最后一段："D:/a/body.dna" → "body.dna"（界面/日志用）
inline std::string FileNameOf( const std::string& filePath )
{
    const auto pos = filePath.find_last_of( "/\\" );
    return pos == std::string::npos ? filePath : filePath.substr( pos + 1 );
}

// 目录 + 文件名：沿用目录原有的分隔符风格，避免拼出 "D:/a\b.dna" 这类混合路径
inline std::string JoinPath( const std::string& dir, const std::string& fileName )
{
    if (dir.empty()) return fileName;
    const char last = dir.back();
    if (last == '/' || last == '\\') return dir + fileName;
    const char sep = dir.find( '\\' ) != std::string::npos ? '\\' : '/';
    return dir + sep + fileName;
}

// DNA 候选路径（按尝试顺序）：属性路径 → 绑定 FBX 同目录固定文件名。
//
// 回退命中后把实际路径写回 DNA Path 属性，之后这个约束就一直带着可用路径，
// merge 或打开都不再需要解析。注意：MoBu 不保留被 merge 文件的源路径，
// 所以同目录兜底只在“直接打开绑定文件”时生效（见 README）。
// 去重并忽略空串：两者指向同一文件时不重复读取
inline std::vector<std::string> DnaCandidates( const std::string& primaryPath,
                                               const std::string& sceneFilePath,
                                               const char* sceneFolderFileName )
{
    std::vector<std::string> candidates;
    const auto add = [&candidates]( const std::string& candidate ) {
        if (candidate.empty()) return;
        for (const auto& existing : candidates)
            if (existing == candidate) return;
        candidates.push_back( candidate );
    };
    add( primaryPath );
    // 场景未存盘 / 场景路径无目录部分时不产生兜底候选：
    // 拼出裸文件名 "body.dna" 会变成相对 MoBu 当前工作目录的路径，命中与否不可预测
    if (sceneFolderFileName && *sceneFolderFileName)
    {
        const std::string sceneDir = DirectoryOf( sceneFilePath );
        if (!sceneDir.empty()) add( JoinPath( sceneDir, sceneFolderFileName ) );
    }
    return candidates;
}

// 候选命中结果：哪一级生效（界面显示与日志用）
enum class DnaSource : std::uint8_t {
    Property,      // DNA Path 属性直接命中
    SceneFolder    // 命中了绑定 FBX 同目录的固定文件名（已写回属性）
};

// ---- FaceBoard 眼睛摇杆 ----
// CTRL_L_eye / CTRL_R_eye 在 DNA 的 GUI 通道里（CTRL_L_eye.tx/ty 等）；
// CTRL_C_eye 不在 DNA 里，Maya 中靠 blendWeighted 把它的驱动角度与左右眼相加，
// 等价于把它的位移直接叠到左右眼摇杆通道上。
// DNA 的 GUI 域严格是 [-1, 1]，越界会被外插成 0（实测 tx=1.05 直接归零），
// 叠加后必须夹紧，否则眼睛会瞬间回中。
inline double ClampEyeStick( double value )
{
    return value > 1.0 ? 1.0 : (value < -1.0 ? -1.0 : value);
}

// FaceBoard 面板的框/背景模型（FRM_*）：锁定后不可在视图中选中，方便选 CTRL_* 控制器
inline bool IsPanelFrameName( const std::string& name )
{
    return name.size() > 4 && name.compare( 0, 4, "FRM_" ) == 0;
}

// FaceBoard 面板根节点（剥掉 namespace 的短名）：显隐与跟随 head 都作用在它上面
inline bool IsFacePanelRootName( const std::string& name )
{
    return name == "GRP_faceGUI";
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
