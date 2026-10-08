#ifndef MOBURIGLOGIC_RIGLOGIC_SCENE_H
#define MOBURIGLOGIC_RIGLOGIC_SCENE_H
/**
 * Head/Body 约束共用的场景扫描工具（依赖 fbsdk；纯逻辑放 riglogic_common.h）
 */

#include <fbsdk/fbsdk.h>

#include "riglogic_common.h"

#include <dna/BinaryStreamReader.h>
#include <dna/Configuration.h>
#include <riglogic/riglogic/RigInstance.h>
#include <riglogic/riglogic/RigLogic.h>
#include <status/Status.h>
#include <trio/streams/FileStream.h>

#include <map>
#include <string>

namespace moburiglogic {

// 约束运行时只需求值数据 + Definition（关节/控制器/网格名与 BS 映射）。
// 不读 Geometry 层：head.dna 网格数据占大头，读了也只会常驻内存
// （已用 H10100 head/body.dna 验证与 DataLayer::All 输出逐值一致）
inline dna::DataLayer RuntimeDataLayers()
{
    return dna::DataLayer::RBFBehavior
         | dna::DataLayer::MachineLearnedBehavior
         | dna::DataLayer::JointBehaviorMetadata
         | dna::DataLayer::TwistSwingBehavior;
}

// 一套 RigLogic 运行时（原生指针 + 手动 create/destroy，工厂模式）
struct RigRuntime {
    trio::FileStream*        stream = nullptr;
    dna::BinaryStreamReader* reader = nullptr;
    rl4::RigLogic*           rig    = nullptr;
    rl4::RigInstance*        inst   = nullptr;
};

inline void DestroyRigRuntime( RigRuntime& rt )
{
    if (rt.inst)   { rl4::RigInstance::destroy( rt.inst );          rt.inst = nullptr; }
    if (rt.rig)    { rl4::RigLogic::destroy( rt.rig );              rt.rig = nullptr; }
    if (rt.reader) { dna::BinaryStreamReader::destroy( rt.reader ); rt.reader = nullptr; }
    if (rt.stream) { trio::FileStream::destroy( rt.stream );        rt.stream = nullptr; }
}

// 全有或全无：失败时 rt 保持为空
inline bool LoadRigRuntime( const char* path, RigRuntime& rt )
{
    rt.stream = trio::FileStream::create( path,
                                          trio::FileStream::AccessMode::Read,
                                          trio::FileStream::OpenMode::Binary );
    if (!rt.stream) return false;

    rt.reader = dna::BinaryStreamReader::create( rt.stream, RuntimeDataLayers() );
    rt.reader->read();
    if (!sc::Status::isOk() || rt.reader->getLODCount() == 0)
    {
        DestroyRigRuntime( rt );
        return false;
    }

    rt.rig = rl4::RigLogic::create( rt.reader );
    if (rt.rig) rt.inst = rl4::RigInstance::create( rt.rig );
    if (!rt.inst)
    {
        DestroyRigRuntime( rt );
        return false;
    }
    return true;
}

// 提取模型的 namespace 前缀（"Char01:pelvis" → "Char01:"；无则空串）
inline std::string ExtractNamespace( FBModel* m )
{
    return ExtractNamespacePrefix( std::string( m->LongName.AsString() ),
                                   std::string( m->Name.AsString() ) );
}

// namespace 感知收集：只收 LongName 以 ns 开头的模型，key=剥掉 ns 的短名。
// 多角色场景下每个约束只认自己 namespace 的对象（ns 为空=收全部，向后兼容）。
inline void CollectModelsNs( FBModel* root, const std::string& ns,
                             std::map<std::string, FBModel*>& out )
{
    if (!root) return;
    if (ns.empty())
    {
        out[ std::string( root->Name.AsString() ) ] = root;
    }
    else
    {
        const std::string longName( root->LongName.AsString() );
        if (longName.rfind( ns, 0 ) == 0)
            out[ longName.substr( ns.size() ) ] = root;
    }
    for (int i = 0; i < root->Children.GetCount(); ++i)
        CollectModelsNs( root->Children[i], ns, out );
}

} // namespace moburiglogic

#endif // MOBURIGLOGIC_RIGLOGIC_SCENE_H
