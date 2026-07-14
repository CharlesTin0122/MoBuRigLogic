#ifndef __RIGLOGIC_BODY_CONSTRAINT_H__
#define __RIGLOGIC_BODY_CONSTRAINT_H__
/**
 * RigLogicBodyConstraint —— MetaHuman 身体修形约束（MotionBuilder 2024）
 *
 * 在 MoBu 求值图内运行 Epic RigLogic（OpenRigLogic 静态库），
 * 读 44 个驱动关节的局部旋转 → RBF/SwingTwist 求解 → 写 258 个修形关节 TRS。
 *
 * 架构对标 Maya 官方 embeddedRL4 节点：
 *   - 输入/输出走 FBAnimationNode（求值图内，播放/渲染时正常工作）
 *   - 被约束属性受约束占用（不可手动改，Plot 可烘焙）
 *
 * 输入公式（与 Python 版 body_corrective.py 一致，已跨 DCC 验证）：
 *   q_rel = q(DNA中立)^-1 * q(PreRotation) * q(LclRotation)
 * 输出：
 *   平移 = 中立 + 增量；旋转 = 增量直写；缩放 = 1 + 增量
 */

//--- SDK include
#include <fbsdk/fbsdk.h>

#include "riglogic_common.h"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#define RIGLOGICBODY__CLASSNAME  RigLogicBodyConstraint
#define RIGLOGICBODY__CLASSSTR   "RigLogicBodyConstraint"

// 前向声明（避免头文件里引入 riglogic 重头文件）
namespace rl4 {
class RigLogic;
class RigInstance;
}
namespace dna {
class BinaryStreamReader;
}
namespace trio {
class FileStream;
}

//! MetaHuman 身体修形约束（RigLogic 求值图内驱动）
class RigLogicBodyConstraint : public FBConstraint
{
    FBConstraintDeclare( RigLogicBodyConstraint, FBConstraint );

public:
    virtual bool FBCreate() override;
    virtual void FBDestroy() override;

    virtual void SetupAllAnimationNodes() override;
    virtual void RemoveAllAnimationNodes() override;

    virtual bool AnimationNodeNotify( FBAnimationNode* pAnimationNode,
                                      FBEvaluateInfo* pEvaluateInfo,
                                      FBConstraintInfo* pConstraintInfo ) override;

    virtual bool FbxStore   ( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat ) override;
    virtual bool FbxRetrieve( FBFbxObject* pFbxObject, kFbxObjectStore pStoreWhat ) override;

    //--- 属性
    FBPropertyString  DnaPath;        //!< body.dna 文件路径
    FBPropertyInt     LodLevel;       //!< 驱动集 LOD（默认 0）
    FBPropertyDouble  LastSolveMs;    //!< 只读：上次求解耗时(ms)，性能观测

    //--- Layout 查询接口
    bool   BindingsReady() const { return mBindingsReady; }
    size_t InputCount()    const { return mInputs.size(); }
    size_t OutputCount()   const { return mOutputs.size(); }
    bool   DnaLoaded()     const { return mRig != nullptr; }

private:
    struct InputBinding {
        FBAnimationNode* node;        // 驱动关节 Rotation 输入（Lcl 欧拉度）
        std::uint16_t    rawBase;     // raw control 基下标（qx）
        double           qCorr[4];    // q(DNA中立)^-1 * q(Pre) 预缓存 (x,y,z,w)
    };
    struct OutputBinding {
        FBAnimationNode* nodeT;       // 修形关节 Translation 输出
        FBAnimationNode* nodeR;       // Rotation 输出
        FBAnimationNode* nodeS;       // Scaling 输出
        std::uint32_t    jointIndex;  // DNA 关节下标
        float            neutralT[3]; // DNA 中立平移
    };

    bool  LoadDna();                  // 读 DNA + 建 RigLogic/RigInstance
    void  ReleaseDna();
    std::uint16_t ResolveLod() const;
    bool  BuildBindings( std::uint16_t lod ); // 按骨架根扫描场景关节，建输入/输出映射

    // RigLogic 运行时（原生指针 + 手动 create/destroy，工厂模式）
    dna::BinaryStreamReader* mReader  = nullptr;
    trio::FileStream*        mStream  = nullptr;
    rl4::RigLogic*           mRig     = nullptr;
    rl4::RigInstance*        mInst    = nullptr;
    std::string              mLoadedDnaPath;   // 已加载路径（换 DNA 检测）

    std::vector<InputBinding>  mInputs;
    std::vector<OutputBinding> mOutputs;
    std::unordered_map<FBAnimationNode*, moburiglogic::OutputRoute> mOutputRoutes;

    int  mGroupSkeleton = -1;         // Reference Group: 骨架根（挂 pelvis/root 均可）
    long mLastEvalId    = -1;         // 每求值ID只求解一次（多输出节点共享结果）
    bool mBindingsReady = false;
};

#endif /* __RIGLOGIC_BODY_CONSTRAINT_H__ */
