#ifndef __RIGLOGIC_HEAD_CONSTRAINT_H__
#define __RIGLOGIC_HEAD_CONSTRAINT_H__
/**
 * RigLogicHeadConstraint —— MetaHuman 头部表情约束（MotionBuilder 2024）
 *
 * 在求值图内运行 head.dna 的 RigLogic：
 *   输入① 252 个表情控制 = 约束自身的可K帧动态属性（jawOpen 等，0-1）
 *   输入② 12 个颈部四元数 = neck_01/neck_02/head 关节 Lcl Rotation（自动读取）
 *   输出① 840 个 FACIAL_* 关节 Lcl TRS
 *   输出② BS 通道权重（LOD0 网格 shape 属性，RigLogic 0-1 → MoBu 0-100）
 *
 * 与身体约束（riglogicbody_constraint）同架构；表情属性可直接 K 帧/接 Relation。
 */

#include <fbsdk/fbsdk.h>

#include "riglogic_common.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#define RIGLOGICHEAD__CLASSNAME  RigLogicHeadConstraint
#define RIGLOGICHEAD__CLASSSTR   "RigLogicHeadConstraint"

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

//! MetaHuman 头部表情约束（RigLogic 求值图内驱动）
class RigLogicHeadConstraint : public FBConstraint
{
    FBConstraintDeclare( RigLogicHeadConstraint, FBConstraint );

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

    FBPropertyString  DnaPath;        //!< head.dna 路径
    FBPropertyInt     LodLevel;       //!< 驱动集/BS LOD
    FBPropertyInt     InputMode;      //!< 0=表情属性(可K帧) 1=FaceBoard面板(GUI控制器)
    FBPropertyDouble  LastSolveMs;    //!< 只读：上次求解耗时

    //--- Layout 查询/操作接口
    bool   BindingsReady() const { return mBindingsReady; }
    size_t ExprCount()     const { return mExprInputs.size(); }
    size_t GuiCount()      const { return mGuiInputs.size(); }
    size_t NeckCount()     const { return mNeckInputs.size(); }
    size_t JointOutCount() const { return mJointOutputs.size(); }
    size_t BsCount()       const { return mBsOutputs.size(); }
    bool   DnaLoaded()     const { return mRig != nullptr; }
    bool   RebuildBindings();
    void   ZeroAllExpressions();   //!< 全部表情属性归零（表情属性模式用）

private:
    struct ExprInput {                // 表情控制：约束上的可K帧属性
        FBProperty*   prop;
        std::uint16_t rawIndex;
    };
    struct GuiInput {                 // FaceBoard 面板控制器：Lcl Translation 的某轴
        FBAnimationNode* node;
        std::uint16_t    guiIndex;    // DNA GUI control 下标
        int              axis;        // 0=tx 1=ty 2=tz
    };
    struct NeckInput {                // 颈部四元数：关节 Lcl Rotation
        FBAnimationNode* node;
        std::uint16_t    rawBase;
        double           qCorr[4];
    };
    struct JointOutput {
        FBAnimationNode* nodeT;
        FBAnimationNode* nodeR;
        FBAnimationNode* nodeS;       // head.dna 无缩放输出，保留槽位为空
        std::uint32_t    jointIndex;
        float            neutralT[3];
        double           qNeutral[4]; // 面部 FBX 中立烘在 Lcl —— 输出需 q中立∘q增量 合成
        bool             composeRot;
    };
    struct BsOutput {
        FBAnimationNode* node;        // 网格 shape 属性的输入节点
        std::uint16_t    channel;
    };

    bool LoadDna();
    void ReleaseDna();
    std::uint16_t ResolveLod() const;
    bool BuildBindings( std::uint16_t lod );

    dna::BinaryStreamReader* mReader = nullptr;
    trio::FileStream*        mStream = nullptr;
    rl4::RigLogic*           mRig    = nullptr;
    rl4::RigInstance*        mInst   = nullptr;
    std::string              mLoadedDnaPath;   // 已加载路径（换 DNA 检测）

    std::vector<ExprInput>   mExprInputs;
    std::vector<GuiInput>    mGuiInputs;
    std::vector<NeckInput>   mNeckInputs;
    std::vector<JointOutput> mJointOutputs;
    std::vector<BsOutput>    mBsOutputs;
    std::unordered_map<FBAnimationNode*, moburiglogic::OutputRoute> mOutputRoutes;

    int  mGroupSkeleton = -1;
    long mLastEvalId    = -1;
    bool mBindingsReady = false;
};

#endif /* __RIGLOGIC_HEAD_CONSTRAINT_H__ */
