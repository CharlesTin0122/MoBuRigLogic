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

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <map>
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

    FBPropertyString  DnaPath;        //!< 当前实际使用的 head.dna 路径（回退命中时已写回）
    FBPropertyInt     LodLevel;       //!< 驱动集/BS LOD
    FBPropertyInt     InputMode;      //!< 0=表情属性(可K帧) 1=FaceBoard面板(GUI控制器)
    FBPropertyDouble  LastSolveMs;    //!< 只读：上次求解耗时
    FBPropertyBool    LockPanelFrames;//!< 1=面板 FRM_* 不可选中（Enable Selection 关），方便选 CTRL_*
    FBPropertyBool    EyeLinkage;     //!< 1=CTRL_C_eye 位移叠加到左右眼摇杆（Maya 的双眼联动）
    FBPropertyBool    ShowFacePanel;  //!< 1=显示 GRP_faceGUI 及全部后代（FBModel::Show，不进求值图）

    //--- Layout 查询/操作接口
    bool   BindingsReady() const { return mBindingsReady; }
    size_t ExprCount()     const { return mExprInputs.size(); }
    size_t GuiCount()      const { return mGuiInputs.size(); }
    size_t NeckCount()     const { return mNeckInputs.size(); }
    size_t JointOutCount() const { return mJointOutputs.size(); }
    size_t BsCount()       const { return mBsOutputs.size(); }
    bool   DnaLoaded()     const { return mRig != nullptr; }
    double LastSolveMilliseconds() const { return mLastSolveMs.load(); }
    bool   DnaFromSceneFolder() const { return mDnaSource == moburiglogic::DnaSource::SceneFolder; }
    bool   EyeLinked()     const { return mEyeLinkNode != nullptr; }   //!< 已绑定 CTRL_C_eye 联动摇杆
    moburiglogic::DnaSource DnaSourceKind() const { return mDnaSource; }  //!< 当前 DNA 由哪一级候选命中
    int    MissingBindings()    const { return mMissingBindings; }      //!< 未在场景中找到的 DNA 关节数
    const std::string& ResolvedDnaPath() const { return mLoadedDnaPath; } //!< 实际加载的 DNA 文件
    bool   RebuildBindings();
    void   ZeroFaceControls();     //!< 面部表情回到中立：面板控制器位移 + 表情属性全部归零（可撤销）
    void   SetPanelFramesLocked( bool locked );   //!< 设置 LockPanelFrames 并立即应用到 FRM_*
    void   SetFacePanelShown( bool shown );       //!< 设置 ShowFacePanel 并立即应用到 GRP_faceGUI 及后代
    int    FacePanelState() const { return mFacePanelState; }   //!< -1=尚未应用 0=未找到 GRP_faceGUI 1=已找到
    const std::string& FacePanelFollowName() const { return mFacePanelFollowName; } //!< 跟随 head 的 Parent/Child 约束名

    //! UI 空闲回调：把求值线程记录的耗时同步到 LastSolveMs 属性（主线程写）
    void   EventUIIdle( HISender pSender, HKEvent pEvent );

private:
    struct ExprInput {                // 表情控制：约束上的可K帧属性
        FBProperty*   prop;
        std::uint16_t rawIndex;
    };
    struct GuiInput {                 // FaceBoard 面板控制器：Lcl Translation 的某轴
        FBAnimationNode* node;
        std::uint16_t    guiIndex;    // DNA GUI control 下标
        int              axis;        // 0=tx 1=ty 2=tz
        bool             eyeStick;    // 左右眼摇杆：需要叠加 CTRL_C_eye 并夹紧
    };
    struct NeckInput {                // 颈部四元数：由全局旋转反推局部（HIK 激活时 Lcl 节点读到全局）
        FBAnimationNode* node;        // 关节全局 Rotation
        FBAnimationNode* parentNode;  // 父关节全局 Rotation；无父级时为空
        std::uint16_t    rawBase;
        double           qNeutralInv[4];
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
    void InvalidateBindings();        //!< 锁内清空绑定（MoBu 销毁节点后必须立即失效，否则会解引用已释放节点）
    std::uint16_t ResolveLod() const;
    bool BuildBindings( std::uint16_t lod );
    std::map<std::string, FBModel*> CollectCharacterModels();   // 场景内本角色 namespace 的全部模型（短名→模型）
    std::string CharacterNamespace() const;                     // Skeleton Root 所属角色的 namespace（含冒号）
    std::size_t ApplyPanelFramesLock( const std::map<std::string, FBModel*>& models,
                                      bool locked );            // 返回处理的 FRM_* 数量
    std::size_t ApplyPanelVisibility( FBModel* panelRoot, bool shown ); // 返回处理的模型数量
    static FBModel* FindFacePanelRoot( const std::map<std::string, FBModel*>& models ); // GRP_faceGUI；无则空
    std::string EnsurePanelFollow( const std::map<std::string, FBModel*>& models,
                                   FBModel* panelRoot );        // 返回跟随约束名；失败为空
    void ApplyPanelState();                                     // 按属性统一应用 锁框/显隐/跟随（主线程）

    dna::BinaryStreamReader* mReader = nullptr;
    trio::FileStream*        mStream = nullptr;
    rl4::RigLogic*           mRig    = nullptr;
    rl4::RigInstance*        mInst   = nullptr;
    std::string              mLoadedDnaPath;   // 实际加载的 DNA 文件路径（可能是兜底路径）
    std::string              mLoadedDnaPropertyPath; // 加载时的 DnaPath 属性值（换 DNA 检测）
    std::string              mDnaRetryKey;     // 兜底懒重试去重键（场景路径|属性路径）

    std::vector<ExprInput>   mExprInputs;
    std::vector<GuiInput>    mGuiInputs;
    FBAnimationNode*         mEyeLinkNode = nullptr;  // CTRL_C_eye 的 Lcl Translation（双眼联动）
    std::vector<NeckInput>   mNeckInputs;
    std::vector<JointOutput> mJointOutputs;
    std::vector<BsOutput>    mBsOutputs;
    std::unordered_map<FBAnimationNode*, moburiglogic::OutputRoute> mOutputRoutes;

    // 并行求值下多个输出节点可能在不同线程同时被通知：求解+读输出须串行
    std::mutex          mSolveMutex;
    std::atomic<double> mLastSolveMs { 0.0 };  // 求值线程只写这里，UI 空闲时同步到 LastSolveMs 属性

    int  mGroupSkeleton = -1;
    long mLastEvalId    = -1;
    int  mAppliedLod    = -1;         // 当前绑定所用 LOD；UI 空闲时与 LodLevel 比对，变了即重建
    int  mStopIdleTicks = 0;          // 停播后的空闲周期计数：延迟补做重建，等在本飞行的求值排空
    int  mAppliedFrameLock = -1;      // 已应用到场景的 LockPanelFrames；-1=尚未应用
    int  mAppliedShowPanel = -1;      // 已应用到场景的 ShowFacePanel；-1=尚未应用
    bool mPanelStateDirty = false;    // 绑定刚重建（新建/打开/重建/换 LOD）：空闲时按属性重新应用面板状态
    int  mFacePanelState = -1;        // -1=尚未应用 0=未找到 GRP_faceGUI 1=已找到
    std::string mFacePanelFollowName; // 跟随 head 的 Parent/Child 约束名（状态栏显示）
    bool mBindingsReady = false;
    bool mRebuildPending = false;      // 播放中请求过重建，停止后由 UI 空闲补做
    moburiglogic::DnaSource mDnaSource = moburiglogic::DnaSource::Property; // 当前 DNA 来源级别
    int  mMissingBindings = 0;         // 上次 BuildBindings 里未找到的 DNA 关节数（只告警）
};

#endif /* __RIGLOGIC_HEAD_CONSTRAINT_H__ */
