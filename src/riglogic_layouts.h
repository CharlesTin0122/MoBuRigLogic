#ifndef __RIGLOGIC_LAYOUTS_H__
#define __RIGLOGIC_LAYOUTS_H__
/**
 * RigLogic 约束的自定义 Layout（属性面板 UI）
 *
 * HeadLayout:  DNA 浏览按钮 / 输入模式下拉 / 绑定状态区 / 归零+诊断按钮
 * BodyLayout:  DNA 浏览按钮 / 绑定状态区
 */

#include <fbsdk/fbsdk.h>

#include "riglogicbody_constraint.h"
#include "riglogichead_constraint.h"

//! Head 约束面板
class RigLogicHeadLayout : public FBConstraintLayout
{
    FBConstraintLayoutDeclare( RigLogicHeadLayout, FBConstraintLayout );

public:
    virtual bool FBCreate() override;
    virtual void FBDestroy() override;

private:
    void UICreate();
    void UIConfigure();
    void RefreshStatus();
    void RebuildBindings();

    void EventBrowse   ( HISender pSender, HKEvent pEvent );
    void EventModeChange( HISender pSender, HKEvent pEvent );
    void EventRebuild  ( HISender pSender, HKEvent pEvent );
    void EventZeroAll  ( HISender pSender, HKEvent pEvent );
    void EventIdle     ( HISender pSender, HKEvent pEvent );

    RigLogicHeadConstraint* mConstraint = nullptr;
    char mStatusCache[512] = {0};
    int  mIdleCounter = 0;

    FBLabel   mLabelDna;
    FBEdit    mEditDnaPath;
    FBButton  mButtonBrowse;
    FBLabel   mLabelMode;
    FBList    mListMode;
    FBLabel   mLabelStatus;
    FBButton  mButtonRebuild;
    FBButton  mButtonZero;
};

//! Body 约束面板
class RigLogicBodyLayout : public FBConstraintLayout
{
    FBConstraintLayoutDeclare( RigLogicBodyLayout, FBConstraintLayout );

public:
    virtual bool FBCreate() override;
    virtual void FBDestroy() override;

private:
    void UICreate();
    void UIConfigure();
    void RefreshStatus();

    void EventBrowse ( HISender pSender, HKEvent pEvent );
    void EventRebuild( HISender pSender, HKEvent pEvent );
    void EventIdle   ( HISender pSender, HKEvent pEvent );

    RigLogicBodyConstraint* mConstraint = nullptr;
    char mStatusCache[384] = {0};
    int  mIdleCounter = 0;

    FBLabel   mLabelDna;
    FBEdit    mEditDnaPath;
    FBButton  mButtonBrowse;
    FBLabel   mLabelStatus;
    FBButton  mButtonRebuild;
};

#endif /* __RIGLOGIC_LAYOUTS_H__ */
