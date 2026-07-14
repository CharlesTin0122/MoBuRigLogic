/**
 * RigLogic 约束 Layout 实现。
 *
 * AddRegion 参数序：第一组=X(Left/Right)，第二组=Y(Top/Bottom)——与 SDK 样例一致。
 * 状态区经 OnUIIdle 节流刷新（约每秒），标准约束操作（直接勾 Active 等）也能反映。
 */

#include "riglogic_layouts.h"

#include <cstdio>
#include <cstring>

namespace {
// 状态文本变化时才写 Caption（避免每秒重绘闪烁）
void SetIfChanged( FBLabel& label, const char* text, char* cache, size_t cacheSize )
{
    if (std::strncmp( cache, text, cacheSize ) != 0)
    {
        std::snprintf( cache, cacheSize, "%s", text );
        label.Caption = text;
    }
}
} // namespace

// ================================================================= Head
FBConstraintLayoutImplementation( RigLogicHeadLayout );
FBRegisterConstraintLayout( RigLogicHeadLayout,
                            RIGLOGICHEAD__CLASSSTR,
                            FB_DEFAULT_SDK_ICON );

bool RigLogicHeadLayout::FBCreate()
{
    mConstraint = (RigLogicHeadConstraint*)(FBConstraint*)Constraint;
    UICreate();
    UIConfigure();
    RefreshStatus();
    FBSystem().OnUIIdle.Add( this, (FBCallback)&RigLogicHeadLayout::EventIdle );
    return true;
}

void RigLogicHeadLayout::FBDestroy()
{
    FBSystem().OnUIIdle.Remove( this, (FBCallback)&RigLogicHeadLayout::EventIdle );
}

void RigLogicHeadLayout::UICreate()
{
    const int lS = 6, lH = 22;

    // 参数序: X(Left/Right) 在前, Y(Top/Bottom) 在后（SDK 约定）
    AddRegion( "LabelDna", "LabelDna",
               lS,   kFBAttachLeft,   "",            1.0,
               lS,   kFBAttachTop,    "",            1.0,
               70,   kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );
    AddRegion( "EditDnaPath", "EditDnaPath",
               lS,   kFBAttachRight,  "LabelDna",    1.0,
               0,    kFBAttachTop,    "LabelDna",    1.0,
               260,  kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );
    AddRegion( "ButtonBrowse", "ButtonBrowse",
               lS,   kFBAttachRight,  "EditDnaPath", 1.0,
               0,    kFBAttachTop,    "EditDnaPath", 1.0,
               80,   kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );

    AddRegion( "LabelMode", "LabelMode",
               lS,   kFBAttachLeft,   "",            1.0,
               lS,   kFBAttachBottom, "LabelDna",    1.0,
               70,   kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );
    AddRegion( "ListMode", "ListMode",
               lS,   kFBAttachRight,  "LabelMode",   1.0,
               0,    kFBAttachTop,    "LabelMode",   1.0,
               280,  kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );

    AddRegion( "ButtonRebuild", "ButtonRebuild",
               lS,   kFBAttachLeft,   "",              1.0,
               lS,   kFBAttachBottom, "LabelMode",     1.0,
               110,  kFBAttachNone,   nullptr,         1.0,
               lH,   kFBAttachNone,   nullptr,         1.0 );
    AddRegion( "ButtonZero", "ButtonZero",
               lS,   kFBAttachRight,  "ButtonRebuild", 1.0,
               0,    kFBAttachTop,    "ButtonRebuild", 1.0,
               110,  kFBAttachNone,   nullptr,         1.0,
               lH,   kFBAttachNone,   nullptr,         1.0 );

    AddRegion( "LabelStatus", "LabelStatus",
               lS,   kFBAttachLeft,   "",              1.0,
               lS,   kFBAttachBottom, "ButtonRebuild", 1.0,
               -lS,  kFBAttachRight,  "",              1.0,
               90,   kFBAttachNone,   nullptr,         1.0 );

    SetControl( "LabelDna",      mLabelDna );
    SetControl( "EditDnaPath",   mEditDnaPath );
    SetControl( "ButtonBrowse",  mButtonBrowse );
    SetControl( "LabelMode",     mLabelMode );
    SetControl( "ListMode",      mListMode );
    SetControl( "ButtonRebuild", mButtonRebuild );
    SetControl( "ButtonZero",    mButtonZero );
    SetControl( "LabelStatus",   mLabelStatus );
}

void RigLogicHeadLayout::UIConfigure()
{
    mLabelDna.Caption = "DNA File";
    mEditDnaPath.Text = mConstraint->DnaPath.AsString() ? mConstraint->DnaPath.AsString() : "";
    mButtonBrowse.Caption = "Browse...";
    mButtonBrowse.OnClick.Add( this, (FBCallback)&RigLogicHeadLayout::EventBrowse );

    mLabelMode.Caption = "Input Mode";
    mListMode.Style = kFBDropDownList;
    mListMode.Items.Add( "Expression Properties (K-frame on constraint)" );
    mListMode.Items.Add( "FaceBoard Panel (drive by CTRL_* controls)" );
    mListMode.ItemIndex = ( (int)mConstraint->InputMode == 1 ) ? 1 : 0;
    mListMode.OnChange.Add( this, (FBCallback)&RigLogicHeadLayout::EventModeChange );

    mButtonRebuild.Caption = "Rebuild Bindings";
    mButtonRebuild.OnClick.Add( this, (FBCallback)&RigLogicHeadLayout::EventRebuild );
    mButtonZero.Caption = "Zero Expressions";
    mButtonZero.OnClick.Add( this, (FBCallback)&RigLogicHeadLayout::EventZeroAll );

    mLabelStatus.WordWrap = true;
}

void RigLogicHeadLayout::RefreshStatus()
{
    char buf[512];
    if (!mConstraint->DnaLoaded())
    {
        std::snprintf( buf, sizeof(buf),
            "DNA not loaded.\n"
            "1) Set DNA file (head.dna)  2) Drag 'head' joint into\n"
            "Skeleton Root reference  3) Check Active." );
    }
    else if (!mConstraint->BindingsReady())
    {
        std::snprintf( buf, sizeof(buf),
            "DNA loaded, bindings NOT ready.\n"
            "Drag the character's 'head' joint into Skeleton Root\n"
            "reference group, then Active on. In multi-character\n"
            "scenes check the namespace of the dragged joint." );
    }
    else
    {
        const bool board = mConstraint->GuiCount() > 0;
        std::snprintf( buf, sizeof(buf),
            "Bindings OK (%s mode)\n"
            "Inputs:  %zu expressions, %zu panel controls, %zu neck joints\n"
            "Outputs: %zu facial joints, %zu blendshape channels\n"
            "Last solve: %.3f ms",
            board ? "FaceBoard" : "Expression",
            mConstraint->ExprCount(), mConstraint->GuiCount(), mConstraint->NeckCount(),
            mConstraint->JointOutCount(), mConstraint->BsCount(),
            (double)mConstraint->LastSolveMs );
    }
    SetIfChanged( mLabelStatus, buf, mStatusCache, sizeof(mStatusCache) );
}

void RigLogicHeadLayout::RebuildBindings()
{
    // Active 关→开 触发 RemoveAllAnimationNodes/SetupAllAnimationNodes
    mConstraint->Active = false;
    mConstraint->Active = true;
    FBSystem().Scene->Evaluate();
    RefreshStatus();
}

void RigLogicHeadLayout::EventIdle( HISender, HKEvent )
{
    // 节流：约每 60 个空闲周期刷一次（状态文本无变化时 SetIfChanged 零开销）
    if (++mIdleCounter < 60) return;
    mIdleCounter = 0;
    RefreshStatus();
}

void RigLogicHeadLayout::EventBrowse( HISender, HKEvent )
{
    FBFilePopup popup;
    popup.Caption = "Select MetaHuman head.dna";
    popup.Style = kFBFilePopupOpen;
    popup.Filter = "*.dna";
    if (popup.Execute())
    {
        mConstraint->DnaPath = popup.FullFilename.AsString();
        mEditDnaPath.Text = popup.FullFilename.AsString();
        RebuildBindings();
    }
}

void RigLogicHeadLayout::EventModeChange( HISender, HKEvent )
{
    mConstraint->InputMode = ( mListMode.ItemIndex == 1 ) ? 1 : 0;
    RebuildBindings();
}

void RigLogicHeadLayout::EventRebuild( HISender, HKEvent )
{
    // 允许用户在编辑框里手改路径后点重建
    mConstraint->DnaPath = mEditDnaPath.Text.AsString();
    RebuildBindings();
}

void RigLogicHeadLayout::EventZeroAll( HISender, HKEvent )
{
    mConstraint->ZeroAllExpressions();
    FBSystem().Scene->Evaluate();
    RefreshStatus();
}

// ================================================================= Body
FBConstraintLayoutImplementation( RigLogicBodyLayout );
FBRegisterConstraintLayout( RigLogicBodyLayout,
                            RIGLOGICBODY__CLASSSTR,
                            FB_DEFAULT_SDK_ICON );

bool RigLogicBodyLayout::FBCreate()
{
    mConstraint = (RigLogicBodyConstraint*)(FBConstraint*)Constraint;
    UICreate();
    UIConfigure();
    RefreshStatus();
    FBSystem().OnUIIdle.Add( this, (FBCallback)&RigLogicBodyLayout::EventIdle );
    return true;
}

void RigLogicBodyLayout::FBDestroy()
{
    FBSystem().OnUIIdle.Remove( this, (FBCallback)&RigLogicBodyLayout::EventIdle );
}

void RigLogicBodyLayout::UICreate()
{
    const int lS = 6, lH = 22;

    AddRegion( "LabelDna", "LabelDna",
               lS,   kFBAttachLeft,   "",            1.0,
               lS,   kFBAttachTop,    "",            1.0,
               70,   kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );
    AddRegion( "EditDnaPath", "EditDnaPath",
               lS,   kFBAttachRight,  "LabelDna",    1.0,
               0,    kFBAttachTop,    "LabelDna",    1.0,
               260,  kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );
    AddRegion( "ButtonBrowse", "ButtonBrowse",
               lS,   kFBAttachRight,  "EditDnaPath", 1.0,
               0,    kFBAttachTop,    "EditDnaPath", 1.0,
               80,   kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );
    AddRegion( "ButtonRebuild", "ButtonRebuild",
               lS,   kFBAttachLeft,   "",            1.0,
               lS,   kFBAttachBottom, "LabelDna",    1.0,
               110,  kFBAttachNone,   nullptr,       1.0,
               lH,   kFBAttachNone,   nullptr,       1.0 );
    AddRegion( "LabelStatus", "LabelStatus",
               lS,   kFBAttachLeft,   "",              1.0,
               lS,   kFBAttachBottom, "ButtonRebuild", 1.0,
               -lS,  kFBAttachRight,  "",              1.0,
               72,   kFBAttachNone,   nullptr,         1.0 );

    SetControl( "LabelDna",      mLabelDna );
    SetControl( "EditDnaPath",   mEditDnaPath );
    SetControl( "ButtonBrowse",  mButtonBrowse );
    SetControl( "ButtonRebuild", mButtonRebuild );
    SetControl( "LabelStatus",   mLabelStatus );
}

void RigLogicBodyLayout::UIConfigure()
{
    mLabelDna.Caption = "DNA File";
    mEditDnaPath.Text = mConstraint->DnaPath.AsString() ? mConstraint->DnaPath.AsString() : "";
    mButtonBrowse.Caption = "Browse...";
    mButtonBrowse.OnClick.Add( this, (FBCallback)&RigLogicBodyLayout::EventBrowse );
    mButtonRebuild.Caption = "Rebuild Bindings";
    mButtonRebuild.OnClick.Add( this, (FBCallback)&RigLogicBodyLayout::EventRebuild );
    mLabelStatus.WordWrap = true;
}

void RigLogicBodyLayout::RefreshStatus()
{
    char buf[384];
    if (!mConstraint->DnaLoaded())
    {
        std::snprintf( buf, sizeof(buf),
            "DNA not loaded.\n"
            "1) Set DNA file (body.dna)  2) Drag 'pelvis' joint into\n"
            "Skeleton Root reference  3) Check Active." );
    }
    else if (!mConstraint->BindingsReady())
    {
        std::snprintf( buf, sizeof(buf),
            "DNA loaded, bindings NOT ready.\n"
            "Drag the character's 'pelvis' into Skeleton Root and\n"
            "activate. Check namespace in multi-character scenes." );
    }
    else
    {
        std::snprintf( buf, sizeof(buf),
            "Bindings OK\n"
            "Inputs: %zu driver joints   Outputs: %zu corrective joints\n"
            "Last solve: %.3f ms",
            mConstraint->InputCount(), mConstraint->OutputCount(),
            (double)mConstraint->LastSolveMs );
    }
    SetIfChanged( mLabelStatus, buf, mStatusCache, sizeof(mStatusCache) );
}

void RigLogicBodyLayout::EventIdle( HISender, HKEvent )
{
    if (++mIdleCounter < 60) return;
    mIdleCounter = 0;
    RefreshStatus();
}

void RigLogicBodyLayout::EventBrowse( HISender, HKEvent )
{
    FBFilePopup popup;
    popup.Caption = "Select MetaHuman body.dna";
    popup.Style = kFBFilePopupOpen;
    popup.Filter = "*.dna";
    if (popup.Execute())
    {
        mConstraint->DnaPath = popup.FullFilename.AsString();
        mEditDnaPath.Text = popup.FullFilename.AsString();
        EventRebuild( nullptr, nullptr );
    }
}

void RigLogicBodyLayout::EventRebuild( HISender, HKEvent )
{
    mConstraint->DnaPath = mEditDnaPath.Text.AsString();
    mConstraint->Active = false;
    mConstraint->Active = true;
    FBSystem().Scene->Evaluate();
    RefreshStatus();
}
