if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

file(READ "${SOURCE_DIR}/CMakeLists.txt" cmake_content)
file(READ "${SOURCE_DIR}/src/riglogic_layouts.cxx" layout_content)
file(READ "${SOURCE_DIR}/src/riglogichead_constraint.cxx" head_content)
file(READ "${SOURCE_DIR}/src/riglogicbody_constraint.cxx" body_content)
file(READ "${SOURCE_DIR}/src/riglogic_scene.h" scene_content)
file(READ "${SOURCE_DIR}/src/riglogic_notice.h" notice_content)

function(assert_contains content needle label)
    string(FIND "${content}" "${needle}" position)
    if(position EQUAL -1)
        message(FATAL_ERROR "Missing ${label}: ${needle}")
    endif()
endfunction()

function(assert_not_contains content needle label)
    string(FIND "${content}" "${needle}" position)
    if(NOT position EQUAL -1)
        message(FATAL_ERROR "Forbidden ${label}: ${needle}")
    endif()
endfunction()

assert_contains("${cmake_content}" [=[OUTPUT_NAME "moburiglogic_2019"]=] "fixed plugin output name")
assert_contains("${cmake_content}" "PRODUCT_VERSION=2019" "MotionBuilder 2019 product definition")
assert_not_contains("${cmake_content}" "set(MOBU_VERSION" "multi-version CMake option")
assert_not_contains("${cmake_content}" "MotionBuilder 2024" "MotionBuilder 2024 support")

assert_contains("${cmake_content}" [=[${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenRigLogic]=]
                "vendored OpenRigLogic root")
assert_contains("${cmake_content}" [=[${ORL_ROOT}/include]=]
                "vendored OpenRigLogic include path")
assert_not_contains("${cmake_content}" [=[    "${ORL_INCLUDE_DIR}")]=]
                    "self-referential OpenRigLogic include path")
assert_contains("${cmake_content}" [=[${ORL_INCLUDE_DIR}/riglogic/RigLogic.h]=]
                "OpenRigLogic header sentinel check")
assert_contains("${cmake_content}" [=[lib/win64/Release/riglogic413_2_5.lib]=]
                "vendored OpenRigLogic static library")
assert_contains("${cmake_content}" [=[message(FATAL_ERROR]=]
                "configuration-time dependency failure")
assert_not_contains("${cmake_content}" "D:/Code/OpenRigLogic"
                    "external OpenRigLogic repository")
assert_not_contains("${cmake_content}" [=[CACHE PATH "OpenRigLogic repository"]=]
                    "OpenRigLogic root cache override")
assert_not_contains("${cmake_content}" [=[CACHE FILEPATH "RigLogic static library"]=]
                    "OpenRigLogic library cache override")

assert_contains("${layout_content}" [=[kFBAttachBottom, "LabelDna"]=] "DNA layout anchor")
assert_contains("${layout_content}" [=[kFBAttachBottom, "LabelMode"]=] "mode layout anchor")
assert_contains("${head_content}" "RigLogicHeadConstraint::RebuildBindings()" "Head safe rebuild")
assert_contains("${body_content}" "RigLogicBodyConstraint::RebuildBindings()" "Body safe rebuild")
assert_contains("${head_content}" "getMeshBlendShapeChannelMappingIndicesForLOD( lod )" "Head mapping-based BlendShape binding")
assert_contains("${head_content}" "ResolveBlendShapeProperty(" "Head plain/prefixed BlendShape naming fallback")
assert_contains("${head_content}" "std::lock_guard<std::mutex> lock( mSolveMutex )" "Head solve serialization")
assert_contains("${body_content}" "std::lock_guard<std::mutex> lock( mSolveMutex )" "Body solve serialization")
assert_not_contains("${head_content}" "        LastSolveMs =" "Head property write on evaluation thread")
assert_not_contains("${body_content}" "        LastSolveMs =" "Body property write on evaluation thread")
assert_contains("${head_content}" "RigLogicHeadConstraint::EventUIIdle" "Head solve-time sync on main thread")
assert_contains("${body_content}" "RigLogicBodyConstraint::EventUIIdle" "Body solve-time sync on main thread")

# 共用工具只保留一份实现
foreach(content_var head_content body_content)
    assert_not_contains("${${content_var}}" "void QuatMul(" "duplicated quaternion helper in ${content_var}")
    assert_not_contains("${${content_var}}" "void CollectModels(" "unused CollectModels in ${content_var}")
    assert_not_contains("${${content_var}}" "DataLayer::All" "Geometry layer loaded at runtime in ${content_var}")
    assert_contains("${${content_var}}" "mLoadedDnaPropertyPath != want" "idempotent FbxRetrieve DNA load in ${content_var}")
    assert_contains("${${content_var}}" "lod != mAppliedLod" "LOD change rebuild in ${content_var}")
    assert_contains("${${content_var}}" "DnaCandidates(" "scene-folder DNA fallback in ${content_var}")
    assert_contains("${${content_var}}" "mDnaSource = source" "DNA source tracking in ${content_var}")
    assert_contains("${${content_var}}" "if (key != mDnaRetryKey)" "scene-folder DNA lazy retry in ${content_var}")
    # 回退命中时属性必须写回实际路径（不再有 Source DNA Path）
    assert_contains("${${content_var}}" "DnaPath = resolvedPath.c_str()" "DNA Path reflects resolved DNA in ${content_var}")
    assert_not_contains("${${content_var}}" "SourceDnaPath" "removed Source DNA Path member in ${content_var}")
    assert_contains("${${content_var}}" "PropertyRemove( legacy )" "legacy Source property removal in ${content_var}")
    # 通知不能在解析回调里直接弹，必须先登记到空闲队列
    assert_contains("${${content_var}}" "DnaNoticeQueue::Push( notice )" "DNA notice queued in ${content_var}")
    assert_contains("${${content_var}}" "DnaNoticeQueue::OnSceneLoaded()" "DNA notice reset on load in ${content_var}")
    assert_not_contains("${${content_var}}" "mDnaFromSceneFolder" "stale fallback flag in ${content_var}")
endforeach()
assert_contains("${head_content}" "kSceneFolderDnaName = \"head.dna\"" "Head scene-folder DNA name")
assert_contains("${body_content}" "kSceneFolderDnaName = \"body.dna\"" "Body scene-folder DNA name")
assert_contains("${body_content}" [=[PropertyList.Find( "Source DNA Path" )]=] "Body legacy Source property lookup")
# Head 还要清掉旧面板分支留下的 Follow Head / Panel Offset
assert_contains("${head_content}" [=[kLegacyPropertyNames[] = { "Source DNA Path", "Follow Head", "Panel Offset" }]=] "Head legacy property names")
assert_contains("${head_content}" "PropertyList.Find( legacyName )" "Head legacy property lookup")

# 眼睛联动：CTRL_C_eye 不在 DNA GUI 通道里，插件叠到左右眼摇杆上并夹紧
assert_contains("${head_content}" "kCentreEyeStick = \"CTRL_C_eye\"" "Head eye linkage centre stick")
assert_contains("${head_content}" "gi.eyeStick = ( ctrlName == kLeftEyeStick || ctrlName == kRightEyeStick )" "Head eye stick tagging")
assert_contains("${head_content}" "eyeLinkNode->ReadData( eyeLink, pEvaluateInfo )" "Head eye linkage read")
assert_contains("${head_content}" "moburiglogic::ClampEyeStick( v )" "Head eye stick clamp")
assert_contains("${head_content}" "if (eyeLinkOn) v += eyeLink[ guiInputs[i].axis ]" "Head eye linkage sum")
assert_contains("${head_content}" [=["Eye Linkage"]=] "Eye Linkage property")
assert_contains("${layout_content}" "DNA Path updated" "DNA source line reflects relocation")
assert_contains("${layout_content}" "EyeLinkLine(" "Eye linkage status line")
assert_not_contains("${body_content}" "EyeLinkage" "body constraint has no eye linkage")

# 并发安全：绑定表只在锁内访问 + 播放中禁止重建（避免节点重建与求值线程并发）
foreach(content_var head_content body_content)
    assert_contains("${${content_var}}" "if (moburiglogic::IsTransportPlaying())" "rebuild guard while playing in ${content_var}")
    assert_contains("${${content_var}}" "mRebuildPending" "pending rebuild after stop in ${content_var}")
    assert_contains("${${content_var}}" "InvalidateBindings()" "binding invalidation on deferred rebuild in ${content_var}")
    assert_contains("${${content_var}}" "mStopIdleTicks >= 10" "delayed pending rebuild in ${content_var}")
    assert_contains("${${content_var}}" "InvalidateBindings()\n{\n    std::lock_guard<std::mutex> lock( mSolveMutex );" "locked teardown in ${content_var}")
    assert_not_contains("${${content_var}}" "mOutputRoutes.clear();\n    mEyeLinkNode = nullptr;\n    mLastEvalId = -1;\n    mBindingsReady = false;" "unlocked teardown in ${content_var}")
    assert_contains("${${content_var}}" "if (!mBindingsReady || !mInst) return false;" "locked binding access in ${content_var}")
    assert_not_contains("${${content_var}}" "const auto& out = mBsOutputs[ route.bindingIndex ];\n        double value" "unlocked BS binding access in ${content_var}")
endforeach()
assert_contains("${head_content}" "std::vector<GuiInput>  guiInputs;" "Head input snapshot")
assert_contains("${body_content}" "std::vector<InputBinding> inputs;" "Body input snapshot")
assert_not_contains("${head_content}" "mGuiInputs[i].node->ReadData" "Head unguarded GUI node read")
assert_not_contains("${body_content}" "mInputs[ inputIndex ].node->ReadData" "Body unguarded input node read")
assert_contains("${notice_content}" "FBMessageBox(" "DNA notice message box")
assert_contains("${notice_content}" "不再提示本次" "DNA notice mute button")
assert_not_contains("${notice_content}" "FBMessageBoxWithCheck(" "unreliable message box variant")
assert_contains("${body_content}" "ComposeJointRotation(" "Body neutral-compose rotation output")
assert_contains("${head_content}" "ComposeJointRotation(" "Head neutral-compose rotation output")

# Zero Expressions：按 DNA GUI 控制器归零面板，并可撤销
assert_contains("${layout_content}" "ZeroFaceControls()" "Zero Expressions button resets face controls")
assert_contains("${head_content}" "GuiControlModelNames(" "Zero Expressions uses DNA GUI controls")
assert_contains("${head_content}" "TransactionAddModelTRS(" "Zero Expressions is undoable")
assert_not_contains("${head_content}" "ZeroAllExpressions" "legacy expression-only zero")

# Lock Panel Frames：FRM_* 不可选中，属性可保存、可脚本设置
assert_contains("${head_content}" [=["Lock Panel Frames"]=] "Lock Panel Frames property")
assert_contains("${head_content}" "Pickable = !locked" "FRM_* selection toggle")
assert_contains("${layout_content}" "SetPanelFramesLocked(" "Lock Panel Frames checkbox")

# 新建约束默认：FaceBoard 模式、锁面板框、隐藏面板
assert_contains("${head_content}" "InputMode = 1;" "Head default FaceBoard input mode")
assert_contains("${head_content}" "LockPanelFrames = true;" "Head default locked panel frames")
assert_contains("${head_content}" "ShowFacePanel = false;" "Head default hidden face panel")
assert_contains("${head_content}" [=["Show Face Panel"]=] "Show Face Panel property")
assert_contains("${layout_content}" "SetFacePanelShown(" "Show Face Panel checkbox")
assert_contains("${layout_content}" "FacePanelLine(" "Face panel status line")
# 面板状态在绑定重建后由主线程空闲应用（新建约束时默认值立即生效），不再“首帧只记录”
assert_contains("${head_content}" "if (BuildBindings( validLod )) mPanelStateDirty = true;" "panel state reapplied after rebuild")
assert_contains("${head_content}" "ApplyPanelState();" "panel state applied on idle")
assert_not_contains("${head_content}" "if (mAppliedFrameLock < 0)" "first-idle record-only frame lock")
assert_contains("${head_content}" "m->Show = shown" "face panel visibility via Show")
assert_not_contains("${head_content}" "->Visibility" "face panel visibility via animatable Visibility")
# 跟随 head 用原生 Parent/Child 约束，不在 RigLogic 求值里写面板变换
assert_contains("${head_content}" [=[TypeCreateConstraint( "Parent/Child" )]=] "native Parent/Child panel follow")
assert_contains("${head_content}" [=["Parent-Child"]=] "reuse existing panel follow constraint")
assert_contains("${head_content}" "Snap();" "panel follow keeps offset")
string(FIND "${head_content}" "bool RigLogicHeadConstraint::AnimationNodeNotify" notify_begin)
string(FIND "${head_content}" "bool RigLogicHeadConstraint::FbxStore" notify_end)
if(notify_begin EQUAL -1 OR notify_end LESS_EQUAL notify_begin)
    message(FATAL_ERROR "Cannot locate Head AnimationNodeNotify body")
endif()
math(EXPR notify_length "${notify_end} - ${notify_begin}")
string(SUBSTRING "${head_content}" ${notify_begin} ${notify_length} notify_content)
foreach(needle "GRP_faceGUI" "FacePanel" "PanelFollow" "Panel Offset" "Follow Head")
    assert_not_contains("${notify_content}" "${needle}" "face panel logic in Head AnimationNodeNotify")
endforeach()

# HIK 激活时 Lcl Rotation 输出节点读到全局旋转：驱动输入必须由全局反推局部
foreach(content_var head_content body_content)
    assert_contains("${${content_var}}" "DriverRelativeQuat(" "global-to-local driver input in ${content_var}")
    assert_not_contains("${${content_var}}" "ANIMATIONNODE_TYPE_LOCAL_ROTATION );\n\n        // qCorr" "Lcl Rotation driver input in ${content_var}")
    assert_not_contains("${${content_var}}" "qCorr" "Lcl-based driver correction in ${content_var}")
endforeach()
assert_not_contains("${head_content}" "getJointVariableAttributeIndices( static_cast" "unsafe Head LOD query")
assert_not_contains("${body_content}" "getJointVariableAttributeIndices( static_cast" "unsafe Body LOD query")
assert_not_contains("${layout_content}" "Active = false" "Layout Active-off rebuild")
assert_not_contains("${layout_content}" "Active = true" "Layout Active-on rebuild")
