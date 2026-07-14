if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

file(READ "${SOURCE_DIR}/CMakeLists.txt" cmake_content)
file(READ "${SOURCE_DIR}/src/riglogic_layouts.cxx" layout_content)
file(READ "${SOURCE_DIR}/src/riglogichead_constraint.cxx" head_content)
file(READ "${SOURCE_DIR}/src/riglogicbody_constraint.cxx" body_content)

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
assert_not_contains("${head_content}" "getJointVariableAttributeIndices( static_cast" "unsafe Head LOD query")
assert_not_contains("${body_content}" "getJointVariableAttributeIndices( static_cast" "unsafe Body LOD query")
assert_not_contains("${layout_content}" "Active = false" "Layout Active-off rebuild")
assert_not_contains("${layout_content}" "Active = true" "Layout Active-on rebuild")
