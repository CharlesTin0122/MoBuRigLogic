cmake_minimum_required(VERSION 3.20)
project(moburiglogic LANGUAGES CXX)

# ------------------------------------------------------------------ MotionBuilder 2019 paths
set(MOBU_ROOT "C:/Program Files/Autodesk/MotionBuilder 2019"
    CACHE PATH "MotionBuilder 2019 installation directory")
set(ORL_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenRigLogic")
set(ORL_INCLUDE_DIR
    "${ORL_ROOT}/include")
set(ORL_LIB
    "${ORL_ROOT}/lib/win64/Release/riglogic413_2_5.lib")
set(ORL_HEADER_SENTINEL
    "${ORL_INCLUDE_DIR}/riglogic/RigLogic.h")
set(ORL_LICENSE
    "${ORL_ROOT}/LICENSE")

if(NOT EXISTS "${ORL_HEADER_SENTINEL}")
    message(FATAL_ERROR
        "Vendored OpenRigLogic header is missing: ${ORL_HEADER_SENTINEL}. "
        "Restore third_party/OpenRigLogic before configuring the project.")
endif()

if(NOT EXISTS "${ORL_LIB}")
    message(FATAL_ERROR
        "Vendored OpenRigLogic static library is missing: ${ORL_LIB}. "
        "Restore third_party/OpenRigLogic before configuring the project.")
endif()

if(NOT EXISTS "${ORL_LICENSE}")
    message(FATAL_ERROR
        "Vendored OpenRigLogic license is missing: ${ORL_LICENSE}. "
        "Restore third_party/OpenRigLogic before configuring the project.")
endif()

set(ORSDK "${MOBU_ROOT}/OpenRealitySDK")

add_library(${PROJECT_NAME} SHARED
    src/library.cxx
    src/riglogicbody_constraint.cxx
    src/riglogicbody_constraint.h
    src/riglogichead_constraint.cxx
    src/riglogichead_constraint.h
    src/riglogic_layouts.cxx
    src/riglogic_layouts.h
)

# MotionBuilder plug-in output
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    OUTPUT_NAME "moburiglogic_2019"
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "${ORSDK}/include"
    "${ORL_INCLUDE_DIR}"
)

target_link_directories(${PROJECT_NAME} PRIVATE "${ORSDK}/lib/x64")

target_link_libraries(${PROJECT_NAME} PRIVATE
    fbsdk.lib
    "${ORL_LIB}"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    KARCH_ENV_WIN=1
    KARCH_ENV_MSC=1
    PRODUCT_VERSION=2019
    _CRT_SECURE_NO_WARNINGS
    NOMINMAX
)

# MSVC: MoBu SDK 要求 MD 运行时
target_compile_options(${PROJECT_NAME} PRIVATE /MD /EHsc /bigobj /utf-8)

include(CTest)
if(BUILD_TESTING)
    add_executable(moburiglogic_unit_tests tests/riglogic_common_tests.cpp)
    target_include_directories(moburiglogic_unit_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_features(moburiglogic_unit_tests PRIVATE cxx_std_17)
    add_test(NAME moburiglogic_unit_tests COMMAND moburiglogic_unit_tests)
    add_test(
        NAME moburiglogic_source_regressions
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_source_regressions.cmake)
endif()
