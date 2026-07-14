cmake_minimum_required(VERSION 3.20)
project(moburiglogic LANGUAGES CXX)

# ------------------------------------------------------------------ MotionBuilder 2024 paths
set(MOBU_ROOT "C:/Program Files/Autodesk/MotionBuilder 2024"
    CACHE PATH "MotionBuilder 2024 installation directory")
set(ORL_ROOT "D:/Code/OpenRigLogic" CACHE PATH "OpenRigLogic repository")
set(ORL_LIB "${ORL_ROOT}/build/Release/riglogic413_2_5.lib"
    CACHE FILEPATH "RigLogic static library")

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
    OUTPUT_NAME "moburiglogic_2024"
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "${ORSDK}/include"
    "${ORL_ROOT}/include"
)

target_link_directories(${PROJECT_NAME} PRIVATE "${ORSDK}/lib/x64")

target_link_libraries(${PROJECT_NAME} PRIVATE
    fbsdk.lib
    "${ORL_LIB}"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    KARCH_ENV_WIN=1
    KARCH_ENV_MSC=1
    PRODUCT_VERSION=2024
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
