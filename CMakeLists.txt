cmake_minimum_required(VERSION 3.20)
project(moburiglogic LANGUAGES CXX)

# ------------------------------------------------------------------ 路径配置
# 支持多版本: -DMOBU_VERSION=2019 / 2024（默认 2024）
set(MOBU_VERSION "2024" CACHE STRING "目标 MotionBuilder 版本 (2019/2024)")
set(MOBU_ROOT "C:/Program Files/Autodesk/MotionBuilder ${MOBU_VERSION}" CACHE PATH "MotionBuilder 安装目录")
set(ORL_ROOT "D:/Code/OpenRigLogic" CACHE PATH "OpenRigLogic 仓库")
set(ORL_LIB "${ORL_ROOT}/build/Release/riglogic413_2_5.lib" CACHE FILEPATH "RigLogic 静态库")

set(ORSDK "${MOBU_ROOT}/OpenRealitySDK")
# 2019 的 SDK 目录小写 samples、lib 结构相同; 2024 的命名空间靠 PRODUCT_VERSION 宏拼接,
# 2019 在 fbversion.h 硬编码 ORSDK2019（宏无害, 统一定义）

add_library(${PROJECT_NAME} SHARED
    src/library.cxx
    src/riglogicbody_constraint.cxx
    src/riglogicbody_constraint.h
    src/riglogichead_constraint.cxx
    src/riglogichead_constraint.h
)

# MoBu 插件后缀是 .dll，放 plugins 目录即可；产物按版本命名区分
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    OUTPUT_NAME "moburiglogic_${MOBU_VERSION}"
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
    PRODUCT_VERSION=${MOBU_VERSION}
    _CRT_SECURE_NO_WARNINGS
    NOMINMAX
)

# MSVC: MoBu SDK 要求 MD 运行时
target_compile_options(${PROJECT_NAME} PRIVATE /MD /EHsc /bigobj /utf-8)
