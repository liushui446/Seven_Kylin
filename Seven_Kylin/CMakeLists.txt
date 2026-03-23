cmake_minimum_required(VERSION 3.10)
project(Seven CXX C)

# ======================
# 强制安装到项目内 install 目录（绝不装系统）
# ======================
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Install path" FORCE)
endif()

# 麒麟V10 系统识别
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    execute_process(COMMAND cat /etc/os-release OUTPUT_VARIABLE OS_RELEASE_INFO ERROR_QUIET)
    string(FIND "${OS_RELEASE_INFO}" "Kylin" KYLIN_FLAG)
    if(${KYLIN_FLAG} GREATER -1)
        message(STATUS "✅ 检测到银河麒麟V10桌面版")
        add_definitions(-DKYLIN_V10_OS)
    endif()

    # 架构适配
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        message(STATUS "✅ 架构：ARM64")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a -O2 -fPIC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -O2 -fPIC")
    else()
        message(STATUS "✅ 架构：x86_64")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64 -O2 -fPIC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -O2 -fPIC")
    endif()
endif()

# C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 编译选项
add_compile_options(-Wall -Wno-unused-variable -Wno-unused-function -fPIC)

# ======================
# ✅ 核心：程序从【自身目录】找库（关键！）
# ======================
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

# 全局头文件
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/module
    ${PROJECT_SOURCE_DIR}/3rdparty
)

# ================= 子模块 =================
add_subdirectory(module/core)
add_subdirectory(module/barrage)
add_subdirectory(module/deception)
add_subdirectory(module/transformation)
add_subdirectory(module/process)
add_subdirectory(app)

# 结束
install(CODE "MESSAGE(STATUS \"✅ 编译完成！安装目录：${CMAKE_INSTALL_PREFIX}\")")
