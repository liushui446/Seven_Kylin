project(core)

file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE INC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/core/*.hpp)

# =============== 修复 jsoncpp 查找 ===============
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
add_library(sub::core ALIAS ${PROJECT_NAME})

# =============== 关键修复：jsoncpp 头文件路径 ===============
target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${JSONCPP_INCLUDE_DIRS}
    /usr/include/jsoncpp
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${JSONCPP_LIBRARIES}
    pthread
    dl
    m
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})
