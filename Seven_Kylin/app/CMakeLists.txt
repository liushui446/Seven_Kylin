file(GLOB APP_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
)

# 👇 必须加这一行！开启调试模式！断点才能生效
# set(CMAKE_BUILD_TYPE Debug)

add_executable(seven-cli ${APP_SRC})

target_link_libraries(seven-cli
    sub::core
    sub::process
    pthread
    dl
    m
    stdc++
)

install(TARGETS seven-cli
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
