#include <cstdio>
#include <locale.h>
#include "core/core.hpp"
#include "process/process.hpp"

// 麒麟V10系统宏（由CMake定义）
#ifdef KYLIN_V10_OS
#define KYLIN_OS_MSG "当前运行在银河麒麟V10系统"
#else
#define KYLIN_OS_MSG "非麒麟V10系统"
#endif

int main(int argc, char* argv[]) {
    // 适配麒麟V10中文控制台编码
    setlocale(LC_ALL, "zh_CN.UTF-8");

    // 控制台输出（示例业务逻辑）
    std::cout << "=====================================" << std::endl;
    std::cout << "        Seven 控制台程序（麒麟V10版）        " << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << KYLIN_OS_MSG << std::endl;
    std::cout << "程序参数个数：" << argc << std::endl;
    
    // 调用业务模块
    std::cout << "\n📌 调用业务模块：" << std::endl;
    seven::start_double_pipe_test();

    std::cout << "\n✅ 程序执行完成！" << std::endl;
    return 0;
}
