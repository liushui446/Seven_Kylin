#ifndef CORE_HPP
#define CORE_HPP

#include "core/CommonCore.hpp"

namespace seven {

    class SEVEN_EXPORTS Core {
    private:
        Core();  // 私有构造函数

    public:
        // 获取单例实例
        static Core* get_init();

        int timeout;
    };
}

#endif
