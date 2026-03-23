#include "core/core.hpp"

namespace seven {
    Core* m_Core;

    Core::Core()
    {

    }
    Core* Core::get_init()
    {
        if (m_Core == NULL)
        {
            m_Core = new Core();
        }
        return m_Core;
    }

}