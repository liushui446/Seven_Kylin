#ifndef SIMMANAGER_HPP
#define SIMMANAGER_HPP

#include "core/CommonCore.hpp"
//#include "json/json.h"
//#include <windows.h>
#include "CalcThread.hpp"

namespace seven {

    // ���ķ����������
    class SimManager {
    public:

        SimManager();

        // 1. ����׼���ӿ�
        int sim_start(const Json::Value& input, Json::Value& result);

        // 2. ���濪ʼ(��ƽ̨��������ӿ�)
        int sim_calc(HANDLE hPipe, const Json::Value& input, Json::Value& result);

        // 3. ������ͣ�ӿ�
        int sim_stop(Json::Value& result);

        // 3. ��������ӿ�
        int sim_end(Json::Value& result);

    private:
        // ��ʼ����������
        void init_sim_config(const Json::Value& input, Json::Value& result);

        SimState sim_state_;          // ����״̬
        //SimConfig config_;            // ��������
        UINT sim_time_;               // ����ʱ��
        std::mutex sim_mutex_;        // �̰߳�ȫ��

        std::shared_ptr<CalcProcessThread> calc_thread_ptr;
    };

    // ȫ�ַ��������ʵ����Ҳ���Ը�����Ҫ��Ϊ�ֲ�/��Ա������
    static SimManager g_sim_manager;

}

#endif
