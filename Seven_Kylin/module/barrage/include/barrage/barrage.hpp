#ifndef BARRAGE_HPP
#define BARRAGE_HPP

#include "core/CommonCore.hpp"

namespace seven {

    // ========================= 1. 结构体封装参数 =========================
    /*enum class Jammer_Level {
        Strong = 1,
        Middle = 2,
        Weak
    };*/

    //struct InputPlatParam {
    //    UINT plat_id;       // 平台ID
    //    LLA plat_initial_pos;
    //    LLA cur_plat_pos;   // 平台经纬高(°/km)
    //    LLA cur_plat_vec;   // 平台经纬高速度(°/km)
    //};

    // 输出结果结构体
    struct BarrageTrackResult {
        double C_NJ_dB;            // 载噪比(dB-Hz)
        double J_S_dB;             // 干信比(dB)
        double sigma_jpll;         // 载波环振荡器颤动(°)
        double sigma_jdll;         // 码环跟踪误差
        bool jam_valid_flag;       // 干扰有效
        bool unlock_flag;          // 失锁标志(true=失锁)
        LLA pos_error;             // 定位误差(°/m)
        LLA target_pos;            // 目标位置
        ECEF pos_error_m;          // 定位误差(米)
        double gdop;               // 几何精度因子
    };

    // ========================= 2. 核心算法类封装 =========================
    class GNSSJammerSim {
    private:
        // WGS84椭球参数
        const double a = 6378137.0;    // 长半轴(m)
        const double f = 1.0 / 298.257223563; // 扁率
        const double e2 = 2 * f - f * f;   // 第一偏心率平方
        const double b = a * (1 - f);  // 短半轴(m)
        const double ep2 = (a * a - b * b) / (b * b); // 第二偏心率平方

    public:
        // 经纬度转ECEF
        ECEF lla_to_ecef(const LLA& lla);

        // ECEF转经纬度
        LLA ecef_to_lla(const ECEF& ecef);

    private:
        // 功率谱密度计算
        void calc_power_spectral_density(double f, const SimConfig& config, int jammer_idx, double& GJ, double& GS);

        // 数值积分（梯形法近似，替代scipy.integrate.quad）
        double integrate_quad(std::function<double(double)> func, double start, double end, int steps = 1000);

        double integrate_quad2(std::function<double(double)> func, double start, double end, int steps = 1000);

        // sinc函数实现
        double sinc(double x);

        // APM电磁传播模型计算干扰接收功率
        double calc_jam_power_apm(const JammerParam& jammer, const LLA& target_pos, const SimConfig& config);

        // 载噪比计算
        double calc_cnr(double Pj, const SimConfig& config, int jammer_idx);

        // 干信比计算
        double calc_jammer_to_signal_ratio(double Pj, const LLA& target_pos, const SimConfig& config);

        // 跟踪环误差计算
        void calc_tracking_errors(double C_NJ_dB, double J_S_dB, const SimConfig& config, int jammer_idx, double& sigma_jpll, double& sigma_jdll);

        // GDOP计算
        double calc_gdop(const LLA& target_pos, const std::vector<LLA>& satellite_pos);

        // 定位误差计算
        void calc_pos_error(bool unlock_flag, double sigma_jdll, double gdop, const SimConfig& config, LLA& pos_error, ECEF& pos_error_m);

    public:
        // 核心接口：输入航迹点和配置，输出结果
        BarrageTrackResult calc_track_result(const LLA& target_pos, const SimConfig& config);

        // 批量处理航迹数据接口
        std::vector<BarrageTrackResult> batch_calc(const std::vector<LLA>& track_points, const SimConfig& config);

        void calc_jammer_area(const SimConfig& barrage_config, vector<JammerRangeResult>& jammer_range);
    };

    int SEVEN_EXPORTS Barrage_Test(Json::Value input, Json::Value& trajectory_result);
    //int SEVEN_EXPORTS Barrage_Test_1(std::vector<InputPlatParam>& server_platform_data, Json::Value& trajectory_result);
    int SEVEN_EXPORTS Barrage_Test_1(CalcTempParam& task_param, SimConfig barrage_config);
    int SEVEN_EXPORTS Barrage_CalcjammerArea(const SimConfig& barrage_config, vector<JammerRangeResult>& jammer_range);

    //static SimConfig barrage_config;
}

#endif
