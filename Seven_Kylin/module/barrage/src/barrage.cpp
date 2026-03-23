#include "barrage/barrage.hpp"

#include <cmath>
#include <Eigen/Dense>  // 需Eigen库处理矩阵运算（GDOP计算）
#include <numeric>      // 数值计算


namespace seven {

    // ========================= 2. 核心算法类封装 =========================
    // 经纬度转ECEF
    ECEF GNSSJammerSim::lla_to_ecef(const LLA& lla) {
        ECEF ecef;
        double lat_deg_rad = lla.lat_deg * M_PI / 180.0;
        double lon_deg_rad = lla.lon_deg * M_PI / 180.0;
        double h_m = lla.h_m * 1000.0; // km -> m

        double N = a / sqrt(1 - e2 * pow(sin(lat_deg_rad), 2));
        ecef.X = (N + h_m) * cos(lat_deg_rad) * cos(lon_deg_rad);
        ecef.Y = (N + h_m) * cos(lat_deg_rad) * sin(lon_deg_rad);
        ecef.Z = ((1 - e2) * N + h_m) * sin(lat_deg_rad);
        return ecef;
    }

    // ECEF转经纬度
    LLA GNSSJammerSim::ecef_to_lla(const ECEF& ecef) {
        LLA lla;
        double p = sqrt(ecef.X * ecef.X + ecef.Y * ecef.Y);

        // 处理极区情况
        if (p < 1e-12) {
            lla.lon_deg = 0.0;
            lla.lat_deg = (ecef.Z >= 0) ? 90.0 : -90.0;
            lla.h_m = (fabs(ecef.Z) - b) / 1000.0; // m -> km
            return lla;
        }

        double theta = atan2(ecef.Z * a, p * b);
        double lon_deg_rad = atan2(ecef.Y, ecef.X);
        double lat_deg_rad = atan2(
            ecef.Z + ep2 * b * pow(sin(theta), 3),
            p - e2 * a * pow(cos(theta), 3)
        );
        double N = a / sqrt(1 - e2 * pow(sin(lat_deg_rad), 2));
        lla.lon_deg = lon_deg_rad * 180.0 / M_PI;
        lla.lat_deg = lat_deg_rad * 180.0 / M_PI;
        lla.h_m = (p / cos(lat_deg_rad) - N) / 1000.0; // m -> km
        return lla;
    }

    // 功率谱密度计算
    void GNSSJammerSim::calc_power_spectral_density(double f, const SimConfig& barrage_config, int jammer_idx, double& GJ, double& GS) {
        // 卫星信号功率谱密度GS(f)
        if (barrage_config.pseudocode == "C/A" || barrage_config.pseudocode == "P(Y)") {
            GS = barrage_config.Tc * pow(sinc(M_PI * f * barrage_config.Tc), 2);
        }
        else if (barrage_config.pseudocode == "M") {
            GS = barrage_config.Tc * pow(sinc(M_PI * f * barrage_config.Tc / 2), 2) * pow(tan(M_PI * f / 2 * barrage_config.fs), 2);
        }
        else {
            GS = 0.0;
        }

        // 干扰信号功率谱密度GJ(f)
        const JammerParam& jammer = barrage_config.jammers[jammer_idx];
        double jam_freq = jammer.freq;
        GJ = 0.0;

        if (jammer.type == "continuous_wave") {
            GJ = 1.0;
        }
        else if (jammer.type == "multi-tone") {
            // 多音干扰简化处理：匹配中心频率±1e3Hz
            if (fabs(f - jam_freq) < 1e3) {
                GJ = 1.0;
            }
        }
        else if (jammer.type == "bandlimited_gaussian") {
            if (fabs(f - jam_freq) < jammer.bandwidth / 2) {
                GJ = 1.0 / jammer.bandwidth;
            }
        }
        else if (jammer.type == "pseudocode") {
            if (fabs(f - jam_freq) < jammer.bandwidth / 2) {
                GJ = 1.0;
            }
        }
        else if (jammer.type == "pulse") {
            double tau = jammer.pulse_width;
            double T = jammer.pulse_period;
            double sa_term = sinc((f - jam_freq) * tau);
            if (fabs(f - jam_freq) < jammer.bandwidth / 2) {
                GJ = (tau / T) * pow(sa_term, 2);
            }
        }
    }

    // 数值积分（梯形法近似，替代scipy.integrate.quad）
    double GNSSJammerSim::integrate_quad2(std::function<double(double)> func, double start, double end, int steps) {
        double h_km = (end - start) / steps;
        double sum = 0.5 * (func(start) + func(end));
        for (int i = 1; i < steps; ++i) {
            sum += func(start + i * h_km);
        }
        return sum * h_km;
    }

    double GNSSJammerSim::integrate_quad(std::function<double(double)> func, double start, double end, int steps) {
        // 1. 合法性校验：steps必须是正偶数（辛普森1/3法要求，自动修正或抛异常二选一）
        if (steps <= 0) {
            throw std::invalid_argument("steps must be a positive integer!");
        }
        // 可选：自动将奇数steps修正为最近的偶数（避免调用方传参错误，推荐加）
        if (steps % 2 != 0) {
            steps = std::round(steps / 2.0) * 2;
            // 也可以抛异常：throw std::invalid_argument("steps must be an even positive integer for Simpson's 1/3 rule!");
        }

        double h_km = (end - start) / steps;
        double sum = func(start) + func(end); // 首尾项直接求和（无0.5系数，和梯形法的第一个区别）

        // 2. 循环区分奇数/偶数步：奇数乘4，偶数乘2（和梯形法的第二个区别）
        for (int i = 1; i < steps; ++i) {
            double x = start + i * h_km;
            if (i % 2 == 1) { // 奇数步（1,3,5...）
                sum += 4 * func(x);
            }
            else { // 偶数步（2,4,6...）
                sum += 2 * func(x);
            }
        }

        // 3. 最终结果乘 h/3（梯形法是乘h，和梯形法的第三个区别）
        return sum * h_km / 3.0;
    }

    // sinc函数实现
    double GNSSJammerSim::sinc(double x) {
        if (fabs(x) < 1e-12) 
            return 1.0;
        return sin(x) / x;
    }

    // APM电磁传播模型计算干扰接收功率
    double GNSSJammerSim::calc_jam_power_apm(const JammerParam& jammer, const LLA& target_pos, const SimConfig& barrage_config) {
        // 转换为ECEF计算距离
        ECEF jam_ecef = lla_to_ecef(jammer.pos);
        ECEF target_ecef = lla_to_ecef(target_pos);
        double dist = sqrt(
            pow(jam_ecef.X - target_ecef.X, 2) +
            pow(jam_ecef.Y - target_ecef.Y, 2) +
            pow(jam_ecef.Z - target_ecef.Z, 2)
        ); // 总距离(米)

        // 垂直/水平距离
        double dist_vertical = fabs((jammer.pos.h_m - target_pos.h_m) * 1000);
        double dist_horizontal = sqrt(pow(dist, 2) - pow(dist_vertical, 2));

        // 天线仰角(°)
        double antenna_elevation = atan2(dist_vertical, dist_horizontal) * 180.0 / M_PI;

        // 路径损耗计算
        double loss = 0.0;
        if (antenna_elevation > 5 || dist < 5000) {
            // FE模型（自由空间损耗）
            loss = pow(4 * M_PI * dist * barrage_config.fc / barrage_config.c, 2);
        }
        else if (dist < 20000) {
            // RO模型（射线光学）
            double refraction_factor = 1.0003;
            loss = pow(4 * M_PI * dist * barrage_config.fc * refraction_factor / barrage_config.c, 2);
        }
        else if (target_pos.h_m * 1000 < 10000) {
            // PE模型（抛物方程）
            loss = pow(4 * M_PI * dist * barrage_config.fc / barrage_config.c, 2) * 1.2;
        }
        else {
            // XO模型（扩展光学）
            loss = pow(4 * M_PI * dist * barrage_config.fc / barrage_config.c, 2) * 0.8;
        }

        // 干扰接收功率(W)
        double jam_power_w = jammer.power;
        // 若输入为dBm，转换为W（可选逻辑）
        if (jam_power_w < 100) {
            jam_power_w = pow(10, jam_power_w / 10);
        }
        double Pj = jam_power_w * 1.0 / loss; // 天线增益简化为1

        return Pj;
    }

    // 载噪比计算
    double GNSSJammerSim::calc_cnr(double Pj, const SimConfig& barrage_config, int jammer_idx) {
        // 积分计算∫GJ(f)GS(f)df（积分范围：fc - beta/2 到 fc + beta/2）
        auto integrand = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, barrage_config, jammer_idx, GJ, GS);
            return GJ * GS;
            };

        double integral_result = integrate_quad(
            integrand,
            barrage_config.fc - barrage_config.beta / 2,
            barrage_config.fc + barrage_config.beta / 2,
            1000
        );
        double integral_total = Pj * integral_result;

        // 载噪比计算（线性值转dB-Hz）
        double C_NJ_linear = barrage_config.satellite.carrier_power / integral_total;
        double C_NJ_dB = (C_NJ_linear > 0) ? 10 * log10(C_NJ_linear) : -100;

        return C_NJ_dB;
    }

    // 干信比计算
    double GNSSJammerSim::calc_jammer_to_signal_ratio(double Pj, const LLA& target_pos, const SimConfig& barrage_config) {
        // 计算卫星信号总接收功率Ps(W)
        double Ps_total = 0.0;
        for (const auto& sat_pos : barrage_config.satellite.sat_pos) {
            // 星地距离计算
            ECEF sat_ecef = lla_to_ecef(sat_pos);
            ECEF target_ecef = lla_to_ecef(target_pos);
            double dist_sat_target = sqrt(
                pow(sat_ecef.X - target_ecef.X, 2) +
                pow(sat_ecef.Y - target_ecef.Y, 2) +
                pow(sat_ecef.Z - target_ecef.Z, 2)
            );

            // 自由空间损耗
            double loss_sat = pow(4 * M_PI * dist_sat_target * barrage_config.fc / barrage_config.c, 2);
            // GPS卫星典型EIRP：48.5 dBW = 70794.58 W
            double sat_eirp = 70794.58;
            double Ps_sat = sat_eirp * 100 / loss_sat;
            Ps_total += Ps_sat;
        }

        // 干信比(dB)
        if (Ps_total < 1e-20) {
            return 200.0; // 避免除零，设为极大值
        }
        double J_S = Pj / Ps_total;
        return 10 * log10(J_S);
    }

    // 跟踪环误差计算
    void GNSSJammerSim::calc_tracking_errors(double C_NJ_dB, double J_S_dB, const SimConfig& barrage_config, int jammer_idx, double& sigma_jpll, double& sigma_jdll) {
        double J_S = pow(10, J_S_dB / 10); // 转线性值
        double C_NJ_linear = pow(10, C_NJ_dB / 10);

        // 1. 载波环振荡器颤动σ_JPLL(°)
        double term = (barrage_config.Bp / C_NJ_linear) * (1 + 1 / (2 * barrage_config.Td * C_NJ_linear));
        sigma_jpll = (360.0 / (2 * M_PI)) * sqrt(term);
        sigma_jpll = fmod(sigma_jpll, 360.0); // 限制在0-360°

        // 2. 码环跟踪误差σ_JDLL
        // 积分项1：∫GJ(f)GS(f)·sin?(πf d Tc) df
        auto integrand_sin2 = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, barrage_config, jammer_idx, GJ, GS);
            double sin_term = sin(M_PI * f * barrage_config.d * barrage_config.Tc);
            return GJ * GS * pow(sin_term, 2);
            };
        double integral_sin2 = integrate_quad(
            integrand_sin2,
            barrage_config.fc - barrage_config.beta / 2,
            barrage_config.fc + barrage_config.beta / 2
        );

        // 积分项2：∫f·GJ(f)GS(f)·sin(πf d Tc) df
        auto integrand_fsin = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, barrage_config, jammer_idx, GJ, GS);
            return f * GJ * GS * sin(M_PI * f * barrage_config.d * barrage_config.Tc);
            };
        double integral_fsin = integrate_quad(
            integrand_fsin,
            barrage_config.fc - barrage_config.beta / 2,
            barrage_config.fc + barrage_config.beta / 2
        );

        // 积分项3：∫GJ(f)GS(f)·cos?(πf d Tc) df
        auto integrand_cos2 = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, barrage_config, jammer_idx, GJ, GS);
            double cos_term = cos(M_PI * f * barrage_config.d * barrage_config.Tc);
            return GJ * GS * pow(cos_term, 2);
            };
        double integral_cos2 = integrate_quad(
            integrand_cos2,
            barrage_config.fc - barrage_config.beta / 2,
            barrage_config.fc + barrage_config.beta / 2
        );

        // 积分项4：∫GJ(f)GS(f)·cos(πf d Tc) df
        auto integrand_cos = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, barrage_config, jammer_idx, GJ, GS);
            return GJ * GS * cos(M_PI * f * barrage_config.d * barrage_config.Tc);
            };
        double integral_cos = integrate_quad(
            integrand_cos,
            barrage_config.fc - barrage_config.beta / 2,
            barrage_config.fc + barrage_config.beta / 2
        );

        // 码环误差计算
        sigma_jdll = 0.0;
        if (integral_fsin != 0) {
            double numerator = sqrt(barrage_config.Bd * J_S * integral_sin2);
            double denominator = 2 * M_PI * integral_fsin;
            double term2 = (J_S * integral_cos2) / (barrage_config.Td * pow(integral_cos, 2));
            sigma_jdll = (numerator / denominator) * sqrt(1 + term2);
        }
    }

    // GDOP计算
    double GNSSJammerSim::calc_gdop(const LLA& target_pos, const std::vector<LLA>& satellite_pos) {
        // 构建几何矩阵G
        Eigen::MatrixXd G(satellite_pos.size(), 4);
        for (int i = 0; i < satellite_pos.size(); ++i) {
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);
            ECEF target_ecef = lla_to_ecef(target_pos);
            Eigen::Vector3d delta(
                sat_ecef.X - target_ecef.X,
                sat_ecef.Y - target_ecef.Y,
                sat_ecef.Z - target_ecef.Z
            );
            double dist = delta.norm();
            if (dist < 1e-6) dist = 1e-6;
            Eigen::Vector3d unit_vec = delta / dist;
            G(i, 0) = unit_vec.x();
            G(i, 1) = unit_vec.y();
            G(i, 2) = unit_vec.z();
            G(i, 3) = 1.0;
        }

        // 计算GDOP = sqrt(trace((G^T G)^-1))
        double gdop = 10.0; // 默认最差值
        try {
            Eigen::MatrixXd GtG = G.transpose() * G;
            Eigen::MatrixXd GtG_inv = GtG.inverse();
            gdop = sqrt(GtG_inv.trace());
        }
        catch (...) {
            // 矩阵不可逆时返回默认值
        }
        return gdop;
    }

    // 定位误差计算
    void GNSSJammerSim::calc_pos_error(bool unlock_flag, double sigma_jdll, double gdop, const SimConfig& barrage_config, LLA& pos_error, ECEF& pos_error_m) {
        if (unlock_flag) {
            // 失锁：惯导漂移误差
            pos_error.lon_deg = barrage_config.ins_drift / 111;  // 1°≈111km，转换为度
            pos_error.lat_deg = barrage_config.ins_drift / 111;
            pos_error.h_m = 0.1;                       // 高度误差(km)

            pos_error_m.X = barrage_config.ins_drift * 1000; // 米
            pos_error_m.Y = barrage_config.ins_drift * 1000;
            pos_error_m.Z = barrage_config.ins_drift * 1000;
        }
        else {
            // 未失锁：伪距误差×GDOP
            double pseudo_range_error = sigma_jdll * barrage_config.Tc * barrage_config.c; // 伪距误差(米)
            double error_m = pseudo_range_error * gdop;

            // 转换为经纬高误差（度/km）
            pos_error.lon_deg = error_m / (1000 * 111); // 米→度
            pos_error.lat_deg = error_m / (1000 * 111);
            pos_error.h_m = error_m / 1000;           // 米→km

            pos_error_m.X = error_m;
            pos_error_m.Y = error_m;
            pos_error_m.Z = error_m;
        }
    }

    BarrageTrackResult GNSSJammerSim::calc_track_result(const LLA& target_pos, const SimConfig& barrage_config) {
        BarrageTrackResult result;
        result.C_NJ_dB = 0.0;
        result.J_S_dB = 0.0;
        result.sigma_jpll = 0.0;
        result.sigma_jdll = 0.0;
        result.jam_valid_flag = false;
        result.unlock_flag = false;
        result.pos_error = { 0.0, 0.0, 0.0 };
        result.pos_error_m = { 0.0, 0.0, 0.0 };
        result.gdop = calc_gdop(target_pos, barrage_config.satellite.sat_pos);

        int jammer_count = barrage_config.jammers.size();
        if (jammer_count == 0) {
            return result; // 无干扰源，返回默认值
        }

        // 累加所有干扰源的影响
        double sum_C_NJ = 0.0;
        double sum_J_S = 0.0;
        double sum_sigma_jpll = 0.0;
        double sum_sigma_jdll = 0.0;
        int unlock_num = 0;

        for (int i = 0; i < jammer_count; i++) {
            // 1. 计算干扰接收功率
            double Pj = calc_jam_power_apm(barrage_config.jammers[i], target_pos, barrage_config);

            // 2. 计算载噪比
            double C_NJ = calc_cnr(Pj, barrage_config, i);
            sum_C_NJ += C_NJ;

            // 3. 计算干信比
            double J_S = calc_jammer_to_signal_ratio(Pj, target_pos, barrage_config);
            sum_J_S += J_S;

            if (C_NJ < 70)// 干扰有效（载噪比低于正常阈值） 原来是30
            {
                result.jam_valid_flag = true;
                // 4. 计算跟踪环误差
                double sigma_jpll, sigma_jdll;
                calc_tracking_errors(C_NJ, J_S, barrage_config, i, sigma_jpll, sigma_jdll);
                sum_sigma_jpll += sigma_jpll;
                sum_sigma_jdll += sigma_jdll;

                // 5. 判定失锁
                bool pll_unlock = (sigma_jpll > barrage_config.pll_unlock_thresh);
                bool dll_unlock = (sigma_jdll > barrage_config.dll_unlock_thresh);
                if (pll_unlock || dll_unlock) {
                    unlock_num++;
                }
            }
        }

        // 平均所有干扰源的结果
        result.C_NJ_dB = sum_C_NJ / jammer_count;
        result.J_S_dB = sum_J_S / jammer_count;
        result.sigma_jpll = sum_sigma_jpll / jammer_count;
        result.sigma_jdll = sum_sigma_jdll / jammer_count;
        result.unlock_flag = (unlock_num > 0); // 任意干扰源导致失锁则整体失锁

        // 计算定位误差
        calc_pos_error(
            result.unlock_flag,
            result.sigma_jdll,
            result.gdop,
            barrage_config,
            result.pos_error,
            result.pos_error_m
        );

        return result;
    }

    // 批量处理航迹数据接口
    std::vector<BarrageTrackResult> GNSSJammerSim::batch_calc(const std::vector<LLA>& track_points, const SimConfig& barrage_config) {
        std::vector<BarrageTrackResult> results;
        for (const auto& point : track_points) {
            results.push_back(calc_track_result(point, barrage_config));
        }
        return results;
    }

    // 辅助函数：生成穿过指定干扰源的航迹（接近-穿过-远离）
    static std::vector<LLA> generate_track_through_jammer(const LLA& jammer_pos, int sim_time = 400) {
        std::vector<LLA> track_points;

        // 初始点：干扰源西北方向0.5度处
        LLA start_pos = {
            jammer_pos.lon_deg - 0.5,
            jammer_pos.lat_deg + 0.5,
            jammer_pos.h_m  //千米
        };
        track_points.push_back(start_pos);

        // 速度：向干扰源移动，最终穿过干扰源
        LLA velocity = {
            (jammer_pos.lon_deg - start_pos.lon_deg) / (sim_time / 2),  // 经度速度
            (jammer_pos.lat_deg - start_pos.lat_deg) / (sim_time / 2),  // 纬度速度
            0.0                                                        // 高度不变
        };

        // 生成航迹点
        for (int step = 1; step <= sim_time; step++) {
            LLA current_pos = start_pos + velocity * step;
            track_points.push_back(current_pos);
        }

        return track_points;
    }

    // 仅计算失锁起止点到干扰源的平均距离作为干扰半径
    void process_platform_tracks(
        const Json::Value& platform_tracks,
        const SimConfig& barrage_config,
        GNSSJammerSim& sim,
        std::vector<JammerRangeResult>& jammer_range) {

        // 遍历每个平台
        for (int plat_idx = 0; plat_idx < platform_tracks.size(); plat_idx++) {
            UINT platform_id = platform_tracks[plat_idx]["platform_id"].asUInt();
            const Json::Value& track_points_json = platform_tracks[plat_idx]["track_points"];

            // 1. 转换JSON航迹点到内部格式
            std::vector<LLA> track_points;
            for (int j = 0; j < track_points_json.size(); j++) {
                LLA target_pos;
                target_pos.lon_deg = track_points_json[j]["lon_deg"].asDouble();
                target_pos.lat_deg = track_points_json[j]["lat_deg"].asDouble();
                target_pos.h_m = track_points_json[j]["h_m"].asDouble() / 1000.0; // 米转千米
                track_points.push_back(target_pos);
            }

            // 2. 仿真计算（获取各航迹点的失锁状态）
            std::vector<BarrageTrackResult> results = sim.batch_calc(track_points, barrage_config);

            // 3. 构造该平台的基础结果
            Json::Value platform_result;
            platform_result["platform_id"] = platform_id;
            Json::Value& track_results = platform_result["track_results"];

            // ========== 核心：计算每个干扰源的失锁范围半径 ==========
            JammerRangeResult range_result;
            range_result.jammer_id = plat_idx + 1;          // 干扰源ID（从1开始）
            range_result.platform_id = platform_id;           // 对应平台ID
            range_result.jammer_centre = sim.lla_to_ecef(barrage_config.jammers[plat_idx].pos); // 干扰源ECEF坐标
            range_result.jammer_centre_lla = barrage_config.jammers[plat_idx].pos;             // 干扰源LLA坐标

            // 关键变量：失锁开始/结束位置、距离
            ECEF unlock_start_pos, unlock_end_pos;
            double dist_start = 0.0, dist_end = 0.0, avg_dist = 0.0;
            bool unlock_start_found = false, unlock_end_found = false;

            // 遍历航迹点，定位失锁的起始和结束位置
            bool last_unlock = false;
            for (int i = 0; i < results.size(); ++i) {
                bool cur_unlock = results[i].unlock_flag;

                // 1. 检测失锁开始：从"未失锁"变为"失锁"
                if (!last_unlock && cur_unlock && !unlock_start_found) {
                    unlock_start_pos = sim.lla_to_ecef(track_points[i]); // 初始失锁位置
                    unlock_start_found = true;
                    // 计算初始失锁位置到干扰源的距离
                    dist_start = sqrt(
                        pow(range_result.jammer_centre.X - unlock_start_pos.X, 2) +
                        pow(range_result.jammer_centre.Y - unlock_start_pos.Y, 2) +
                        pow(range_result.jammer_centre.Z - unlock_start_pos.Z, 2)
                    );
                }

                // 2. 检测失锁结束：从"失锁"变为"未失锁"（且已找到开始位置）
                if (last_unlock && !cur_unlock && unlock_start_found && !unlock_end_found) {
                    unlock_end_pos = sim.lla_to_ecef(track_points[i - 1]); // 最终失锁位置（取前一个点）
                    unlock_end_found = true;
                    // 计算最终失锁位置到干扰源的距离
                    dist_end = sqrt(
                        pow(range_result.jammer_centre.X - unlock_end_pos.X, 2) +
                        pow(range_result.jammer_centre.Y - unlock_end_pos.Y, 2) +
                        pow(range_result.jammer_centre.Z - unlock_end_pos.Z, 2)
                    );
                    // 计算平均距离（失锁范围半径）
                    avg_dist = (dist_start + dist_end) / 2.0;
                    break; // 找到结束点后退出遍历
                }

                last_unlock = cur_unlock;
            }

            // 特殊情况：全程失锁（只找到开始点，未找到结束点）
            if (unlock_start_found && !unlock_end_found) {
                unlock_end_pos = sim.lla_to_ecef(track_points.back()); // 取最后一个航迹点作为结束点
                dist_end = sqrt(
                    pow(range_result.jammer_centre.X - unlock_end_pos.X, 2) +
                    pow(range_result.jammer_centre.Y - unlock_end_pos.Y, 2) +
                    pow(range_result.jammer_centre.Z - unlock_end_pos.Z, 2)
                );
                avg_dist = (dist_start + dist_end) / 2.0;
            }

            // 填充干扰范围结果（仅保留核心字段）
            range_result.jammer_radius = avg_dist;                  // 失锁范围半径（核心输出）
            range_result.jammer_start_point = unlock_start_pos;     // 初始失锁位置
            range_result.jammer_end_point = unlock_end_pos;         // 最终失锁位置
            range_result.jammer_start_lla = sim.ecef_to_lla(unlock_start_pos);
            range_result.jammer_end_lla = sim.ecef_to_lla(unlock_end_pos);

            jammer_range.push_back(range_result); // 存入结果列表
        }
    }

    void GNSSJammerSim::calc_jammer_area(const SimConfig& barrage_config, vector<JammerRangeResult>& jammer_range)
    {
        // ---------- 4.1 自动生成航迹（每个干扰源对应一条航迹） ----------
        Json::Value auto_platform_tracks(Json::arrayValue);

        // 为每个干扰源生成一条穿过其位置的航迹
        for (int jammer_idx = 0; jammer_idx < barrage_config.jammers.size(); jammer_idx++) {
            Json::Value platform_track;
            UINT platform_id = jammer_idx + 1; // 干扰源ID
            platform_track["platform_id"] = platform_id;

            // 生成穿过该干扰源的航迹点
            LLA jammer_pos = barrage_config.jammers[jammer_idx].pos;
            std::vector<LLA> track_points = generate_track_through_jammer(jammer_pos);

            // 转换为JSON格式的航迹点
            Json::Value track_points_json(Json::arrayValue);
            for (const auto& point : track_points) {
                Json::Value point_json;
                point_json["lon_deg"] = point.lon_deg;
                point_json["lat_deg"] = point.lat_deg;
                point_json["h_m"] = point.h_m; //千米
                track_points_json.append(point_json);
            }

            platform_track["track_points"] = track_points_json;
            auto_platform_tracks.append(platform_track);
        }

        // 使用自动生成的航迹进行计算
        GNSSJammerSim sim;
        process_platform_tracks(auto_platform_tracks, barrage_config, sim, jammer_range);
    }

    // ========================= 3. 使用示例 =========================
    //int Barrage_Test(Json::Value input, Json::Value& trajectory_result) {

    //    Jammer_Level jammer_strength = static_cast<Jammer_Level>(input["jammer_level"].asInt());

    //    // 若存在"jammer_num"则取其整数值，不存在则返回默认值0（可自定义）
    //    int jammer_num = input.get("jammer_num", 1).asInt();

    //    // 1. 初始化配置
    //    //SimConfig barrage_config;
    //    /*if (jammer_strength == Jammer_Level::High)
    //    {
    //        barrage_config.beta = 5e4;
    //    }
    //    else if (jammer_strength == Jammer_Level::Middle)
    //    {
    //        barrage_config.beta = 9e4;
    //    }
    //    else if (jammer_strength == Jammer_Level::Low)
    //    {
    //        barrage_config.beta = 2e5;
    //    }*/

    //    //干扰源添加
    //    for (int i = 0; i < jammer_num; i++)
    //    {
    //        //干扰位置计算


    //        // 配置干扰源
    //        JammerParam jammer;
    //        jammer.pos = { 120.0, 27.63, 8.3 };
    //        jammer.power = 10.0; // W
    //        jammer.type = "continuous_wave";
    //        jammer.bandwidth = 20e6;
    //        jammer.freq = GNSS_FC;
    //        barrage_config.jammers.push_back(jammer);
    //    }

    //    // 配置卫星参数
    //    barrage_config.satellite.carrier_power = 1e-16;
    //    barrage_config.satellite.sat_pos = {
    //        {125.0, 30.0, 5000},
    //        {115.0, 35.0, 5000},
    //        {130.0, 25.0, 5000},
    //        {110.0, 28.0, 5000}
    //    };

    //    // 配置干扰源
    //    //JammerParam jammer1;
    //    //jammer1.pos = {120.0, 27.63, 8.3};
    //    //jammer1.power = 10.0; // W
    //    //jammer1.type = "continuous_wave";
    //    //jammer1.bandwidth = 20e6;
    //    //jammer1.freq = GNSS_FC;
    //    //barrage_config.jammers.push_back(jammer1);

    //    /*JammerParam jammer2;
    //    jammer2.pos = { 119.75, 27.64, 8.3 };
    //    jammer2.power = 10.0;
    //    jammer2.type = "continuous_wave";
    //    jammer2.bandwidth = 1e6;
    //    jammer2.freq = GNSS_FC;
    //    barrage_config.jammers.push_back(jammer2);*/

    //    // 2. 待处理的航迹数据
    //    // 解析多平台航迹数据
    //    const Json::Value& platform_tracks = input["platform_tracks"];

    //    std::vector<LLA> track_points = {
    //        {119.045, 27.2233, 1.3},   // 航迹点1
    //    };

    //    LLA target_velocity = { 0.005, 0.003, 0 };

    //    UINT sim_time = 400;  // 仿真时长(s)200
    //    for (int step = 0; step < sim_time; ++step)
    //    {
    //        LLA target_pos;
    //        target_pos = track_points[0] + target_velocity * step;
    //        track_points.push_back(target_pos);
    //    }

    //    // 3. 初始化仿真类并计算
    //    GNSSJammerSim sim;
    //    std::vector<BarrageTrackResult> results = sim.batch_calc(track_points, barrage_config);

    //    // 4. 输出结果
    //    trajectory_result.clear();

    //    bool last_unlock_status = false;     //上一次失锁状态
    //    bool cur_unlock_status = false;     //当前失锁状态
    //    ECEF jammer_centre;
    //    ECEF jammer_start;                   //干扰开始点
    //    ECEF jammer_end;                     //干扰结束点
    //    double r_jammer = 0;                     //干扰半径

    //    //干扰点信息
    //    Json::Value& jammer_list = trajectory_result["jammer_list"];
    //    for (int cnt = 0; cnt < barrage_config.jammers.size(); cnt++)
    //    {
    //        Json::Value jammer_mes;
    //        jammer_mes["id"] = cnt + 1;
    //        jammer_mes["pos_lla"]["lon_deg"] = barrage_config.jammers[cnt].pos.lon_deg;
    //        jammer_mes["pos_lla"]["lat_deg"] = barrage_config.jammers[cnt].pos.lat_deg;
    //        jammer_mes["pos_lla"]["h_m"] = barrage_config.jammers[cnt].pos.h_m * 1000;      //km->m

    //        /*ECEF tmp_pos = sim.lla_to_ecef(barrage_config.jammers[cnt].pos);
    //        jammer_centre.X = tmp_pos.X;
    //        jammer_centre.Y = tmp_pos.Y;
    //        jammer_centre.Z = tmp_pos.Z;*/

    //        jammer_list.append(jammer_mes);
    //    }

    //    Json::Value& track_points_json = trajectory_result["track_points"];
    //    // 遍历航迹点结果，写入JSON
    //    for (int i = 0; i < results.size(); ++i) {
    //        // 定义单个航迹点的JSON对象，存储当前点的所有参数
    //        Json::Value track_point;

    //        // 航迹点序号(和原打印一致，i+1)
    //        track_point["step"] = i + 1;
    //        //目标点
    //        track_point["pos_tar_lla"]["lon_deg"] = track_points[i].lon_deg;
    //        track_point["pos_tar_lla"]["lat_deg"] = track_points[i].lat_deg;
    //        track_point["pos_tar_lla"]["h_m"] = track_points[i].h_m * 1000;

    //        // 载噪比(dB-Hz)
    //        track_point["cn0_dbhz"] = results[i].C_NJ_dB;
    //        // 干信比(dB)
    //        track_point["js_dB"] = results[i].J_S_dB;
    //        // 载波环误差(°)
    //        track_point["carrier_loop_error_deg"] = results[i].sigma_jpll;
    //        // 码环误差（保留原变量名，无单位则不标注）
    //        track_point["code_loop_error"] = results[i].sigma_jdll;
    //        // 失锁标志
    //        track_point["unlock_flag_bool"] = results[i].unlock_flag;  // 布尔值

    //        // 经纬度高定位误差（°/°/m，修正原代码km标注错误，h_m是米，贴合变量定义）
    //        track_point["pos_error_lla"]["lon_deg"] = results[i].pos_error.lon_deg;
    //        track_point["pos_error_lla"]["lat_deg"] = results[i].pos_error.lat_deg;
    //        track_point["pos_error_lla"]["h_m"] = results[i].pos_error.h_m * 1000;

    //        // X/Y/Z定位误差（米）
    //        track_point["pos_error_xyz_m"]["X"] = results[i].pos_error_m.X;
    //        track_point["pos_error_xyz_m"]["Y"] = results[i].pos_error_m.Y;
    //        track_point["pos_error_xyz_m"]["Z"] = results[i].pos_error_m.Z;

    //        // GDOP值
    //        //track_point["gdop"] = results[i].gdop;

    //        // 将当前航迹点对象加入数组，trajectory_result最终是JSON数组
    //        track_points_json.append(track_point);

    //        last_unlock_status = cur_unlock_status;
    //        cur_unlock_status = results[i].unlock_flag;
    //        if (!last_unlock_status || cur_unlock_status)
    //        {
    //            ECEF tmp_pos = sim.lla_to_ecef(track_points[i]);
    //            jammer_start.X = tmp_pos.X;
    //            jammer_start.Y = tmp_pos.Y;
    //            jammer_start.Z = tmp_pos.Z;
    //        }
    //        else if (last_unlock_status || !cur_unlock_status)
    //        {
    //            if (i > 0)
    //            {
    //                ECEF tmp_pos = sim.lla_to_ecef(track_points[i - 1]);
    //                jammer_end.X = tmp_pos.X;
    //                jammer_end.Y = tmp_pos.Y;
    //                jammer_end.Z = tmp_pos.Z;

    //                double r_1 = sqrt(pow((jammer_centre.X - jammer_start.X), 2) + pow((jammer_centre.Y - jammer_start.Y), 2) + pow((jammer_centre.Z - jammer_start.Z), 2));
    //                double r_2 = sqrt(pow((jammer_centre.X - jammer_end.X), 2) + pow((jammer_centre.Y - jammer_end.Y), 2) + pow((jammer_centre.Z - jammer_end.Z), 2));
    //                r_jammer = (r_1 + r_2) / 2;
    //            }
    //        }
    //    }

    //    //干扰范围
    //    Json::Value& jammer_area_json = trajectory_result["jammer area"];
    //    LLA jammer_pos_centra = sim.ecef_to_lla(jammer_centre);
    //    jammer_pos_centra.h_m = jammer_pos_centra.h_m * 1000;
    //    jammer_area_json["jammer pos centra"]["lon_deg"] = jammer_pos_centra.lon_deg;
    //    jammer_area_json["jammer pos centra"]["lat_deg"] = jammer_pos_centra.lat_deg;
    //    jammer_area_json["jammer pos centra"]["h_m"] = jammer_pos_centra.h_m;
    //    jammer_area_json["jammer r"] = r_jammer; //m

    //    return 0;
    //}

    int Barrage_Test_1(CalcTempParam& task_param, SimConfig barrage_config) {

        // 1. 待处理的航迹数据

        //初始化仿真类
        GNSSJammerSim sim;
        task_param.trajectory_result.clear();

        for (int i = 0; i < task_param.serveral_plat.size(); i++) {
            UINT platform_id = task_param.serveral_plat[i].plat_id;

            // 转换JSON航迹点到内部格式
            std::vector<LLA> track_points;
            for (int j = 0; j < task_param.return_frames; j++) {

                LLA target_pos;
                target_pos.lon_deg = task_param.serveral_plat[i].cur_plat_pos.lon_deg + task_param.serveral_plat[i].cur_plat_vec.lon_deg * j;
                target_pos.lat_deg = task_param.serveral_plat[i].cur_plat_pos.lat_deg + task_param.serveral_plat[i].cur_plat_vec.lat_deg * j;
                target_pos.h_m = task_param.serveral_plat[i].cur_plat_pos.h_m + task_param.serveral_plat[i].cur_plat_vec.h_m * j;
                track_points.push_back(target_pos);

                //记录运行固定帧数后的位置
                if (j == task_param.return_frames - 1)
                {
                    task_param.serveral_plat[i].cur_plat_pos = target_pos + task_param.serveral_plat[i].cur_plat_vec;
                }
            }

            //仿真计算
            std::vector<BarrageTrackResult> results = sim.batch_calc(track_points, barrage_config);

            // 4. 输出结果
            
            // 构造该平台的结果
            Json::Value platform_result;
            platform_result["platform_id"] = platform_id;

            Json::Value& track_results = platform_result["track_results"];
            //Json::Value& track_result_json = trajectory_result["track_points"];
            // 遍历航迹点结果，写入JSON
            for (int i = 0; i < results.size(); ++i) {
                // 定义单个航迹点的JSON对象，存储当前点的所有参数
                Json::Value track_point;

                // 航迹点序号(和原打印一致，i+1)
                track_point["step"] = task_param.run_frames + i;
                //目标点
                track_point["pos_tar_lla"]["lon_deg"] = track_points[i].lon_deg;
                track_point["pos_tar_lla"]["lat_deg"] = track_points[i].lat_deg;
                track_point["pos_tar_lla"]["h_m"] = track_points[i].h_m * 1000;

                // 载噪比(dB-Hz)
                track_point["cn0_dbhz"] = results[i].C_NJ_dB;
                // 干信比(dB)
                track_point["js_dB"] = results[i].J_S_dB;
                // 载波环误差(°)
                track_point["carrier_loop_error_deg"] = results[i].sigma_jpll;
                // 码环误差（保留原变量名，无单位则不标注）
                track_point["code_loop_error"] = results[i].sigma_jdll;
                // 干扰有效标志
                track_point["barrage_valid"] = results[i].jam_valid_flag;
                // 失锁标志
                track_point["unlock_flag_bool"] = results[i].unlock_flag;  // 布尔值

                // 经纬度高定位误差（°/°/m，修正原代码km标注错误，h_m是米，贴合变量定义）
                track_point["pos_error_lla"]["lon_deg"] = results[i].pos_error.lon_deg;
                track_point["pos_error_lla"]["lat_deg"] = results[i].pos_error.lat_deg;
                track_point["pos_error_lla"]["h_m"] = results[i].pos_error.h_m * 1000;

                // X/Y/Z定位误差（米）
                track_point["pos_error_xyz_m"]["X"] = results[i].pos_error_m.X;
                track_point["pos_error_xyz_m"]["Y"] = results[i].pos_error_m.Y;
                track_point["pos_error_xyz_m"]["Z"] = results[i].pos_error_m.Z;

                // GDOP值
                //track_point["gdop"] = results[i].gdop;

                // 将当前航迹点对象加入数组，trajectory_result最终是JSON数组
                track_results.append(track_point);
            }
            task_param.trajectory_result.append(platform_result);
        }
        //运行帧数计算
        task_param.run_frames = task_param.run_frames + task_param.return_frames;
        return 0;
    }

    int Barrage_CalcjammerArea(const SimConfig& barrage_config, vector<JammerRangeResult>& jammer_range)
    {
        GNSSJammerSim sim;
        sim.calc_jammer_area(barrage_config, jammer_range);
        return 0;
    }
}