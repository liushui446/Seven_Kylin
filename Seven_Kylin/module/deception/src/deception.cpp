#include "deception/deception.hpp"
//#include "core/json.hpp"
//
//using namespace Json;
#include <numeric>
#include <cmath>
#include <Eigen/Dense>  // 需Eigen库处理矩阵运算（GDOP计算）

namespace seven {

    // 角度转弧度
    double GNSSDeceptionError::deg2rad(double deg) const {
        return deg * M_PI / 180.0;
    }

    // 弧度转角度
    double GNSSDeceptionError::rad2deg(double rad) const {
        return rad * 180.0 / M_PI;
    }

    // LLA转ECEF(米)
    ECEF GNSSDeceptionError::lla_to_ecef(const LLA& lla) const {
        double lat_rad = deg2rad(lla.lat_deg);
        double lon_rad = deg2rad(lla.lon_deg);
        double h_m = lla.h_m * 1000.0;

        double e2 = 2 * F_ratio - F_ratio * F_ratio;
        double N = A_length / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));

        ECEF ecef;
        ecef.X = (N + h_m) * cos(lat_rad) * cos(lon_rad);
        ecef.Y = (N + h_m) * cos(lat_rad) * sin(lon_rad);
        ecef.Z = ((1 - e2) * N + h_m) * sin(lat_rad);
        return ecef;
    }

    // ECEF转LLA
    LLA GNSSDeceptionError::ecef_to_lla(const ECEF& ecef) const {
        double x = ecef.X;
        double y = ecef.Y;
        double z = ecef.Z;

        double e2 = 2 * F_ratio - F_ratio * F_ratio;
        double b = A_length * (1 - F_ratio);
        double ep2 = (A_length * A_length - b * b) / (b * b);

        double p = sqrt(x * x + y * y);
        LLA lla;

        // 处理极区情况
        if (p < 1e-12) {
            lla.lon_deg = 0.0;
            lla.lat_deg = rad2deg(M_PI / 2 * (z >= 0 ? 1 : -1));
            lla.h_m = (fabs(z) - b) / 1000.0;
            return lla;
        }

        double theta = atan2(z * A_length, p * b);
        double lon_rad = atan2(y, x);
        double lat_rad = atan2(z + ep2 * b * pow(sin(theta), 3),
            p - e2 * A_length * pow(cos(theta), 3));

        double N = A_length / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
        double h_m = (p / cos(lat_rad)) - N;

        lla.lon_deg = rad2deg(lon_rad);
        lla.lat_deg = rad2deg(lat_rad);
        lla.h_m = h_m / 1000.0;

        return lla;
    }

    // 计算两点间ECEF距离(米)
    double GNSSDeceptionError::calc_ecef_distance(const ECEF& p1, const ECEF& p2) const {
        double dx = p1.X - p2.X;
        double dy = p1.Y - p2.Y;
        double dz = p1.Z - p2.Z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    LLA GNSSDeceptionError::interpolate_deception_points(
        const LLA& start_pos,
        const LLA& target_deception_pos,
        int t,
        const LLA& target_velocity
    ) {
        // 起点
        double lon0 = start_pos.lon_deg;
        double lat0 = start_pos.lat_deg;
        double h0 = start_pos.h_m;

        // 方向向量（起点 -> 欺骗终点）
        double dx = target_deception_pos.lon_deg - lon0;
        double dy = target_deception_pos.lat_deg - lat0;

        // 总距离
        double dist_total = std::sqrt(dx * dx + dy * dy);

        // 避免除 0
        if (dist_total < 1e-12) {
            return start_pos;
        }

        // 单位方向向量
        double ux = dx / dist_total;
        double uy = dy / dist_total;

        // 目标真实飞行的距离
        double vx = target_velocity.lon_deg;
        double vy = target_velocity.lat_deg;
        double dist_real = std::sqrt((vx * t) * (vx * t) + (vy * t) * (vy * t));

        // 计算当前欺骗点
        double lon = lon0 + ux * dist_real;
        double lat = lat0 + uy * dist_real;
        double h = h0;

        LLA cur_deception_pos = { lon, lat, h, t };

        return cur_deception_pos;
    }

    // 筛选GDOP最优的4颗卫星
    std::vector<LLA> GNSSDeceptionError::select_optimal_satellites(const LLA& target_pos) const {
        // 转换目标到ECEF
        ECEF target_ecef = lla_to_ecef(target_pos);
        std::vector<ECEF> sat_ecef_list;
        for (const auto& sat : params.satellite_pos) {
            sat_ecef_list.push_back(lla_to_ecef(sat));
        }

        double min_gdop = INFINITY;
        std::vector<int> best_idx = { 0, 1, 2, 3 };  // 默认前4颗

        // 生成所有4颗卫星组合(简化实现：仅遍历前10选4，实际可优化)
        std::vector<int> indices(params.satellite_pos.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::vector<bool> mask(indices.size(), false);
        std::fill(mask.end() - 4, mask.end(), true);

        do {
            std::vector<int> combo;
            for (int i = 0; i < indices.size(); ++i) {
                if (mask[i]) combo.push_back(indices[i]);
            }
            if (combo.size() != 4) continue;

            // 构建几何矩阵G
            Eigen::MatrixXd G(4, 4);
            bool valid = true;
            for (int i = 0; i < 4; ++i) {
                ECEF sat_ecef = sat_ecef_list[combo[i]];
                double dx = sat_ecef.X - target_ecef.X;
                double dy = sat_ecef.Y - target_ecef.Y;
                double dz = sat_ecef.Z - target_ecef.Z;
                double dist = sqrt(dx * dx + dy * dy + dz * dz);

          if (dist < 1e-6 || std::isnan(dist)) {
                    valid = false;
                    break;
                }

                G(i, 0) = dx / dist;
                G(i, 1) = dy / dist;
                G(i, 2) = dz / dist;
                G(i, 3) = 1.0;
            }

            if (!valid) continue;

            // 计算GDOP
            Eigen::MatrixXd A = G.transpose() * G;
            //if (A.conditionNumber() > 1e12) continue;  // 条件数过大

            try {
                Eigen::MatrixXd A_inv = A.inverse();
                double gdop = sqrt(A_inv.trace());
                if (gdop < min_gdop) {
                    min_gdop = gdop;
                    best_idx = combo;
                }
            }
            catch (...) {
                continue;
            }
        } while (std::next_permutation(mask.begin(), mask.end()));

        // 提取最优卫星
        std::vector<LLA> optimal_sats;
        for (int idx : best_idx) {
            optimal_sats.push_back(params.satellite_pos[idx]);
        }
        return optimal_sats;
    }

    // 计算信号相对功率
    double GNSSDeceptionError::calc_signal_power(const LLA& trans_pos, const LLA& recv_pos, double freq) const {
        ECEF trans_ecef = lla_to_ecef(trans_pos);
        ECEF recv_ecef = lla_to_ecef(recv_pos);
        double dist = calc_ecef_distance(trans_ecef, recv_ecef);

        if (dist < 1e-6) return 0.0;

        // 自由空间损耗公式
        double loss = pow(4 * M_PI * dist * FREQ / C_LIGHT, 2);
        return 1.0 / loss;
    }

    // 判断欺骗信号是否有效
    bool GNSSDeceptionError::is_deception_valid(const LLA& target_pos, const std::vector<LLA>& satellite_pos) const {
        // 计算真实卫星平均功率
        double real_power_sum = 0.0;
        for (const auto& sat : satellite_pos) {
            real_power_sum += calc_signal_power(sat, target_pos, FREQ);
        }
        double real_power = real_power_sum / satellite_pos.size();

        // 计算欺骗信号平均功率
        double deception_power_sum = 0.0;
        for (const auto& jammer : params.jammers) {
            deception_power_sum += calc_signal_power(jammer.pos, target_pos, jammer.freq);
        }
        double deception_power = deception_power_sum / params.jammers.size();

        //if (real_power < 1e-12) return false;

        // 功率比转dB
        double power_ratio_dB = 10 * log10(deception_power / real_power);
        return power_ratio_dB > params.power_ratio_threshold;
    }

    // 计算欺骗时延
    std::vector<double> GNSSDeceptionError::calculate_deception_delay(const LLA& target_pos,
        const std::vector<LLA>& satellite_pos) const {
        ECEF deception_ecef = lla_to_ecef(params.deception_pos);
        ECEF target_ecef = lla_to_ecef(target_pos);
        std::vector<double> delays;

        for (int i = 0; i < params.jammer_num; ++i) {
            ECEF jammer_ecef = lla_to_ecef(params.jammers[i].pos);
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);

            double R_SD = calc_ecef_distance(sat_ecef, deception_ecef);
            double R_SJ = calc_ecef_distance(sat_ecef, jammer_ecef);
            double R_JR = calc_ecef_distance(jammer_ecef, target_ecef);

            double tau = (R_SD - R_SJ - R_JR) / C_LIGHT;
            delays.push_back(tau);
        }

        // 时延修正：确保非负
        double min_tau = *std::min_element(delays.begin(), delays.end());
        if (min_tau < 0) {
            for (auto& tau : delays) {
                tau += fabs(min_tau);
            }
        }

        return delays;
    }

    // 求解定位误差
    LLA GNSSDeceptionError::solve_position_error(const std::vector<LLA>& satellite_pos,
        const LLA& target_pos,
        const std::vector<double>& delays) const {
        ECEF target_ecef = lla_to_ecef(target_pos);
        int n = params.jammer_num;

        // 构建观测矩阵M
        Eigen::MatrixXd M(n, 4);
        std::vector<double> R_SR_list;

        for (int i = 0; i < n; ++i) {
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);
            double dx = sat_ecef.X - target_ecef.X;
            double dy = sat_ecef.Y - target_ecef.Y;
            double dz = sat_ecef.Z - target_ecef.Z;
            double R_SR = sqrt(dx * dx + dy * dy + dz * dz);

            if (R_SR < 1e-6 || std::isnan(R_SR)) {
                return { 0.0, 0.0, 0.0 };
            }

            R_SR_list.push_back(R_SR);
            double unit_x = dx / R_SR;
            double unit_y = dy / R_SR;
            double unit_z = dz / R_SR;

            M(i, 0) = unit_x / C_LIGHT;
            M(i, 1) = unit_y / C_LIGHT;
            M(i, 2) = unit_z / C_LIGHT;
            M(i, 3) = 1.0;
        }

        // 构建时延向量T
        Eigen::VectorXd T(n);
        for (int i = 0; i < n; ++i) {
            T(i) = delays[i];
        }

        // 构建修正向量A
        Eigen::VectorXd A(n);
        for (int i = 0; i < n; ++i) {
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);
            ECEF jammer_ecef = lla_to_ecef(params.jammers[i].pos);

            double R_SJ = calc_ecef_distance(sat_ecef, jammer_ecef);
            double R_JR = calc_ecef_distance(jammer_ecef, target_ecef);
            double R_SR = R_SR_list[i];

            A(i) = (R_SJ + R_JR - R_SR) / C_LIGHT;
        }

        // 最小二乘求解
        try {
            Eigen::MatrixXd MtM = M.transpose() * M;
            /*if (MtM.conditionNumber() > 1e12) {
                return { 0.0, 0.0, 0.0 };
            }*/

            Eigen::MatrixXd MtM_inv = MtM.inverse();
            Eigen::VectorXd delta_X = MtM_inv * M.transpose() * (T + A);

            // 提取位置误差(米)
            ECEF error_ecef;
            error_ecef.X = delta_X(0);
            error_ecef.Y = delta_X(1);
            error_ecef.Z = delta_X(2);

            // 计算受干扰后的位置
            ECEF new_target_ecef;
            new_target_ecef.X = target_ecef.X + error_ecef.X;
            new_target_ecef.Y = target_ecef.Y + error_ecef.Y;
            new_target_ecef.Z = target_ecef.Z + error_ecef.Z;

            LLA new_target_lla = ecef_to_lla(new_target_ecef);

            // 计算误差
            LLA pos_error;
            pos_error.lon_deg = new_target_lla.lon_deg - target_pos.lon_deg;
            pos_error.lat_deg = new_target_lla.lat_deg - target_pos.lat_deg;
            pos_error.h_m = new_target_lla.h_m - target_pos.h_m;

            return pos_error;
        }
        catch (...) {
            return { 0.0, 0.0, 0.0 };
        }
    }

    // 设置自定义参数
    void GNSSDeceptionError::set_params(const SimParams& deception_config) {
        params = deception_config;
    }

    // 核心计算接口：输入航迹点，输出误差结果
    TrackResult GNSSDeceptionError::calculate_error(const LLA& target_pos) {
        TrackResult result;
        result.target_pos = target_pos;
        result.deception_pos = params.deception_pos;
        result.deception_valid = false;

        // 1. 筛选最优卫星
        std::vector<LLA> optimal_sats = select_optimal_satellites(target_pos);

        // 2. 判定欺骗有效性
        if (!is_deception_valid(target_pos, optimal_sats)) {
            result.error_pos = target_pos;
            result.pos_error = { 0.0, 0.0, 0.0 };
            return result;
        }
        result.deception_valid = true;

        // 3. 计算欺骗时延
        std::vector<double> delays = calculate_deception_delay(target_pos, optimal_sats);

        // 4. 求解定位误差
        result.pos_error = solve_position_error(optimal_sats, target_pos, delays);

        // 5. 计算受干扰后的位置
        result.error_pos.lon_deg = target_pos.lon_deg - result.pos_error.lon_deg;
        result.error_pos.lat_deg = target_pos.lat_deg - result.pos_error.lat_deg;
        result.error_pos.h_m = target_pos.h_m - result.pos_error.h_m;

        return result;
    }

    // 批量处理航迹数据
    std::vector<TrackResult> GNSSDeceptionError::batch_calculate(const std::vector<LLA>& track_points) {
        std::vector<TrackResult> results;
        for (const auto& point : track_points) {
            LLA cur_deception_pos = interpolate_deception_points(params.plat_initial_pos, params.target_deception_pos, point.current_run_t, params.cur_plat_vec);
            params.deception_pos = cur_deception_pos;
            results.push_back(calculate_error(point));
        }
        return results;
    }


    // 示例使用
    //int Deception_Test(Json::Value input, Json::Value& trajectory_result) {
    //    
    //    Jammer_Level jammer_strength = static_cast<Jammer_Level>(input.get("jammer_level", 2).asInt());

    //    // 若存在"jammer_num"则取其整数值，不存在则返回默认值0（可自定义）
    //    int jammer_num = input.get("jammer_num", 1).asInt();

    //    // 1. 创建计算实例
    //    GNSSDeceptionError gnss_error;

    //    // 2. (可选)自定义参数
    //    SimParams deception_config;
    //    deception_config.jammer_num = jammer_num;
    //    //干扰源添加
    //    for (int i = 0; i < jammer_num; i++)
    //    {
    //        LLA lla_jam_pos;
    //        deception_config.jammer_pos.push_back(lla_jam_pos);
    //    }

    //    //deception_config.deception_pos = {115.32, 29.15, 1.5};  // 修改欺骗点
    //    //gnss_error.set_params(deception_config);

    //    if (jammer_strength == Jammer_Level::High)
    //    {
    //        deception_config.deception_pos = { 115.32, 29.33, 0.5 };  // 修改欺骗点
    //        gnss_error.set_params(deception_config);
    //    }
    //    else if (jammer_strength == Jammer_Level::Middle)
    //    {
    //        deception_config.deception_pos = { 115.32, 29.23, 0.5 };  // 修改欺骗点
    //        gnss_error.set_params(deception_config);
    //    }
    //    else if (jammer_strength == Jammer_Level::Low)
    //    {
    //        deception_config.deception_pos = { 115.32, 29.15, 0.5 };  // 修改欺骗点
    //        gnss_error.set_params(deception_config);
    //    }

    //    // 3. 输入航迹数据
    //    std::vector<LLA> track_points = {
    //        {115.193, 29.027, 1},
    //    };

    //    LLA target_velocity = { 0.001, 0.0005, 0 };

    //    UINT sim_time = 100;  // 仿真时长(s)200

    //    LLA inital_pos = track_points[0];
    //    for (int step = 0; step < sim_time; step++)
    //    {
    //        LLA target_pos;
    //        target_pos = inital_pos + target_velocity * step;
    //        track_points.push_back(target_pos);
    //    }

    //    // 4. 批量计算
    //    std::vector<TrackResult> results = gnss_error.batch_calculate(track_points);

    //    // 5. 清空并写入航迹点数组（核心：对接欺骗式干扰的结果字段）
    //    trajectory_result.clear(); // 可选，复用对象时建议保留
    //    //干扰点信息
    //    Json::Value& jammer_list = trajectory_result["jammer_list"];
    //    for (int cnt = 0; cnt < deception_config.jammer_pos.size(); cnt++)
    //    {
    //        Json::Value jammer_mes;
    //        jammer_mes["id"] = cnt + 1;
    //        jammer_mes["pos_lla"]["lon_deg"] = deception_config.jammer_pos[cnt].lon_deg;
    //        jammer_mes["pos_lla"]["lat_deg"] = deception_config.jammer_pos[cnt].lat_deg;
    //        jammer_mes["pos_lla"]["h_m"] = deception_config.jammer_pos[cnt].h_m * 1000;      //km->m

    //        /*ECEF tmp_pos = sim.lla_to_ecef(config.jammers[cnt].pos);
    //        jammer_centre.X = tmp_pos.X;
    //        jammer_centre.Y = tmp_pos.Y;
    //        jammer_centre.Z = tmp_pos.Z;*/

    //        jammer_list.append(jammer_mes);
    //    }

    //    Json::Value& track_points_json = trajectory_result["track_points"]; // 航迹点数组引用，简化书写

    //    for (size_t i = 0; i < results.size(); ++i) {
    //        const auto& res = results[i]; // 复用原代码的常量引用，避免拷贝
    //        Json::Value track_point;      // 单个航迹点的JSON对象

    //        // 基础字段：航迹点序号（和原打印一致，i+1）
    //        track_point["index"] = static_cast<int>(i + 1); // size_t转int，适配JsonCpp

    //        // 目标位置（经纬度高，°/°/m，修正原km标注，贴合h_m变量定义）
    //        track_point["target_pos"]["lon_deg"] = res.target_pos.lon_deg; // 经度(°)
    //        track_point["target_pos"]["lat_deg"] = res.target_pos.lat_deg; // 纬度(°)
    //        track_point["target_pos"]["h_m"] = res.target_pos.h_m;         // 高度(m)，原始单位

    //        // 受干扰位置（经纬度高，°/°/m，修正原km标注，贴合h_m变量定义）
    //        track_point["error_pos"]["lon_deg"] = res.error_pos.lon_deg; // 经度(°)
    //        track_point["error_pos"]["lat_deg"] = res.error_pos.lat_deg; // 纬度(°)
    //        track_point["error_pos"]["h_m"] = res.error_pos.h_m;         // 高度(m)，原始单位

    //        // 欺骗有效标志：布尔值+字符串，兼顾程序解析和人工查看（和原打印一致）
    //        track_point["deception_valid_bool"] = res.deception_valid; // 布尔值（推荐，解析友好）

    //        // 定位误差（经纬度高，°/°/m，同目标位置的单位规范）
    //        track_point["pos_error"]["lon_deg"] = res.pos_error.lon_deg; // 经度误差(°)
    //        track_point["pos_error"]["lat_deg"] = res.pos_error.lat_deg; // 纬度误差(°)
    //        track_point["pos_error"]["h_m"] = res.pos_error.h_m;         // 高度误差(m)，原始单位
    //        // 可选：高度误差转千米，新增h_m字段

    //        // 将当前航迹点加入数组
    //        track_points_json.append(track_point);
    //    }

    //    return 0;
    //}

    int Deception_Use(CalcTempParam& task_param, SimParams& deception_config) {

        //Jammer_Level jammer_strength = static_cast<Jammer_Level>(input.get("jammer_level", 2).asInt());

        // 1. 创建计算实例
        GNSSDeceptionError gnss_error;
        // 2. 清空并写入航迹点数组（核心：对接欺骗式干扰的结果字段）
        task_param.trajectory_result.clear(); // 可选，复用对象时建议保留

        // 3. 解析多平台航迹数据
        for (int i = 0; i < task_param.serveral_plat.size(); i++) {
            UINT platform_id = task_param.serveral_plat[i].plat_id;
            
            // 转换JSON航迹点到内部格式
            std::vector<LLA> track_points;
            for (int j = 1; j < task_param.return_frames; j++) {

                LLA target_pos;
                target_pos.lon_deg = task_param.serveral_plat[i].cur_plat_pos.lon_deg + task_param.serveral_plat[i].cur_plat_vec.lon_deg * j;
                target_pos.lat_deg = task_param.serveral_plat[i].cur_plat_pos.lat_deg + task_param.serveral_plat[i].cur_plat_vec.lat_deg * j;
                target_pos.h_m = task_param.serveral_plat[i].cur_plat_pos.h_m + task_param.serveral_plat[i].cur_plat_vec.h_m * j;
                target_pos.current_run_t = task_param.run_frames + j;
                track_points.push_back(target_pos);

                if (j == task_param.return_frames - 1)
                {
                    task_param.serveral_plat[i].cur_plat_pos = target_pos + task_param.serveral_plat[i].cur_plat_vec;
                }
            }

            //当前平台初始位置和速度初始化
            deception_config.plat_initial_pos = task_param.serveral_plat[i].plat_initial_pos;
            deception_config.cur_plat_vec = task_param.serveral_plat[i].cur_plat_vec;
            // 4. 欺骗点设置参数设置
            deception_config.target_deception_pos = task_param.serveral_plat[i].deception_pos;
            std::cout << "当前平台欺骗位置:{" << to_string(deception_config.target_deception_pos.lon_deg) << ", " << to_string(deception_config.target_deception_pos.lat_deg) << ", " << to_string(deception_config.target_deception_pos.h_m) << " }" << std::endl;
            gnss_error.set_params(deception_config);

            // 5. 批量计算
            std::vector<TrackResult> results = gnss_error.batch_calculate(track_points);

            // 6. 构造该平台的结果
            Json::Value platform_result;
            platform_result["platform_id"] = platform_id;

            Json::Value& track_results = platform_result["track_results"];
            for (size_t i = 0; i < results.size(); ++i) {
                const auto& res = results[i]; // 复用原代码的常量引用，避免拷贝
                Json::Value track_point;      // 单个航迹点的JSON对象

                // 基础字段：航迹点序号（和原打印一致，i+1）
                track_point["index"] = static_cast<int>(task_param.run_frames + i); // size_t转int，适配JsonCpp

                // 目标位置（经纬度高，°/°/m，修正原km标注，贴合h_m变量定义）
                track_point["target_pos"]["lon_deg"] = res.target_pos.lon_deg; // 经度(°)
                track_point["target_pos"]["lat_deg"] = res.target_pos.lat_deg; // 纬度(°)
                track_point["target_pos"]["h_m"] = res.target_pos.h_m * 1000;         // 高度(m)，原始单位

                // 受干扰位置（经纬度高，°/°/m，修正原km标注，贴合h_m变量定义）
                track_point["error_pos"]["lon_deg"] = res.error_pos.lon_deg; // 经度(°)
                track_point["error_pos"]["lat_deg"] = res.error_pos.lat_deg; // 纬度(°)
                track_point["error_pos"]["h_m"] = res.error_pos.h_m * 1000;         // 高度(m)，原始单位

                // 欺骗有效标志：布尔值+字符串，兼顾程序解析和人工查看（和原打印一致）
                track_point["deception_valid_bool"] = res.deception_valid; // 布尔值（推荐，解析友好）

                // 定位误差（经纬度高，°/°/m，同目标位置的单位规范）
                track_point["pos_error"]["lon_deg"] = res.pos_error.lon_deg; // 经度误差(°)
                track_point["pos_error"]["lat_deg"] = res.pos_error.lat_deg; // 纬度误差(°)
                track_point["pos_error"]["h_m"] = res.pos_error.h_m * 1000;         // 高度误差(m)，原始单位

                // 当前欺骗点（经纬度高，°/°/m，同目标位置的单位规范）
                track_point["deception_pos"]["lon_deg"] = res.deception_pos.lon_deg; // 经度误差(°)
                track_point["deception_pos"]["lat_deg"] = res.deception_pos.lat_deg; // 纬度误差(°)
                track_point["deception_pos"]["h_m"] = res.deception_pos.h_m * 1000;         // 高度误差(m)，原始单位

                // 将当前航迹点加入数组
                track_results.append(track_point);
            }
            task_param.trajectory_result.append(platform_result);
        }
        //运行帧数计算
        task_param.run_frames = task_param.run_frames + task_param.return_frames;
        return 0;
    }

}
