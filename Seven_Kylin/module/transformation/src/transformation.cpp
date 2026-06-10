#include "transformation/transformation.hpp"

#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>

using namespace Json;

namespace seven {

    // ====================== 全局基础参数 (若未在头文件定义) ======================
    // 请确保这些参数与头文件或Python代码一致
    // const double TRANSITION_SPEED = 0.08;
    // const double COLLISION_RADIUS = 4.0;
    // const int MAX_COLLISION_ITER = 15;
    // const double MAX_ADJUST_STEP = 1.0;
    // const double ERROR_STABLE_THRESHOLD = 0.02;
    // const double MAX_SPEED = 5.0;
    // const double R_EARTH = 6378137.0;

    // 多编队仿真器映射表：key = formation_id, value = 仿真器实例
    std::unordered_map<int, UUVFormationSimulator*> g_FormationSimulators;

    string formationToStr(Formation_Type type)
    {
        string formation_str = "";
        switch (type)
        {
        case Formation_Type::Circle:
            formation_str = "Circle";
            break;
        case Formation_Type::Diamond:
            formation_str = "Diamond";
            break;
        case Formation_Type::Line:
            formation_str = "Line";
            break;
        case Formation_Type::Rectangle:
            formation_str = "Rectangle";
            break;
        case Formation_Type::Triangle:
            formation_str = "Triangle";
            break;
        case Formation_Type::Custom:
            formation_str = "Custom";
            break;
        default:
            break;
        }
        return formation_str;
    }

    // 角度转弧度
    inline double to_radians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    // 弧度转角度
    inline double to_degrees(double radians) {
        return radians * 180.0 / M_PI;
    }

    // ====================== UUVFormationSimulator 实现 ======================
    UUVFormationSimulator::UUVFormationSimulator(const FormationConfig& cfg, CustomFormationList& custom_data)
        : config(cfg), current_time(0.0), last_output_time(0.0),
        is_transition(false), last_formation(cfg.current_formation), max_id(0) {
        trajectory_.clearAllTrajectory();
        _validate_config();
        _set_custom_data_list(custom_data);
        _init_nodes();
        trajectory_.addFrame((current_time * 10), config.trans_formation, nodes);
    }

    void UUVFormationSimulator::_validate_config() {
        if (config.node_num < 2 || config.node_num > 10) {
            throw std::invalid_argument("节点数量必须在2~10之间，当前值：" + std::to_string(config.node_num));
        }
        /*if (config.rel_distance <= 0.0) {
            throw std::invalid_argument("节点间距必须大于0");
        }*/
    }

    void UUVFormationSimulator::_init_nodes() {
        // 初始化主节点
        UUVNode main_node;
        main_node.id = 0;
        main_node.pos_ = { config.main_node.lon_deg, config.main_node.lat_deg };
        main_node.speed = config.init_speed;
        main_node.heading = config.init_heading;
        // 初始化新增字段
        main_node.is_joining = false;
        main_node.is_leaving = false;
        nodes.push_back(main_node);

        // 初始化从节点
        for (int i = 1; i < config.node_num; ++i) {
            UUVNode node;
            node.id = i;
            node.pos_ = { config.main_node.lon_deg, config.main_node.lat_deg };
            node.speed = config.init_speed;
            node.heading = config.init_heading;
            // 初始化新增字段
            node.is_joining = false;
            node.is_leaving = false;
            nodes.emplace_back(node);
        }

        max_id = config.node_num - 1; // 初始化最大ID
        _set_target_formation();
        _set_initial_position();

        // 初始化last_rel_x/y
        for (auto& node : nodes) {
            node.last_rel_x = node.rel_x;
            node.last_rel_y = node.rel_y;
        }
    }

    void UUVFormationSimulator::_set_custom_data_list(CustomFormationList& data) {
        for (auto &pair : data)
        {
            custom_data.emplace(pair.first, pair.second);
        }
    }

    void UUVFormationSimulator::_set_initial_position() {

        nodes[0].rel_x = 0.0;
        nodes[0].rel_y = 0.0;

        // 主节点经纬度（基准点）
        double ref_lon = config.main_node.lon_deg;
        double ref_lat = config.main_node.lat_deg;

        // 遍历所有从节点
        for (size_t i = 1; i < nodes.size(); ++i) {
            auto& node = nodes[i];

            // 1. 设置初始相对坐标 = 目标坐标
            node.rel_x = node.target_x;
            node.rel_y = node.target_y;

            // ======================================================
            // 核心：根据相对坐标 rel_x / rel_y 计算真实经纬度
            // ======================================================
            auto geo = _enu2geo(node.rel_x, node.rel_y, ref_lon, ref_lat);

            // 2. 直接赋值给节点经纬度
            node.pos_.lon_deg = geo.first;
            node.pos_.lat_deg = geo.second;
        }
    }

    bool UUVFormationSimulator::switch_formation(const Formation_Type& cmd) {
        std::lock_guard<std::mutex> lock(sim_mutex);
        if (cmd == config.current_formation) {
            printf("当前已是该队形\n");
            return false;
        }

        last_formation = config.current_formation;
        config.trans_formation = cmd;
        if(cmd == Formation_Type::Custom)
        {
            if(config.node_num != custom_data[config.custom_id].node_num)
            {
                printf("⚠️ 自定义队形节点数与编队配置不匹配！请检查\n");
                return false;
            }
        }

        _set_target_formation();
        is_transition = true;
        return true;
    }

    // ====================== 【新增】设置目标队形（过滤脱离节点） ======================
    void UUVFormationSimulator::_set_target_formation() {
        // 1. 过滤：需要参与编队的从节点（排除脱离节点）
        std::vector<UUVNode*> valid_slaves;
        for (size_t i = 1; i < nodes.size(); ++i) {
            if (!nodes[i].is_leaving) {
                valid_slaves.push_back(&nodes[i]);
            }
        }

        int slave_count = static_cast<int>(valid_slaves.size());
        if (slave_count <= 0) {
            return;
        }

        // 2. 生成新队形的所有目标点位
        auto target_positions = _generate_formation_positions(slave_count);
        if (target_positions.empty()) {
            return;
        }

        // ================================
        // 【核心修改】最近邻目标分配
        // 每个节点 → 分配离自己当前位置最近的目标点
        // ================================
        std::vector<std::pair<double, double>> available_targets = target_positions;

        for (UUVNode* node : valid_slaves) {
            int best_idx = 0;
            float min_dist = 1e9;

            // 寻找离当前节点最近的目标点
            for (int i = 0; i < available_targets.size(); ++i) {
                float tx = available_targets[i].first;
                float ty = available_targets[i].second;

                float dx = node->rel_x - tx;
                float dy = node->rel_y - ty;
                float dist = sqrt(dx * dx + dy * dy);

                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = i;
                }
            }

            // 分配最近目标点，并从候选列表移除（避免重复分配）
            node->target_x = available_targets[best_idx].first;
            node->target_y = available_targets[best_idx].second;

            // 从可用目标中删除已分配的点
            available_targets.erase(available_targets.begin() + best_idx);
        }

        // 3. 正在脱离的节点：保持自己的脱离目标
        for (size_t i = 1; i < nodes.size(); ++i) {
            UUVNode& node = nodes[i];
            if (node.is_leaving) {
                node.target_x = node.leave_target_x;
                node.target_y = node.leave_target_y;
                printf("脱离点消失位置更新为：%.1f, %.1f\n", node.target_x, node.target_y);
            }
        }
    }

    // ====================== 【修改】队形过渡（处理加入节点） ======================
    void UUVFormationSimulator::_transition_formation() {
        for (size_t i = 1; i < nodes.size(); ++i) {
            UUVNode& node = nodes[i];

            if (node.is_joining) {
                node.join_progress = std::min(1.0, node.join_progress + 1.0 / node.join_total_frames);
                node.rel_x += (node.target_x - node.rel_x) * 0.1;
                node.rel_y += (node.target_y - node.rel_y) * 0.1;

                if (node.join_progress >= 1.0) {
                    node.is_joining = false;
                    printf("✅ 节点 ID:%d 已完全加入编队\n", node.id);
                }
            }
            // ====================== 【脱离节点也参与移动】 ======================
            else {
                if (node.is_leaving) {
                    // 慢速移动到脱离目标
                    node.rel_x += (node.target_x - node.rel_x) * TRANSITION_SPEED * 0.6;
                    node.rel_y += (node.target_y - node.rel_y) * TRANSITION_SPEED * 0.6;
                }
                else {
                    node.rel_x += (node.target_x - node.rel_x) * TRANSITION_SPEED;
                    node.rel_y += (node.target_y - node.rel_y) * TRANSITION_SPEED;
                }
            }
        }
    }

    std::vector<Point2D> UUVFormationSimulator::checkCollision1(const std::vector<Point2D>& positions) {
        std::vector<Point2D> adjusted = positions;
        int iter_num = 0;
        bool has_collision = true;
        int num_uavs = static_cast<int>(adjusted.size());

        while (has_collision && iter_num < MAX_COLLISION_ITER) {
            has_collision = false;
            std::vector<std::tuple<double, int, int>> collision_pairs;

            // 检测碰撞对
            for (int i = 0; i < num_uavs; ++i) {
                for (int j = i + 1; j < num_uavs; ++j) {
                    Point2D diff = adjusted[i] - adjusted[j];
                    double dis = diff.norm();
                    if (dis <= config.collision_radius * 2) {
                        collision_pairs.emplace_back(dis, i, j);
                        has_collision = true;
                    }
                }
            }

            if (!has_collision) {
                break;
            }

            // 按距离升序排序碰撞对
            std::sort(collision_pairs.begin(), collision_pairs.end(),
                [](const std::tuple<double, int, int>& a, const std::tuple<double, int, int>& b) {
                    return std::get<0>(a) < std::get<0>(b);
                });

            // 调整碰撞位置
            for (const auto& pair : collision_pairs) {
                double dis = std::get<0>(pair);
                int i = std::get<1>(pair);
                int j = std::get<2>(pair);

                Point2D diff = adjusted[i] - adjusted[j];
                Point2D dir_vec = diff.normalized();
                double adj = std::min((config.collision_radius * 2 - dis) / 2, MAX_ADJUST_STEP);

                if (i == 0) {
                    adjusted[j] = adjusted[j] - dir_vec * adj;
                }
                else if (j == 0) {
                    adjusted[i] = adjusted[i] + dir_vec * adj;
                }
                else {
                    adjusted[i] = adjusted[i] + dir_vec * adj;
                    adjusted[j] = adjusted[j] - dir_vec * adj;
                }
            }

            iter_num++;
        }

        return adjusted;
    }

    void UUVFormationSimulator::apply_collision_avoidance() {
        if (nodes.size() <= 1) {
            return;
        }

        std::vector<Point2D> ps;
        for (const auto& node : nodes) {
            ps.emplace_back(node.rel_x, node.rel_y);
        }

        std::vector<Point2D> adj = checkCollision1(ps);

        // 主节点位置固定为(0,0)
        nodes[0].rel_x = 0.0;
        nodes[0].rel_y = 0.0;
        for (size_t i = 1; i < nodes.size(); ++i) {
            nodes[i].rel_x = adj[i].x;
            nodes[i].rel_y = adj[i].y;
        }
    }

    std::vector<std::pair<double, double>> UUVFormationSimulator::_generate_formation_positions(int cnt) {
        std::vector<std::pair<double, double>> positions;
        if (cnt < 1 || cnt > 9) {
            throw std::invalid_argument("从节点数量必须在1~9之间，当前值：" + std::to_string(cnt));
        }

        double d = config.rel_distance;  //需要设置
        Formation_Type f = config.trans_formation;

        if (f == Formation_Type::Line) {
            for (int i = 1; i <= cnt; ++i) {
                positions.emplace_back(0.0, -i * d);
            }
        }
        else if (f == Formation_Type::Rectangle) {
            if (cnt < 3) {
                throw std::invalid_argument("矩形队形，从节点数量必须在3~9之间，当前值：" + std::to_string(cnt));
            }
            int num = cnt + 1;
            if (num == 4) {
                positions = { {d, 0}, {d, -d}, {0, -d} };
            }
            else if (num == 5) {
                positions = { {-d, d}, {-d, -d}, {d, d}, {d, -d} };
            }
            else if (num == 6) {
                positions = { {-d, 0}, {d, 0}, {-d, -d}, {0, -d}, {d, -d} };
            }
            else if (num == 7) {
                positions = { {-d, 0}, {d, 0}, {0, -d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d} };
            }
            else if (num == 8) {
                positions = { {-d, 0}, {d, 0}, {-d, -d}, {d, -d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d} };
            }
            else if (num == 9) {
                positions = { {-d, 0}, {d, 0}, {-d, -d}, {0, -d}, {d, -d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d} };
            }
            else if (num == 10) {
                positions = { {-2 * d,0}, {-d,0}, {d,0}, {2 * d,0}, {-2 * d,-d}, {-d,-d}, {0,-d}, {d,-d}, {2 * d,-d} };
            }
        }
        else if (f == Formation_Type::Circle) {
            double r = d * 2;
            for (int i = 0; i < cnt; ++i) {
                double angle = 2 * M_PI * i / cnt;
                positions.emplace_back(r * std::sin(angle), r * std::cos(angle));
            }
        }
        else if (f == Formation_Type::Diamond) {
            if (cnt < 3) {
                throw std::invalid_argument("菱形队形，从节点数量必须在3~9之间，当前值：" + std::to_string(cnt));
            }
            int num = cnt + 1;
            if (num == 4) {
                positions = { {-d, 0}, {0, -d}, {d, 0} };
            }
            else if (num == 5) {
                positions = { {0, d}, {-d, 0}, {d, 0}, {0, -d} };
            }
            else if (num == 6) {
                positions = { {0, d}, {-d, 0}, {d, 0}, {0, -d}, {0, -2*d} };
            }
            else if (num == 7) {
                positions = { {0, -d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d}, {0, -3 * d}, {0, -4 * d} };
            }
            else if (num == 8) {
                positions = { {-d / 2, -d}, {d / 2, -d}, {-d, -2 * d}, {d, -2 * d}, {-d / 2, -3 * d}, {d / 2, -3 * d}, {0, -4 * d} };
            }
            else if (num == 9) {
                positions = { {-d / 2, -d}, {d / 2, -d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d}, {-d / 2, -3 * d}, {d / 2, -3 * d}, {0, -4 * d} };
            }
            else if (num == 10) {
                positions = { {-d / 2, -d}, {d / 2, -d}, {-1.5 * d, -2 * d}, {-0.5 * d, -2 * d}, {0.5 * d, -2 * d}, {1.5 * d, -2 * d}, {-d / 2, -3 * d}, {d / 2, -3 * d}, {0, -4 * d} };
            }
        }
        else if (f == Formation_Type::Triangle) {
            if (cnt < 2) {
                throw std::invalid_argument("三角形队形，从节点数量必须在2~9之间，当前值：" + std::to_string(cnt));
            }
            int num = cnt + 1;
            if (num == 3) {
                positions = { {-d, -d}, {d, -d} };
            }
            else if (num == 4) {
                positions = { {0, d}, {-d, -d}, {d, -d} };
            }
            else if (num == 5) {
                positions = { {-d / 2, -d}, {d / 2, -d}, {-d, -2 * d}, {d, -2 * d} };
            }
            else if (num == 6) {
                positions = { {-d / 2, -d}, {d / 2, -d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d} };
            }
            else if (num == 7) {
                positions = { {-d / 2, -d}, {d / 2, -d}, {-1.5 * d, -2 * d}, {-0.5 * d, -2 * d}, {0.5 * d, -2 * d}, {1.5 * d, -2 * d} };
            }
            else if (num == 8) {
                positions = { {-d, -d}, {0, -d}, {d, -d}, {-1.5 * d, -2 * d}, {-0.5 * d, -2 * d}, {0.5 * d, -2 * d}, {1.5 * d, -2 * d} };
            }
            else if (num == 9) {
                positions = { {-d, -d}, {0, -d}, {d, -d}, {-2 * d, -2 * d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d}, {2 * d, -2 * d} };
            }
            else if (num == 10) {
                positions = { {-d / 2, -d}, {d / 2, -d}, {-d, -2 * d}, {0, -2 * d}, {d, -2 * d}, {-1.5 * d, -3 * d}, {-0.5 * d, -3 * d}, {0.5 * d, -3 * d}, {1.5 * d, -3 * d} };
            }
        }
        else if (f == Formation_Type::Custom) {
            int num = cnt + 1;
            // 从全局上下文获取自定义队形点位
            if(custom_data.find(config.custom_id) != custom_data.end())
            {
                if(custom_data.find(config.custom_id)->second.node_num == num)//初始化自定义队形使用
                {
                    vector<Point2D>& temp_positions = custom_data.find(config.custom_id)->second.node_rel_positions;//包含主节点
                    for(size_t i = 1; i < temp_positions.size(); i++)
                    {
                        //printf("自定义队形目标点 %d: (%.1f, %.1f)\n", i+1, positions[i].first, positions[i].second);
                        positions.emplace_back(temp_positions[i].x, temp_positions[i].y);
                    }
                }
                else//节点数不匹配(增加节点或者删除节点)
                {
                    for(size_t i = 1; i < nodes.size(); i++)
                    {
                        //printf("自定义队形目标点 %d: (%.1f, %.1f)\n", i+1, positions[i].first, positions[i].second);
                        positions.emplace_back(nodes[i].custom_rel_x, nodes[i].custom_rel_y);
                    }
                }
            }
        }

        config.current_formation = config.trans_formation;
        return positions;
    }

    std::pair<double, double> UUVFormationSimulator::_geo2enu(double lon, double lat, double rlon, double rlat) {
        double lr = to_radians(lon);
        double la = to_radians(lat);
        double rlr = to_radians(rlon);
        double rla = to_radians(rlat);

        double x = R_EARTH * (lr - rlr) * std::cos(rla);
        double y = R_EARTH * (la - rla);

        return { x, y };
    }

    std::pair<double, double> UUVFormationSimulator::_enu2geo(double x, double y, double rlon, double rlat) {
        double rlr = to_radians(rlon);
        double rla = to_radians(rlat);

        double dlon = x / (R_EARTH * std::cos(rla));
        double dlat = y / R_EARTH;

        double lon = to_degrees(rlr + dlon);
        double lat = to_degrees(rla + dlat);

        return { lon, lat };
    }

    // ====================== 【修改】更新机动（处理脱离节点） ======================
    void UUVFormationSimulator::_update_maneuver() {
        UUVNode& main = nodes[0];
        double dt = config.sim_step;

        double current_v_main = config.init_speed + config.acceleration * current_time;
        current_v_main = std::max(0.1, current_v_main);

        main.heading = fmod(main.heading + config.heading_rate * dt, 360.0);
        if (main.heading < 0) main.heading += 360.0;
        double main_hdg_rad = to_radians(main.heading);
        main.speed = current_v_main;

        double dx_main = current_v_main * std::sin(main_hdg_rad) * dt;
        double dy_main = current_v_main * std::cos(main_hdg_rad) * dt;
        auto [new_lon, new_lat] = _enu2geo(dx_main, dy_main, main.pos_.lon_deg, main.pos_.lat_deg);
        main.pos_.lon_deg = new_lon;
        main.pos_.lat_deg = new_lat;

        if (is_transition) {
            _transition_formation();
        }
        apply_collision_avoidance();
        _formation_keeping();  // 队形保持：避碰后持续拉回目标位置
        double w = to_radians(config.heading_rate);

        std::vector<size_t> indices_to_remove;

        // ====================== 逆序遍历（和Python一致） ======================
        for (int i = nodes.size() - 1; i >= 1; --i) {
            UUVNode& node = nodes[i];

            if (node.is_leaving) {
                // ====================== 找最近节点 ======================
                UUVNode* closest_node = nullptr;
                double min_dist = 1e9;

                for (size_t j = 1; j < nodes.size(); ++j) {
                    UUVNode& n = nodes[j];
                    if (n.is_leaving) continue;

                    double dx = n.rel_x - node.rel_x;
                    double dy = n.rel_y - node.rel_y;
                    double d = std::hypot(dx, dy);
                    if (d < min_dist) {
                        min_dist = d;
                        closest_node = &n;
                    }
                }

                // ====================== 距离超过 5倍间隔 → 删除 ======================
                if (closest_node != nullptr) {
                    double current_dist = std::hypot(closest_node->rel_x - node.rel_x,
                        closest_node->rel_y - node.rel_y);
                    double delete_dist = 5 * config.rel_distance;

                    printf("脱离节点ID:%d → 最近节点:%d | 距离:%.1f / 阈值:%.1f\n",
                        node.id, closest_node->id, current_dist, delete_dist);

                    if (current_dist > delete_dist) {
                        printf("✅ 节点 ID:%d 已远离编队，消失\n", node.id);
                        indices_to_remove.push_back(i);
                    }
                }

                // ====================== 经纬度更新（和普通节点完全一样） ======================
                double rx = node.rel_x;
                double ry = node.rel_y;
                double wx = rx * cos(main_hdg_rad) - ry * sin(main_hdg_rad);
                double wy = rx * sin(main_hdg_rad) + ry * cos(main_hdg_rad);
                auto [lon, lat] = _enu2geo(wx, wy, main.pos_.lon_deg, main.pos_.lat_deg);
                node.pos_.lon_deg = lon;
                node.pos_.lat_deg = lat;

                continue;
            }

            // ====================== 普通节点 ======================
            double rx = node.rel_x;
            double ry = node.rel_y;
            double des_vx, des_vy;

            if (std::fabs(w) < 1e-4) {
                des_vx = current_v_main * std::sin(main_hdg_rad);
                des_vy = current_v_main * std::cos(main_hdg_rad);
            }
            else {
                double v_rel_x = -w * ry;
                double v_rel_y = w * rx;
                des_vx = current_v_main * std::sin(main_hdg_rad) + v_rel_x;
                des_vy = current_v_main * std::cos(main_hdg_rad) + v_rel_y;
            }

            double desired_speed = std::hypot(des_vx, des_vy);
            desired_speed = std::min(desired_speed, MAX_SPEED);
            node.speed = desired_speed;

            double hdg = std::atan2(des_vx, des_vy);
            hdg = to_degrees(hdg);
            hdg = fmod(hdg, 360.0);
            if (hdg < 0) hdg += 360.0;
            node.heading = hdg;

            double wx = rx * cos(main_hdg_rad) - ry * sin(main_hdg_rad);
            double wy = rx * sin(main_hdg_rad) + ry * cos(main_hdg_rad);
            auto [lon, lat] = _enu2geo(wx, wy, main.pos_.lon_deg, main.pos_.lat_deg);
            node.pos_.lon_deg = lon;
            node.pos_.lat_deg = lat;
        }

        // 批量删除
        if (!indices_to_remove.empty()) {
            std::sort(indices_to_remove.rbegin(), indices_to_remove.rend());
            for (size_t idx : indices_to_remove) {
                nodes.erase(nodes.begin() + idx);
            }
            config.node_num = nodes.size();
            _set_target_formation();
        }
    }

    // ====================== 队形保持 ======================
    void UUVFormationSimulator::_formation_keeping() {
        // 将因避碰偏离的节点拉回目标相对位置
        // 关键约束：每步最大修正量硬限制为 0.5m，确保碰撞避免始终优先
        for (size_t i = 1; i < nodes.size(); ++i) {
            UUVNode& node = nodes[i];
            if (node.is_leaving || node.is_joining) {
                continue;
            }
            double dx = node.target_x - node.rel_x;
            double dy = node.target_y - node.rel_y;
            double dist = std::hypot(dx, dy);
            if (dist < 0.03) {
                continue;  // 已在目标位置，无需调整
            }

            // 自适应回位速度（基于偏离距离），硬限制每步修正量
            double speed;
            if (dist < 1.5) {
                speed = 0.10;       // 微调：慢速收敛
            }
            else if (dist < 5.0) {
                speed = 0.10 + (dist - 1.5) * 0.02;   // 0.10 → 0.17
            }
            else if (dist < 10.0) {
                speed = 0.17 + (dist - 5.0) * 0.006;  // 0.17 → 0.20
            }
            else {
                speed = 0.20;
            }

            double correction = dist * speed;
            if (correction > FK_MAX_STEP) {
                speed = FK_MAX_STEP / dist;  // 缩放速度以满足硬限制
            }

            node.rel_x += dx * speed;
            node.rel_y += dy * speed;
        }
    }

    void UUVFormationSimulator::_set_custom_id(int custom_id)
    {
        config.custom_id = custom_id;
    }

    // ====================== 编队稳定判断 ======================
    void UUVFormationSimulator::_record_transition_step() {
        // 没有在过渡，直接返回
        if (!is_transition) {
            return;
        }

        // 如果有节点正在加入编队 → 不判断稳定
        bool any_joining = false;
        for (const auto& n : nodes) {
            if (n.is_joining) {
                any_joining = true;
                break;
            }
        }
        if (any_joining) {
            return;
        }

        // 计算最大队形误差（跳过：主节点、脱离节点、加入节点）
        double max_err = 0.0;
        for (const auto& n : nodes) {
            if (n.id == 0 || n.is_leaving || n.is_joining) {
                continue;
            }
            double err = std::hypot(n.rel_x - n.target_x, n.rel_y - n.target_y);
            if (err > max_err) {
                max_err = err;
            }
        }

        // 误差小于阈值 → 视为稳定
        if (max_err < ERROR_STABLE_THRESHOLD) {
            // 如果还有节点在脱离 → 不结束过渡
            bool has_leaving = false;
            for (const auto& n : nodes) {
                if (n.is_leaving) {
                    has_leaving = true;
                    break;
                }
            }

            if (!has_leaving) {
                is_transition = false;
                printf("✅ 编队已稳定\n");
            }
        }
    }

    // ====================== 【新增】添加节点 ======================
    void UUVFormationSimulator::add_node(vector<UUVNode>& input) {
        std::lock_guard<std::mutex> lock(sim_mutex);
        if (nodes.size() >= 10) {
            printf("❌ 已达到最大节点数10个，无法添加\n");
            return;
        }

        if (nodes.size() + input.size() > 10) {
            printf("❌ 超过最大节点数10个，添加数量过多\n");
            return;
        }

        
        UUVNode& main = nodes[0];

        for (int node_index = 0; node_index < input.size(); node_index++) {

            max_id += 1;
            // 计算新节点相对于主节点的初始位置
            auto [rel_x, rel_y] = _geo2enu(input[node_index].pos_.lon_deg, input[node_index].pos_.lat_deg, main.pos_.lon_deg, main.pos_.lat_deg);

            // 创建新节点
            UUVNode new_node;
            new_node.id = max_id;
            new_node.pos_.lon_deg = input[node_index].pos_.lon_deg;
            new_node.pos_.lat_deg = input[node_index].pos_.lat_deg;
            new_node.speed = input[node_index].speed;
            new_node.heading = input[node_index].heading;
            new_node.rel_x = rel_x;
            new_node.rel_y = rel_y;
            new_node.last_rel_x = rel_x;
            new_node.last_rel_y = rel_y;
            new_node.custom_rel_x = input[node_index].custom_rel_x;
            new_node.custom_rel_y = input[node_index].custom_rel_y;
            new_node.is_joining = true;
            new_node.join_progress = 0.0;
            new_node.join_total_frames = input[node_index].join_total_frames;
            new_node.is_leaving = false;

            nodes.push_back(new_node);

            printf("\n✅ 成功添加节点 ID:%d\n", max_id);
        }
        
        config.node_num = static_cast<int>(nodes.size());

        _set_target_formation();
        is_transition = true;
        
        printf("   所有节点正在重排为 %zu 节点队形...\n", nodes.size());
    }

    // ====================== 删除末尾节点(改变脱离方向) ======================
    void UUVFormationSimulator::remove_last_node(int num) {
        std::lock_guard<std::mutex> lock(sim_mutex);
        if (nodes.size() <= 2) {
            printf("❌ 节点数过少，无法删除\n");
            return;
        }

        if ((nodes.size() - num) < 2) {
            printf("❌ 移除节点数量过多，请减少!\n");
            return;
        }

        UUVNode& main = nodes[0];
        UUVNode* leave_node = nullptr;
        while (num--)
        {
            leave_node = nullptr;
            for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
                if (it->id != 0 && !it->is_leaving) {
                    leave_node = &(*it);
                    break;
                }
            }

            if (!leave_node) {
                printf("❌ 没有可删除的节点\n");
                return;
            }

            leave_node->is_leaving = true;

            // ====================== 【修改为Python逻辑】 ======================
            double opposite_heading = fmod(main.heading + 90.0, 360.0);  //  改成 +90°
            if (opposite_heading < 0) opposite_heading += 360.0;
            double opposite_rad = to_radians(opposite_heading);

            double leave_dist = config.rel_distance * 15.0;  //  15倍距离
            leave_node->leave_target_x = leave_dist * std::sin(opposite_rad);
            leave_node->leave_target_y = leave_dist * std::cos(opposite_rad);
        }

        _set_target_formation();
        is_transition = true;

        printf("\n✅ 节点 ID:%d 开始脱离\n", leave_node->id);
        printf("   脱离目标相对位置: (%.1f, %.1f)\n", leave_node->leave_target_x, leave_node->leave_target_y);
    }

    UAVTrajectory& UUVFormationSimulator::step_simulation() {
        std::lock_guard<std::mutex> lock(sim_mutex);
        trajectory_.clearAllTrajectory();
        for (double cnt = 0; cnt < (config.return_frames / 10); cnt += config.sim_step) {
            current_time += config.sim_step;
            _update_maneuver();
            _record_transition_step();
            trajectory_.addFrame((current_time * 10), config.current_formation, nodes);
        }
        return trajectory_;
    }

    void UUVFormationSimulator::step_single_frame() {
        std::lock_guard<std::mutex> lock(sim_mutex);
        current_time += config.sim_step;
        _update_maneuver();
        _record_transition_step();
    }

    void UUVFormationSimulator::record_current_frame() {
        std::lock_guard<std::mutex> lock(sim_mutex);
        trajectory_.addFrame(static_cast<int>(current_time * 10),
                             config.current_formation, nodes);
    }

    void UUVFormationSimulator::clear_trajectory() {
        std::lock_guard<std::mutex> lock(sim_mutex);
        trajectory_.clearAllTrajectory();
    }

    UAVTrajectory& UUVFormationSimulator::getUAVtrajectory()
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        return trajectory_;
    }

    double UUVFormationSimulator::_calculate_formation_error(const UUVNode& node) {
        if (node.id == 0 || node.is_leaving || node.is_joining) {
            return 0.0;
        }
        return std::hypot(node.rel_x - node.target_x, node.rel_y - node.target_y);
    }

    void UUVFormationSimulator::InitialParams(FormationConfig& forparams_)
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        config = forparams_;
    }

    void UUVFormationSimulator::apply_follower_offset(size_t node_index, double delta_rel_x, double delta_rel_y)
    {
        if (node_index < nodes.size()) {
            nodes[node_index].rel_x += delta_rel_x;
            nodes[node_index].rel_y += delta_rel_y;
        }
    }

    void UUVFormationSimulator::apply_leader_offset(double delta_lon, double delta_lat)
    {
        if (!nodes.empty()) {
            nodes[0].pos_.lon_deg += delta_lon;
            nodes[0].pos_.lat_deg += delta_lat;
            config.main_node.lon_deg = nodes[0].pos_.lon_deg;
            config.main_node.lat_deg = nodes[0].pos_.lat_deg;
        }
    }

    double UUVFormationSimulator::get_main_heading_rad() const
    {
        if (!nodes.empty()) {
            return to_radians(nodes[0].heading);
        }
        return 0.0;
    }

    // ====================== 外部接口实现 ======================

    // 清理所有编队仿真器
    void Cleanup_All_Formations() {
        for (auto& pair : g_FormationSimulators) {
            if (pair.second != nullptr) {
                delete pair.second;
                pair.second = nullptr;
            }
        }
        g_FormationSimulators.clear();
    }

    // ====================== 跨编队全局碰撞避免 ======================
    void ApplyInterFormationAvoidance() {
        if (g_FormationSimulators.size() <= 1) return;

        // 取第一个编队的主节点为全局 ENU 参考原点
        auto first_it = g_FormationSimulators.begin();
        UUVFormationSimulator* ref_sim = first_it->second;
        if (ref_sim == nullptr) return;

        const auto& ref_nodes = ref_sim->get_nodes();
        if (ref_nodes.empty()) return;
        double ref_lon = ref_nodes[0].pos_.lon_deg;
        double ref_lat = ref_nodes[0].pos_.lat_deg;

        // 收集所有节点的全局 ENU 位置
        struct NodeEntry {
            UUVFormationSimulator* sim;
            size_t node_index;
            bool is_leader;
        };
        std::vector<NodeEntry> entries;
        std::vector<Point2D> positions;

        for (auto& pair : g_FormationSimulators) {
            UUVFormationSimulator* sim = pair.second;
            if (sim == nullptr) continue;
            const auto& nodes = sim->get_nodes();
            for (size_t i = 0; i < nodes.size(); ++i) {
                const UUVNode& node = nodes[i];
                double lr = to_radians(node.pos_.lon_deg);
                double la = to_radians(node.pos_.lat_deg);
                double rlr = to_radians(ref_lon);
                double rla = to_radians(ref_lat);
                double gx = R_EARTH * (lr - rlr) * std::cos(rla);
                double gy = R_EARTH * (la - rla);

                entries.push_back({sim, i, (i == 0)});
                positions.emplace_back(gx, gy);
            }
        }

        // 全局碰撞分离（使用更大的跨编队有效半径）
        double inter_radius = COLLISION_RADIUS * 2.0 + INTER_FORMATION_BUFFER;
        std::vector<Point2D> adjusted = positions;
        int iter = 0;
        bool has_collision = true;

        while (has_collision && iter < MAX_COLLISION_ITER) {
            has_collision = false;
            std::vector<std::tuple<double, int, int>> pairs;

            for (size_t i = 0; i < adjusted.size(); ++i) {
                for (size_t j = i + 1; j < adjusted.size(); ++j) {
                    if (entries[i].sim == entries[j].sim) continue;  // 跳过同编队
                    double d = (adjusted[i] - adjusted[j]).norm();
                    if (d < inter_radius) {
                        pairs.emplace_back(d, (int)i, (int)j);
                        has_collision = true;
                    }
                }
            }
            if (!has_collision) break;

            std::sort(pairs.begin(), pairs.end(),
                [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });

            for (const auto& pr : pairs) {
                double d = std::get<0>(pr);
                int a = std::get<1>(pr);
                int b = std::get<2>(pr);
                Point2D dir = (adjusted[a] - adjusted[b]).normalized();
                double step = std::min((inter_radius - d) / 2.0, MAX_ADJUST_STEP);
                adjusted[a] = adjusted[a] + dir * step;
                adjusted[b] = adjusted[b] - dir * step;
            }
            iter++;
        }

        // 将调整量写回各节点
        std::set<UUVFormationSimulator*> modified_sims;
        for (size_t k = 0; k < entries.size(); ++k) {
            double delta_x = adjusted[k].x - positions[k].x;
            double delta_y = adjusted[k].y - positions[k].y;
            if (std::fabs(delta_x) < 1e-8 && std::fabs(delta_y) < 1e-8) continue;

            UUVFormationSimulator* sim = entries[k].sim;
            size_t idx = entries[k].node_index;

            if (entries[k].is_leader) {
                // 领航节点：将调整后的 ENU 位置转回 lon/lat
                double rlr = to_radians(ref_lon);
                double rla = to_radians(ref_lat);
                double new_lon = to_degrees(rlr + adjusted[k].x / (R_EARTH * std::cos(rla)));
                double new_lat = to_degrees(rla + adjusted[k].y / R_EARTH);
                double delta_lon = new_lon - sim->get_nodes()[0].pos_.lon_deg;
                double delta_lat = new_lat - sim->get_nodes()[0].pos_.lat_deg;
                sim->apply_leader_offset(delta_lon, delta_lat);
            }
            else {
                // 跟随节点：用逆旋转矩阵将全局 ENU 调整量转为 rel 调整量
                double hdg_rad = sim->get_main_heading_rad();
                double cos_h = std::cos(hdg_rad);
                double sin_h = std::sin(hdg_rad);
                double delta_rel_x =  cos_h * delta_x + sin_h * delta_y;
                double delta_rel_y = -sin_h * delta_x + cos_h * delta_y;
                sim->apply_follower_offset(idx, delta_rel_x, delta_rel_y);
            }
            modified_sims.insert(sim);
        }

        // 重跑编队内避碰，修复因跨编队调整可能引入的编队内碰撞
        for (auto* sim : modified_sims) {
            sim->reapply_collision_avoidance();
        }
    }

    // 初始化单个编队（兼容旧接口，使用 config 中的 formation_id，默认为 0）
    //void Init_formation(const FormationConfig& config, Json::Value& trajectory_result) {
    //    int fid = config.formation_id;

    //    // 如果该 ID 已存在，先清理旧实例
    //    auto it = g_FormationSimulators.find(fid);
    //    if (it != g_FormationSimulators.end() && it->second != nullptr) {
    //        delete it->second;
    //        it->second = nullptr;
    //    }

    //    try {
    //        UUVFormationSimulator* sim = new UUVFormationSimulator(config);
    //        g_FormationSimulators[fid] = sim;
    //        printf("编队 [ID:%d] 初始化成功！\n", fid);

    //        UAVTrajectory& trajectory_data = sim->getUAVtrajectory();

    //        auto all_trajectory = trajectory_data.getAllTrajectory();
    //        if (all_trajectory.empty()) return;

    //        trajectory_result["frames"] = Json::Value(Json::arrayValue);
    //        trajectory_result["formation_id"] = fid;

    //        for (const auto& group : all_trajectory) {
    //            int frame_id = group.frame;
    //            const auto& frame_nodes = group.nodes_;

    //            Json::Value frame_obj;
    //            frame_obj["frame_id"] = frame_id;
    //            frame_obj["formation_type"] = static_cast<int>(group.formation);

    //            Json::Value nodes(Json::arrayValue);
    //            double max_error = 0.0;
    //            for (const auto& node : frame_nodes) {
    //                double err = sim->_calculate_formation_error(node);
    //                max_error = std::max(max_error, err);

    //                Json::Value node_json;
    //                node_json["node_id"] = node.id;
    //                node_json["lon"] = std::round(node.pos_.lon_deg * 1e6) / 1e6;
    //                node_json["lat"] = std::round(node.pos_.lat_deg * 1e6) / 1e6;
    //                node_json["speed"] = std::round(node.speed * 1e3) / 1e3;
    //                node_json["heading"] = std::round(node.heading * 1e3) / 1e3;
    //                node_json["rel_x"] = std::round(node.rel_x * 1e3) / 1e3;
    //                node_json["rel_y"] = std::round(node.rel_y * 1e3) / 1e3;
    //                node_json["target_x"] = std::round(node.target_x * 1e3) / 1e3;
    //                node_json["target_y"] = std::round(node.target_y * 1e3) / 1e3;
    //                node_json["formation_error"] = std::round(err * 1e4) / 1e4;

    //                nodes.append(node_json);
    //            }

    //            frame_obj["nodes"] = nodes;
    //            trajectory_result["frames"].append(frame_obj);
    //        }
    //    }
    //    catch (const std::exception& e) {
    //        printf("编队 [ID:%d] 初始化失败：%s\n", fid, e.what());
    //        g_FormationSimulators.erase(fid);
    //    }
    //}

    // 初始化多个编队
    void Init_Multi_Formation(const MultiFormationContext& context, Json::Value& result) {
        // 先清理所有旧实例
        Cleanup_All_Formations();

        result["formations"] = Json::Value(Json::arrayValue);

        CustomFormationList custom_data = context.custom_formations;  // 传递整个自定义编队列表，供每个编队实例查询
        auto data = context.formations;
        for (auto iter = data.begin(); iter != data.end(); iter++) {
            int fid = iter->first;

            const FormationConfig& cfg = iter->second;

            try {
                UUVFormationSimulator* sim = new UUVFormationSimulator(cfg, custom_data);

                g_FormationSimulators[fid] = sim;
                printf("多编队初始化：编队 [ID:%d] 成功，节点数=%d\n", fid, cfg.node_num);

                // 为每个编队生成初始轨迹摘要
                Json::Value form_entry;
                form_entry["formation_id"] = fid;
                form_entry["custom_id"] = cfg.custom_id;
                form_entry["node_num"] = cfg.node_num;
                form_entry["current_formation"] = static_cast<int>(cfg.current_formation);
                form_entry["main_lon"] = cfg.main_node.lon_deg;
                form_entry["main_lat"] = cfg.main_node.lat_deg;
                result["formations"].append(form_entry);
            }
            catch (const std::exception& e) {
                printf("多编队初始化：编队 [ID:%d] 失败：%s\n", fid, e.what());
            }
        }
        printf("多编队初始化完成，共 %zu 个编队\n", g_FormationSimulators.size());
    }

    bool SwitchFormation(int formation_id, int custom_id, const Formation_Type& type) {
        UUVFormationSimulator* sim = GetFormationSimulator(formation_id);
        if (sim == nullptr) {
            printf("仿真器 [ID:%d] 未初始化！\n", formation_id);
            return false;
        }
        sim->_set_custom_id(custom_id);  // 更新当前编队的 custom_id，供切换到自定义队形时查询使用
        bool flag = sim->switch_formation(type);
        return flag;
    }

    void TurnFormation(int formation_id, double heading_rate) {
        UUVFormationSimulator* sim = GetFormationSimulator(formation_id);
        if (sim == nullptr) {
            printf("仿真器 [ID:%d] 未初始化！\n", formation_id);
            return;
        }
        sim->set_heading_rate(heading_rate);
    }

    void AddNode(int formation_id, vector<UUVNode>& input) {
        UUVFormationSimulator* sim = GetFormationSimulator(formation_id);
        if (sim == nullptr) {
            printf("仿真器 [ID:%d] 未初始化！\n", formation_id);
            return;
        }
        sim->add_node(input);
    }

    // ====================== 【新增】外部接口：删除节点 ======================
    void RemoveLastNode(int formation_id, int num) {
        UUVFormationSimulator* sim = GetFormationSimulator(formation_id);
        if (sim == nullptr) {
            printf("仿真器 [ID:%d] 未初始化！\n", formation_id);
            return;
        }
        sim->remove_last_node(num);
    }

    // 单编队兼容接口：遍历所有编队，合并输出结果
    /*void Transformation_Use(CalcTempParam& task_param) {
        if (g_FormationSimulators.empty()) {
            printf("无可用仿真器！\n");
            return;
        }

        task_param.trajectory_result.clear();
        Json::Value formations_obj(Json::objectValue);

        for (auto& pair : g_FormationSimulators) {
            int fid = pair.first;
            UUVFormationSimulator* sim = pair.second;
            if (sim == nullptr) continue;

            UAVTrajectory trajectory_data = sim->step_simulation();
            auto all_trajectory = trajectory_data.getAllTrajectory();

            task_param.run_frames = sim->getRunframe() * 10;

            Json::Value frames_array(Json::arrayValue);

            for (const auto& group : all_trajectory) {
                int frame_id = group.frame;
                const auto& frame_nodes = group.nodes_;

                Json::Value frame_obj;
                frame_obj["frame_id"] = frame_id;
                frame_obj["formation_type"] = static_cast<int>(group.formation);

                Json::Value nodes_array(Json::arrayValue);

                for (const auto& node : frame_nodes) {
                    double err = sim->_calculate_formation_error(node);

                    Json::Value node_json;
                    node_json["node_id"] = node.id;
                    node_json["lon"] = std::round(node.pos_.lon_deg * 1e6) / 1e6;
                    node_json["lat"] = std::round(node.pos_.lat_deg * 1e6) / 1e6;
                    node_json["speed"] = std::round(node.speed * 1e3) / 1e3;
                    node_json["heading"] = std::round(node.heading * 1e3) / 1e3;
                    node_json["rel_x"] = std::round(node.rel_x * 1e3) / 1e3;
                    node_json["rel_y"] = std::round(node.rel_y * 1e3) / 1e3;
                    node_json["target_x"] = std::round(node.target_x * 1e3) / 1e3;
                    node_json["target_y"] = std::round(node.target_y * 1e3) / 1e3;
                    node_json["formation_error"] = std::round(err * 1e4) / 1e4;

                    nodes_array.append(node_json);
                }

                frame_obj["nodes"] = nodes_array;
                frames_array.append(frame_obj);
            }

            formations_obj[std::to_string(fid)] = frames_array;
        }

        task_param.trajectory_result["formations"] = formations_obj;
    }*/

    void Transformation_Use(CalcTempParam& task_param) {
        if (g_FormationSimulators.empty()) {
            printf("无可用仿真器！\n");
            return;
        }

        task_param.trajectory_result.clear();
        Json::Value formations_obj(Json::objectValue);
        std::mutex json_mutex;

        using SimPair = std::pair<int, UUVFormationSimulator*>;
        std::vector<SimPair> sim_list(g_FormationSimulators.begin(), g_FormationSimulators.end());
        size_t num_formations = sim_list.size();

        // 从第一个编队获取批次仿真参数
        UUVFormationSimulator* first_sim = sim_list[0].second;
        if (first_sim == nullptr) return;
        double sim_step = first_sim->get_config().sim_step;
        double total_batch_time = first_sim->get_config().return_frames / 10.0;

        // ======================
        // Phase 0：并行清空所有编队的轨迹
        // ======================
        concurrency::parallel_for((size_t)0, num_formations, [&](size_t index)
            {
                UUVFormationSimulator* sim = sim_list[index].second;
                if (sim) sim->clear_trajectory();
            });

        // ======================
        // Phase 1：逐帧循环 —— 每帧执行：步进 → 跨编队避碰 → 记录轨迹
        // ======================
        for (double t = 0.0; t < total_batch_time; t += sim_step) {
            // 1a: 并行步进 —— 所有编队各自前进一帧
            concurrency::parallel_for((size_t)0, num_formations, [&](size_t index)
                {
                    UUVFormationSimulator* sim = sim_list[index].second;
                    if (sim) sim->step_single_frame();
                });

            // 1b: 串行避碰 —— 跨编队全局碰撞避免（每帧都执行）
            if (num_formations > 1) {
                ApplyInterFormationAvoidance();
            }

            // 1c: 并行记录 —— 将避碰后的节点状态写入轨迹
            concurrency::parallel_for((size_t)0, num_formations, [&](size_t index)
                {
                    UUVFormationSimulator* sim = sim_list[index].second;
                    if (sim) sim->record_current_frame();
                });
        }

        // ======================
        // Phase 2：并行组装 JSON 输出
        // ======================
        concurrency::parallel_for((size_t)0, num_formations, [&](size_t index)
            {
                auto& pair = sim_list[index];
                int fid = pair.first;
                UUVFormationSimulator* sim = pair.second;
                if (sim == nullptr) return;

                UAVTrajectory& trajectory_data = sim->getUAVtrajectory();
                auto all_trajectory = trajectory_data.getAllTrajectory();
                Json::Value frames_array(Json::arrayValue);

                for (const auto& group : all_trajectory) {
                    Json::Value frame_obj;
                    frame_obj["frame_id"] = group.frame;
                    frame_obj["formation_type"] = static_cast<int>(group.formation);
                    Json::Value nodes_array(Json::arrayValue);

                    for (const auto& node : group.nodes_) {
                        double err = sim->_calculate_formation_error(node);
                        Json::Value node_json;
                        node_json["node_id"] = node.id;
                        node_json["lon"] = std::round(node.pos_.lon_deg * 1e6) / 1e6;
                        node_json["lat"] = std::round(node.pos_.lat_deg * 1e6) / 1e6;
                        node_json["speed"] = std::round(node.speed * 1e3) / 1e3;
                        node_json["heading"] = std::round(node.heading * 1e3) / 1e3;
                        node_json["rel_x"] = std::round(node.rel_x * 1e3) / 1e3;
                        node_json["rel_y"] = std::round(node.rel_y * 1e3) / 1e3;
                        node_json["target_x"] = std::round(node.target_x * 1e3) / 1e3;
                        node_json["target_y"] = std::round(node.target_y * 1e3) / 1e3;
                        node_json["formation_error"] = std::round(err * 1e4) / 1e4;
                        nodes_array.append(node_json);
                    }
                    frame_obj["nodes"] = nodes_array;
                    frames_array.append(frame_obj);
                }

                std::lock_guard<std::mutex> lock(json_mutex);
                formations_obj[std::to_string(fid)] = frames_array;
            });

        // 设置运行帧数
        if (!sim_list.empty()) {
            task_param.run_frames = sim_list[0].second->getRunframe() * 10;
        }

        task_param.trajectory_result["formations"] = formations_obj;
        printf("多编队 PPL 并行计算完成！\n");
    }

    // 多编队单帧接口：只执行指定 formation_id 的编队
    void Transformation_Use_Multi(int formation_id, CalcTempParam& task_param) {
        UUVFormationSimulator* sim = GetFormationSimulator(formation_id);
        if (sim == nullptr) {
            printf("仿真器 [ID:%d] 未初始化！\n", formation_id);
            return;
        }

        UAVTrajectory trajectory_data = sim->step_simulation();
        auto all_trajectory = trajectory_data.getAllTrajectory();

        task_param.trajectory_result.clear();
        Json::Value frames_array(Json::arrayValue);

        task_param.run_frames = sim->getRunframe() * 10;

        for (const auto& group : all_trajectory) {
            int frame_id = group.frame;
            const auto& frame_nodes = group.nodes_;

            Json::Value frame_obj;
            frame_obj["frame_id"] = frame_id;
            frame_obj["formation_type"] = static_cast<int>(group.formation);

            Json::Value nodes_array(Json::arrayValue);

            for (const auto& node : frame_nodes) {
                double err = sim->_calculate_formation_error(node);

                Json::Value node_json;
                node_json["node_id"] = node.id;
                node_json["lon"] = std::round(node.pos_.lon_deg * 1e6) / 1e6;
                node_json["lat"] = std::round(node.pos_.lat_deg * 1e6) / 1e6;
                node_json["speed"] = std::round(node.speed * 1e3) / 1e3;
                node_json["heading"] = std::round(node.heading * 1e3) / 1e3;
                node_json["rel_x"] = std::round(node.rel_x * 1e3) / 1e3;
                node_json["rel_y"] = std::round(node.rel_y * 1e3) / 1e3;
                node_json["target_x"] = std::round(node.target_x * 1e3) / 1e3;
                node_json["target_y"] = std::round(node.target_y * 1e3) / 1e3;
                node_json["formation_error"] = std::round(err * 1e4) / 1e4;

                nodes_array.append(node_json);
            }

            frame_obj["nodes"] = nodes_array;
            frames_array.append(frame_obj);
        }

        task_param.trajectory_result["frames"] = frames_array;
        task_param.trajectory_result["formation_id"] = formation_id;
    }

    void UAVTrajectory::addFrame(int frame, const Formation_Type formation, const vector<UUVNode>& nodes)
    {
        trajectory_data.push_back({ frame, formation, nodes });
    }

    void UAVTrajectory::addFormationChangeFrame(int frame) {
        formation_change_frames.push_back(frame);
    }

    void UAVTrajectory::clearAllTrajectory()
    {
        trajectory_data.clear();
    }

    const std::vector<TrajectoryFrame>& UAVTrajectory::getAllTrajectory() const {
        return trajectory_data;
    }

    const std::vector<int>& UAVTrajectory::getFormationChangeFrames() const {
        return formation_change_frames;
    }

}


