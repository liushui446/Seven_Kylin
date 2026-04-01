#include "transformation/transformation.hpp"

#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>

using namespace Json;

namespace seven {

    // 全局仿真器实例
    UUVFormationSimulator* g_pFormationSimulator = nullptr;

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
    UUVFormationSimulator::UUVFormationSimulator(const FormationConfig& cfg)
        : config(cfg), current_time(0.0), last_output_time(0.0),
        is_transition(false), last_formation(cfg.current_formation){
        trajectory_.clearAllTrajectory();
        _validate_config();
        _init_nodes();
        trajectory_.addFrame((current_time * 10), cfg.trans_formation, nodes);
    }

    void UUVFormationSimulator::_validate_config() {
        if (config.node_num < 2 || config.node_num > 10) {
            throw std::invalid_argument("节点数量必须在2~10之间，当前值：" + std::to_string(config.node_num));
        }
        if (config.rel_distance <= 0.0) {
            throw std::invalid_argument("节点间距必须大于0");
        }
    }

    void UUVFormationSimulator::_init_nodes() {
        // 初始化主节点
        UUVNode main_node;
        main_node.id = 0;
        main_node.pos_ = { config.main_node.lon_deg, config.main_node.lat_deg };
        main_node.speed = config.init_speed;
        main_node.heading = config.init_heading;
        nodes.push_back(main_node);

        // 初始化从节点
        for (int i = 1; i < config.node_num; ++i) {
            UUVNode node;
            node.id = i;
            node.pos_ = { config.main_node.lon_deg, config.main_node.lat_deg };
            node.speed = config.init_speed;
            node.heading = config.init_heading;
            nodes.emplace_back(node);
        }

        _set_target_formation();
        _set_initial_position();

        // 初始化last_rel_x/y
        for (auto& node : nodes) {
            node.last_rel_x = node.rel_x;
            node.last_rel_y = node.rel_y;
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

    void UUVFormationSimulator::switch_formation(const Formation_Type& cmd) {
        std::lock_guard<std::mutex> lock(sim_mutex);
        if (cmd == config.current_formation) {
            printf("当前已是该队形\n");
            return;
        }

        last_formation = config.current_formation;
        config.trans_formation = cmd;
        _set_target_formation();
        is_transition = true;
        //transition_data.clear();
        printf("\n✅ 切换队形：%s → %s，开始记录...\n", formationToStr(last_formation), formationToStr(config.trans_formation));
    }

    // ====================== 设置目标队形 【给所有正在脱离的节点单独设置目标点】 ======================
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
            double min_dist = 1e9;

            // 寻找离当前节点最近的目标点
            for (int i = 0; i < available_targets.size(); ++i) {
                double tx = available_targets[i].first;
                double ty = available_targets[i].second;

                double dx = node->rel_x - tx;
                double dy = node->rel_y - ty;
                double dist = sqrt(dx*dx + dy*dy);

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

    // ====================== 【修改】队形过渡【脱离节点也参与移动】 ======================
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
                double adj = std::min((COLLISION_RADIUS * 2 - dis) / 2, MAX_ADJUST_STEP);

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

    //void UUVFormationSimulator::_record_transition_step(Json::Value& trajectory_result) {
    //    std::lock_guard<std::mutex> lock(sim_mutex);
    //    if (!is_transition) {
    //        return;
    //    }

    //    Json::Value step_data;
    //    step_data["sim_time"] = std::round(current_time * 1000) / 1000;
    //    //step_data["from_formation"] = last_formation;
    //    step_data["to_formation"] = static_cast<int>(config.current_formation);

    //    Json::Value nodes_json(Json::arrayValue);
    //    double max_error = 0.0;
    //    for (const auto& node : nodes) {
    //        double err = _calculate_formation_error(node);
    //        max_error = std::max(max_error, err);

    //        Json::Value node_json;
    //        node_json["id"] = node.id;
    //        node_json["lon"] = std::round(node.lon * 1e6) / 1e6;
    //        node_json["lat"] = std::round(node.lat * 1e6) / 1e6;
    //        node_json["speed"] = std::round(node.speed * 1e3) / 1e3;
    //        node_json["heading"] = std::round(node.heading * 1e3) / 1e3;
    //        node_json["rel_x"] = std::round(node.rel_x * 1e3) / 1e3;
    //        node_json["rel_y"] = std::round(node.rel_y * 1e3) / 1e3;
    //        node_json["target_x"] = std::round(node.target_x * 1e3) / 1e3;
    //        node_json["target_y"] = std::round(node.target_y * 1e3) / 1e3;
    //        node_json["formation_error"] = std::round(err * 1e4) / 1e4;

    //        nodes_json.append(node_json);
    //    }

    //    step_data["nodes"] = nodes_json;

    //    trajectory_result["frames"] = Json::Value(Json::arrayValue);

    //    // 4.1 构建单个帧对象
    //    Json::Value frame_obj;
    //    frame_obj["frame_id"] = frame_id;

    //    double err = _calculate_formation_error(node);
    //    max_error = std::max(max_error, err);
    //    // 4.2 构建该帧下的nodes数组
    //    Json::Value nodes(Json::arrayValue);
    //    for (const auto& node : nodes) {
    //        // 构建单个节点对象
    //        Json::Value node_obj;
    //        node_obj["node_id"] = uav_data.uav_id;
    //        node_obj["pos_x"] = uav_data.position.x; // 节点挂载pos对象
    //        node_obj["pos_y"] = uav_data.position.y;

    //        nodes.append(node_obj); // 节点加入该帧的nodes数组
    //    }

    //    // 4.3 帧对象挂载nodes数组
    //    frame_obj["nodes"] = nodes;

    //    // 4.4 帧对象加入frames数组
    //    trajectory_result["frames"].append(frame_obj);
    //    
    //    //transition_data.push_back(step_data);

    //    // 检测是否达到稳定状态
    //    //if (max_error < ERROR_STABLE_THRESHOLD) {
    //    //    is_transition = false;
    //    //    // 写入JSON文件

    //    //    /*std::ofstream f("trans.json");
    //    //    if (f.is_open()) {
    //    //        f << std::setw(2) << transition_data << std::endl;
    //    //        f.close();
    //    //        printf("✅ 变换完成！共 %zu 步，已写入 trans.json\n", transition_data.size());
    //    //    }*/
    //    //}
    //}

    std::vector<std::pair<double, double>> UUVFormationSimulator::_generate_formation_positions(int cnt) {
        std::vector<std::pair<double, double>> positions;
        if (cnt < 1 || cnt > 9) {
            throw std::invalid_argument("从节点数量必须在1~9之间，当前值：" + std::to_string(cnt));
        }

        double d = config.rel_distance;
        Formation_Type f = config.trans_formation;

        if (f == Formation_Type::Line) {
            for (int i = 1; i <= cnt; ++i) {
                positions.emplace_back(0.0, -i * d);
            }
        }
        else if (f == Formation_Type::Rectangle) {
            if (cnt < 3) {
                throw std::invalid_argument("矩形队形从节点数量必须在3~9之间，当前值：" + std::to_string(cnt));
            }
            int num = cnt + 1;
            if (num == 4) {
                positions = {{d, 0}, {d, -d}, {0, -d}};
            } else if (num == 5) {
                positions = {{-d, d}, {-d, -d}, {d, d}, {d, -d}};
            } else if (num == 6) {
                positions = {{-d, 0}, {d, 0}, {-d, -d}, {0, -d}, {d, -d}};
            } else if (num == 7) {
                positions = {{-d, 0}, {d, 0}, {0, -d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}};
            } else if (num == 8) {
                positions = {{-d, 0}, {d, 0}, {-d, -d}, {d, -d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}};
            } else if (num == 9) {
                positions = {{-d, 0}, {d, 0}, {-d, -d}, {0, -d}, {d, -d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}};
            } else if (num == 10) {
                positions = {{-2*d,0}, {-d,0}, {d,0}, {2*d,0}, {-2*d,-d}, {-d,-d}, {0,-d}, {d,-d}, {2*d,-d}};
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
                throw std::invalid_argument("菱形队形从节点数量必须在3~9之间，当前值：" + std::to_string(cnt));
            }
            int num = cnt + 1;
            if (num == 4) {
                positions = {{-d, 0}, {0, -d}, {d, 0}};
            } else if (num == 5) {
                positions = {{0, d}, {-d, 0}, {d, 0}, {0, -d}};
            } else if (num == 6) {
                positions = {{0, d}, {-d, 0}, {d, 0}, {0, -d}, {0, -2*d}};
            } else if (num == 7) {
                positions = {{0, -d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}, {0, -3*d}, {0, -4*d}};
            } else if (num == 8) {
                positions = {{-d/2, -d}, {d/2, -d}, {-d, -2*d}, {d, -2*d}, {-d/2, -3*d}, {d/2, -3*d}, {0, -4*d}};
            } else if (num == 9) {
                positions = {{-d/2, -d}, {d/2, -d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}, {-d/2, -3*d}, {d/2, -3*d}, {0, -4*d}};
            } else if (num == 10) {
                positions = {{-d/2, -d}, {d/2, -d}, {-1.5*d, -2*d}, {-0.5*d, -2*d}, {0.5*d, -2*d}, {1.5*d, -2*d}, {-d/2, -3*d}, {d/2, -3*d}, {0, -4*d}};
            }
        }
        else if (f == Formation_Type::Triangle) {
            if (cnt < 2) {
                throw std::invalid_argument("三角形队形从节点数量必须在2~9之间，当前值：" + std::to_string(cnt));
            }
            int num = cnt + 1;
            if (num == 3) {
                positions = {{-d, -d}, {d, -d}};
            } else if (num == 4) {
                positions = {{0, d}, {-d, -d}, {d, -d}};
            } else if (num == 5) {
                positions = {{-d/2, -d}, {d/2, -d}, {-d, -2*d}, {d, -2*d}};
            } else if (num == 6) {
                positions = {{-d/2, -d}, {d/2, -d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}};
            } else if (num == 7) {
                positions = {{-d/2, -d}, {d/2, -d}, {-1.5*d, -2*d}, {-0.5*d, -2*d}, {0.5*d, -2*d}, {1.5*d, -2*d}};
            } else if (num == 8) {
                positions = {{-d, -d}, {0, -d}, {d, -d}, {-1.5*d, -2*d}, {-0.5*d, -2*d}, {0.5*d, -2*d}, {1.5*d, -2*d}};
            } else if (num == 9) {
                positions = {{-d, -d}, {0, -d}, {d, -d}, {-2*d, -2*d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}, {2*d, -2*d}};
            } else if (num == 10) {
                positions = {{-d/2, -d}, {d/2, -d}, {-d, -2*d}, {0, -2*d}, {d, -2*d}, {-1.5*d, -3*d}, {-0.5*d, -3*d}, {0.5*d, -3*d}, {1.5*d, -3*d}};
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

    UAVTrajectory& UUVFormationSimulator::step_simulation() {
        std::lock_guard<std::mutex> lock(sim_mutex);
        trajectory_.clearAllTrajectory();
        for (double cnt = 0; cnt < (config.return_frames/10); cnt += config.sim_step) {
            current_time += config.sim_step;
            _update_maneuver();
            trajectory_.addFrame((current_time * 10), config.current_formation, nodes);
            _record_transition_step();
        }
        return trajectory_;
    }

    // ====================== 【新增】编队稳定判断 ======================
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
    void UUVFormationSimulator::add_node(double lon, double lat, double speed, double heading, int join_frames) {
        std::lock_guard<std::mutex> lock(sim_mutex);
        if (nodes.size() >= 10) {
            printf("❌ 已达到最大节点数10个，无法添加\n");
            return;
        }

        max_id += 1;
        UUVNode& main = nodes[0];

        // 计算新节点相对于主节点的初始位置
        auto [rel_x, rel_y] = _geo2enu(lon, lat, main.pos_.lon_deg, main.pos_.lat_deg);

        // 创建新节点
        UUVNode new_node;
        new_node.id = max_id;
        new_node.pos_.lon_deg = lon;
        new_node.pos_.lat_deg = lat;
        new_node.speed = speed;
        new_node.heading = heading;
        new_node.rel_x = rel_x;
        new_node.rel_y = rel_y;
        new_node.last_rel_x = rel_x;
        new_node.last_rel_y = rel_y;
        new_node.is_joining = true;
        new_node.join_progress = 0.0;
        new_node.join_total_frames = join_frames;
        new_node.is_leaving = false;

        nodes.push_back(new_node);
        config.node_num = static_cast<int>(nodes.size());

        _set_target_formation();
        is_transition = true;

        printf("\n✅ 成功添加节点 ID:%d\n", max_id);
        printf("   所有节点正在重排为 %zu 节点队形...\n", nodes.size());
    }

    // ====================== 删除末尾节点(改变脱离方向) ======================
    void UUVFormationSimulator::remove_last_node() {
        std::lock_guard<std::mutex> lock(sim_mutex);
        if (nodes.size() <= 2) {
            printf("❌ 节点数过少，无法删除\n");
            return;
        }

        UUVNode* leave_node = nullptr;
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
        UUVNode& main = nodes[0];

        // ====================== 【修改为Python逻辑】 ======================
        double opposite_heading = fmod(main.heading + 90.0, 360.0);  //  改成 +90°
        if (opposite_heading < 0) opposite_heading += 360.0;
        double opposite_rad = to_radians(opposite_heading);

        double leave_dist = config.rel_distance * 15.0;  //  15倍距离
        leave_node->leave_target_x = leave_dist * std::sin(opposite_rad);
        leave_node->leave_target_y = leave_dist * std::cos(opposite_rad);

        _set_target_formation();
        is_transition = true;

        printf("\n✅ 节点 ID:%d 开始脱离\n", leave_node->id);
        printf("   脱离目标相对位置: (%.1f, %.1f)\n", leave_node->leave_target_x, leave_node->leave_target_y);
    }

    UAVTrajectory& UUVFormationSimulator::getUAVtrajectory()
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        return trajectory_;
    }

    double UUVFormationSimulator::_calculate_formation_error(const UUVNode& node) {
        if (node.id == 0) {
            return 0.0;
        }
        return std::hypot(node.rel_x - node.target_x, node.rel_y - node.target_y);
    }

    void UUVFormationSimulator::InitialParams(FormationConfig& forparams_)
    {
        // 初始化位置（默认第一个队形）
        std::lock_guard<std::mutex> lock(sim_mutex);
        config = forparams_;
    }

    // ====================== 外部接口实现 ======================
    void Init_formation(const FormationConfig& config, Json::Value& trajectory_result) {
        if (g_pFormationSimulator != nullptr) {
            delete g_pFormationSimulator;
            g_pFormationSimulator = nullptr;
        }
        try {
            g_pFormationSimulator = new UUVFormationSimulator(config);
            printf("编队初始化成功！\n");

            //获取初始化队形
            UAVTrajectory& trajectory_data = g_pFormationSimulator->getUAVtrajectory();

            auto all_trajectory = trajectory_data.getAllTrajectory();
            if (all_trajectory.empty()) return;

            trajectory_result["frames"] = Json::Value(Json::arrayValue);

            // 3. 遍历分组，构建每个帧的层级
            for (const auto& group : all_trajectory) {
                int frame_id = group.frame;                // 帧号
                const auto& frame_nodes = group.nodes_;    // 该帧下的所有节点

                // 4.1 构建单个帧对象
                Json::Value frame_obj;
                frame_obj["frame_id"] = frame_id;
                frame_obj["formation_type"] = static_cast<int>(group.formation);

                // 4.2 构建该帧下的nodes数组
                Json::Value nodes(Json::arrayValue);
                double max_error = 0.0;
                for (const auto& node : frame_nodes) {
                    double err = g_pFormationSimulator->_calculate_formation_error(node);
                    max_error = std::max(max_error, err);
                    // 构建单个节点对象
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

                    nodes.append(node_json); // 节点加入该帧的nodes数组
                }

                // 4.3 帧对象挂载nodes数组
                frame_obj["nodes"] = nodes;

                // 4.4 帧对象加入frames数组
                trajectory_result["frames"].append(frame_obj);
            }
        }
        catch (const std::exception& e) {
            printf("编队初始化失败：%s\n", e.what());
            g_pFormationSimulator = nullptr;
        }
    }

    void SwitchFormation(const Formation_Type& type) {
        if (g_pFormationSimulator == nullptr) {
            printf("仿真器未初始化！\n");
        }
        g_pFormationSimulator->switch_formation(type);
    }

    void TurnFormation(double heading_rate) {
        if (g_pFormationSimulator == nullptr) {
            printf("仿真器未初始化！\n");
        }
        g_pFormationSimulator->set_heading_rate(heading_rate);
    }

    // ====================== 【新增】外部接口：添加节点 ======================
    void AddNode(double lon, double lat, double speed, double heading, int join_frames) {
        if (g_pFormationSimulator == nullptr) {
            printf("仿真器未初始化！\n");
            return;
        }
        g_pFormationSimulator->add_node(lon, lat, speed, heading, join_frames);
    }

    // ====================== 【新增】外部接口：删除节点 ======================
    void RemoveLastNode() {
        if (g_pFormationSimulator == nullptr) {
            printf("仿真器未初始化！\n");
            return;
        }
        g_pFormationSimulator->remove_last_node();
    }

    //void Transformation_Use(CalcTempParam& task_param) {
    //    if (g_pFormationSimulator == nullptr) {
    //        printf("仿真器未初始化！\n");
    //    }
    //    UAVTrajectory trajectory_data = g_pFormationSimulator->step_simulation();
    //    // 2. 清空并写入航迹点数组（核心：对接欺骗式干扰的结果字段）
    //    task_param.trajectory_result.clear(); // 可选，复用对象时建议保留

    //    //运行帧数计算
    //    task_param.run_frames = g_pFormationSimulator->getRunframe() * 10;

    //    auto all_trajectory = trajectory_data.getAllTrajectory();
    //    if (all_trajectory.empty()) return;

    //    //Json::Value frame_nodes = Json::arrayValue;
    //    //frame_nodes = task_param.trajectory_result["frames"];

    //    // 3. 遍历分组，构建每个帧的层级
    //    for (const auto& group : all_trajectory) {
    //        int frame_id = group.frame;                // 帧号
    //        const auto& frame_nodes = group.nodes_;    // 该帧下的所有节点

    //        // 4.1 构建单个帧对象
    //        Json::Value frame_obj;
    //        frame_obj["frame_id"] = frame_id;

    //        // 4.2 构建该帧下的nodes数组
    //        Json::Value& nodes = frame_obj["nodes"];
    //        double max_error = 0.0;
    //        for (const auto& node : frame_nodes) {
    //            double err = g_pFormationSimulator->_calculate_formation_error(node);
    //            max_error = std::max(max_error, err);
    //            // 构建单个节点对象
    //            Json::Value node_json;
    //            node_json["node_id"] = node.id;
    //            node_json["lon"] = std::round(node.pos_.lon_deg * 1e6) / 1e6;
    //            node_json["lat"] = std::round(node.pos_.lat_deg * 1e6) / 1e6;
    //            node_json["speed"] = std::round(node.speed * 1e3) / 1e3;
    //            node_json["heading"] = std::round(node.heading * 1e3) / 1e3;
    //            node_json["rel_x"] = std::round(node.rel_x * 1e3) / 1e3;
    //            node_json["rel_y"] = std::round(node.rel_y * 1e3) / 1e3;
    //            node_json["target_x"] = std::round(node.target_x * 1e3) / 1e3;
    //            node_json["target_y"] = std::round(node.target_y * 1e3) / 1e3;
    //            node_json["formation_error"] = std::round(err * 1e4) / 1e4;

    //            nodes.append(node_json); // 节点加入该帧的nodes数组
    //        }

    //        // 4.3 帧对象挂载nodes数组
    //        //frame_obj["nodes"] = nodes;
    //        //nodes = frame_obj["nodes"];

    //        // 4.4 帧对象加入frames数组
    //        task_param.trajectory_result.append(frame_obj);
    //    }
    //}

    void Transformation_Use(CalcTempParam& task_param) {
        if (g_pFormationSimulator == nullptr) {
            printf("仿真器未初始化！\n");
            return; // 必须return，否则崩溃
        }

        // 获取轨迹数据
        UAVTrajectory trajectory_data = g_pFormationSimulator->step_simulation();
        auto all_trajectory = trajectory_data.getAllTrajectory();

        // 清空输出
        task_param.trajectory_result.clear();

        // ======================
        // 【关键修复1】手动创建 "frames" 数组
        // ======================
        Json::Value frames_array(Json::arrayValue);

        // 运行帧数
        task_param.run_frames = g_pFormationSimulator->getRunframe() * 10;

        if (all_trajectory.empty()) {
            // 空的也要把结构写对，防止输出NULL
            task_param.trajectory_result["frames"] = frames_array;
            return;
        }

        // 遍历每一帧
        for (const auto& group : all_trajectory) {
            int frame_id = group.frame;
            const auto& frame_nodes = group.nodes_;

            Json::Value frame_obj;
            frame_obj["frame_id"] = frame_id;
            frame_obj["formation_type"] = static_cast<int>(group.formation);

            Json::Value nodes_array(Json::arrayValue); // 节点数组

            for (const auto& node : frame_nodes) {
                double err = g_pFormationSimulator->_calculate_formation_error(node);

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

            // 把节点数组放入当前帧
            frame_obj["nodes"] = nodes_array;

            // ======================
            // 【关键修复2】加入 frames 数组
            // ======================
            frames_array.append(frame_obj);
        }

        // ======================
        // 【关键修复3】最终绑定到 trajectory_result["frames"]
        // ======================
        task_param.trajectory_result["frames"] = frames_array;
    }


    /**
    * @brief 添加单帧轨迹数据
    */
    void UAVTrajectory::addFrame(int frame, const Formation_Type formation, const vector<UUVNode>& nodes)
    {
        trajectory_data.push_back({ frame, formation, nodes });
    }

    /**
     * @brief 记录队形变换帧数
     */
    void UAVTrajectory::addFormationChangeFrame(int frame) {
        formation_change_frames.push_back(frame);
    }

    void UAVTrajectory::clearAllTrajectory()
    {
        trajectory_data.clear();
    }

    /**
     * @brief 获取所有轨迹数据
     */
    const std::vector<TrajectoryFrame>& UAVTrajectory::getAllTrajectory() const {
        return trajectory_data;
    }

    /**
     * @brief 获取队形变换帧数记录
     */
    const std::vector<int>& UAVTrajectory::getFormationChangeFrames() const {
        return formation_change_frames;
    }

}
