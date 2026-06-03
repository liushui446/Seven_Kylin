#ifndef TRANS_HPP
#define TRANS_HPP

#include "core/CommonCore.hpp"

namespace seven{

    // 全局常量定义
    const double R_EARTH = 6378137.0;
    const double TRANSITION_SPEED = 0.08;
    const double COLLISION_RADIUS = 4.0;
    const int MAX_COLLISION_ITER = 15;
    const double MAX_ADJUST_STEP = 1.0;
    const double ERROR_STABLE_THRESHOLD = 0.02;

    // 运动学物理约束
    const double MAX_SPEED = 1000.0;

    struct UUVNode;
    struct FormationConfig;
    class UUVFormationSimulator;

    string formationToStr(Formation_Type type);

    /**
    * @brief 轨迹存储类
    */
    class UAVTrajectory {
    private:
        std::vector<TrajectoryFrame> trajectory_data;  // 所有轨迹数据
        std::vector<int> formation_change_frames;      // 队形变换帧数记录

    public:
        /**
         * @brief 添加单帧轨迹数据
         */
        void addFrame(int frame, const Formation_Type formation, const vector<UUVNode>& nodes);

        /**
         * @brief 记录队形变换帧数
         */
        void addFormationChangeFrame(int frame);

        /**
         * @brief 清空所有轨迹数据
         */
        void clearAllTrajectory();

        /**
         * @brief 获取所有轨迹数据
         */
        const std::vector<TrajectoryFrame>& getAllTrajectory() const;

        /**
         * @brief 获取队形变换帧数记录
         */
        const std::vector<int>& getFormationChangeFrames() const;
    };

    // UUV编队仿真核心类
    class UUVFormationSimulator {
    private:
        FormationConfig config;
        std::vector<UUVNode> nodes;
        UAVTrajectory trajectory_;  //轨迹数据
        double current_time;
        double last_output_time;
        bool is_transition;
        int max_id; // 跟踪最大节点ID
        Formation_Type last_formation;
        std::mutex sim_mutex;  // 线程安全锁

        // 私有方法
        void _validate_config();
        void _init_nodes();
        void _set_target_formation();
        void _set_initial_position();
        std::vector<std::pair<double, double>> _generate_formation_positions(int cnt);
        void _transition_formation();
        std::vector<Point2D> checkCollision1(const std::vector<Point2D>& positions);
        void apply_collision_avoidance();
       // void _record_transition_step(Json::Value& trajectory_result);
        std::pair<double, double> _geo2enu(double lon, double lat, double rlon, double rlat);
        std::pair<double, double> _enu2geo(double x, double y, double rlon, double rlat);
        void _update_maneuver();
        void _record_transition_step();

    public:
        // 构造函数
        UUVFormationSimulator(const FormationConfig& cfg);

        // 队形切换
        void switch_formation(const Formation_Type& cmd);

        // 仿真步进
        UAVTrajectory& step_simulation();

        void add_node(double lon, double lat, double speed, double heading, int join_frames);

        void remove_last_node();

        // 获取当前配置
        FormationConfig get_config() const { return config; }

        double getRunframe() const { return current_time; }

        // 获取当前轨迹数据
        UAVTrajectory& getUAVtrajectory();

        double _calculate_formation_error(const UUVNode& node);

        // 设置航向变化率
        void set_heading_rate(double rate) { config.heading_rate = rate; }

        void InitialParams(FormationConfig& forparams_);
    };

    // 外部接口声明（集成到服务端的核心接口）
    // 初始化编队
    void SEVEN_EXPORTS Init_formation(const FormationConfig& config, Json::Value& trajectory_result);

    // 编队变换执行（每帧调用）
    void SEVEN_EXPORTS Transformation_Use(CalcTempParam& task_param);

    void SEVEN_EXPORTS SwitchFormation(const Formation_Type& type);

    void SEVEN_EXPORTS TurnFormation(double heading_rate);

    void SEVEN_EXPORTS AddNode(double lon, double lat, double speed, double heading, int join_frames);

    void SEVEN_EXPORTS RemoveLastNode();
    // 全局仿真器实例（服务端单例使用）
    extern UUVFormationSimulator* g_pFormationSimulator;

}

#endif
