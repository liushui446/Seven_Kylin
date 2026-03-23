#ifndef TRANS_HPP
#define TRANS_HPP

#include "core/CommonCore.hpp"

namespace seven{

    // 全局常量定义
    const double R_EARTH = 6378137.0;
    const double WINDOW_RANGE = 80.0;
    const double TRANSITION_SPEED = 0.08;
    const double COLLISION_RADIUS = 4.0;
    const int MAX_COLLISION_ITER = 15;
    const double MAX_ADJUST_STEP = 1.0;
    const double ERROR_STABLE_THRESHOLD = 0.02;

    // 运动学物理约束
    const double MAX_SPEED = 5.0;
    const double MAX_TURN_RATE = 45.0;  // 最大转向速率 (度/秒)

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

    public:
        // 构造函数
        UUVFormationSimulator(const FormationConfig& cfg);

        // 队形切换
        void switch_formation(const Formation_Type& cmd);

        // 仿真步进
        UAVTrajectory& step_simulation();

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

    // 全局仿真器实例（服务端单例使用）
    extern UUVFormationSimulator* g_pFormationSimulator;

}

namespace seven {

    

    /**
    * @brief 无人机编队变换核心类
    */
    class UAVFormationTransformer {
    private:
        UAVFormationParams params_;              // 初始化参数
        UAVTrajectory trajectory_;               // 轨迹数据
        std::vector<Point2D> current_positions_; // 当前位置
        std::vector<Point2D> target_positions_;  // 目标位置
        std::vector<double> initial_distances_;   // 缓存每架无人机到匹配目标的初始距离
        //int current_formation_idx_ = 0;        // 当前队形索引
        Formation_Type current_formation;        // 当前队形
        int frame_count_ = 0;                    // 当前帧数

        /**
         * @brief 生成指定类型的队形位置
         */
        std::vector<Point2D> generateFormation(const Formation_Type& formation_type);

        //动态调整平缓指数
        double calculateDynamicAlpha(int uav_idx, double current_dist);

        /**
         * @brief 匹配最近的目标位置
         */
        std::vector<int> matchClosestTarget();

        /**
        * @brief 碰撞检测与位置调整
        */
        std::vector<Point2D> checkCollision(const std::vector<Point2D>& positions);

        std::vector<Point2D> checkCollision1(const std::vector<Point2D>& positions);

        /**
         * @brief 更新无人机位置（平滑过渡）
         */
        bool updatePositions();

    public:
        /**
         * @brief 构造函数（初始化参数）
         */
        UAVFormationTransformer();

        //初始化参数
        void InitialParams(UAVFormationParams& forparams_);

        //初始化队形
        Formation_Type InitialFormation();

        /**
        * @brief 切换到下一个队形
        */
        void switchFormation();

        /**
         * @brief 运行编队变换计算（生成轨迹）
         */
        Formation_Type runTransformation(vector<TrajectoryFrame>& end_trajectory);

        /**
         * @brief 获取轨迹数据
         */
        const UAVTrajectory& getTrajectory() const;

        /**
         * @brief 获取当前队形名称
         */
        Formation_Type getCurrentFormation() const;

        /**
         * @brief 获取当前所有无人机位置
         */
        std::vector<Point2D> getCurrentPositions() const;

        void setCurrentPositions(vector<TrajectoryFrame> positions_);

        /**
         * @brief 队形类型转string
         */
        //string formationToStr(Formation_Type type) const;

    };

    //void SEVEN_EXPORTS Transformation_Test(Json::Value input, Json::Value& trajectory_result);
    void SEVEN_EXPORTS Transformation_Use(UAVFormationParams& params, vector<TrajectoryFrame>& initial_trajectory,
        vector<TrajectoryFrame>& end_trajectory, Json::Value input, Json::Value& trajectory_result);
    void SEVEN_EXPORTS Init_formation(UAVFormationParams& formation_param_, vector<TrajectoryFrame>& initial_trajectory,
        vector<TrajectoryFrame>& end_trajectory, Json::Value& trajectory_result);

    //static UAVFormationParams formation_param_;
    //static vector<TrajectoryFrame> initial_trajectory;
    //static vector<TrajectoryFrame> end_trajectory;
}

#endif
