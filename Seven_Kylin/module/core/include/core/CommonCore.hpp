#ifndef COMMONCORE_HPP
#define COMMONCORE_HPP

#include <jsoncpp/json/json.h>

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <set>
#include <deque>
#include <algorithm>
#include <functional>
#include <filesystem>
#include <mutex>
#include <fstream>
#include <unordered_map>
#include <cmath>
#include <stdexcept>
#include <mutex>
#include <thread>
#include <iomanip>
#include <condition_variable>
#include <atomic>

// 跨平台导出宏（Windows / 麒麟V10 Linux 通用）
#ifdef _WIN32
#   define SEVEN_EXPORTS __declspec(dllexport)
#else
#   define SEVEN_EXPORTS
#endif

# if (defined Windows)
#   define SEVEN_EXPORTS __declspec(dllexport)
# elif defined Linux
#   define SEVEN_EXPORTS __attribute__ ((visibility ("default")))
# endif

using std::cout;
using std::endl;
using std::map;
using std::set;
using std::pair;
using std::deque;
using std::vector;
using std::string;
using std::to_string;
using std::shared_ptr;
using std::make_shared;
using std::unique_ptr;
using std::make_unique;
using std::for_each;
using std::mutex;
using std::function;
using std::condition_variable;
using std::ifstream;
using std::ofstream;
using std::thread;
using std::dynamic_pointer_cast;
using std::tuple;

typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;
typedef double				DOUBLE;
typedef int                 INT;
typedef unsigned int        UINT;

typedef int HANDLE;

typedef unsigned char uchar;
typedef unsigned short ushort;

// 宏定义常量
#define M_PI 3.141592653589793238462643383

#define GNSS_FC 1575.42e6 // GNSS中心频率(GPS L1频段, Hz)

namespace seven
{
    enum class SimState {
        ENDDING = 0,  // 仿真终止
        STOPPED = 1,  // 仿真暂停
        RUNNING = 2,   // 仿真运行中
        IDLE = 3   // 仿真准备状态
    };

    enum class Cmd_Type
    {
        Barrage = 1,             // 压制
        Deception = 2,           // 欺骗
        Transformation = 3       // 编队
    };

    enum class Sim_Type
    {
        STARTTED = 1,  // 仿真准备
        RUNNING = 2,   // 仿真运行中
        STOPPED = 3,   // 仿真暂停
        ENDDING = 4    // 仿真结束
    };

    enum class Jammer_Level
    {
        Low = 1,                // 弱
        Middle = 2,             // 中
        High = 3                // 强
    };

    enum class Formation_Type
    {
        Rectangle = 1,
        Triangle = 2,
        Circle = 3,
        Diamond = 4,
        Line = 5
    };

    // 坐标结构体定义
    struct LLA {
        double lon_deg = 0.0;  // 经度(°)
        double lat_deg = 0.0;  // 纬度(°)
        double h_m = 0.0;     // 高度(km)
        int    current_run_t = 0;         // 运行帧

        // 【加法运算符】LLA + LLA，对应分量逐元素相加
        friend LLA operator+(const LLA& lhs, const LLA& rhs) {
            return { lhs.lon_deg + rhs.lon_deg,
                    lhs.lat_deg + rhs.lat_deg,
                    lhs.h_m + rhs.h_m };
        }

        // 【减法运算符】LLA - LLA，对应分量逐元素相减
        friend LLA operator-(const LLA& lhs, const LLA& rhs) {
            return { lhs.lon_deg - rhs.lon_deg,
                    lhs.lat_deg - rhs.lat_deg,
                    lhs.h_m - rhs.h_m };
        }

        // 【点乘运算符】LLA · LLA，返回标量值（分量相乘后求和）
        // 点乘是向量运算核心，结果为double类型
        friend LLA operator*(const LLA& lhs, const double& other) {
            double lon_deg = lhs.lon_deg * other;
            double lat_deg = lhs.lat_deg * other;
            double h_m = lhs.h_m * other;
            return { lon_deg, lat_deg, h_m };
        }

        // 拓展：自增/自减运算符（可选，工程中常用，贴合使用习惯）
        LLA& operator+=(const LLA& other) {
            *this = *this + other;
            return *this;
        }
        LLA& operator-=(const LLA& other) {
            *this = *this - other;
            return *this;
        }
        LLA& operator=(const LLA& other) {
            this->lon_deg = other.lon_deg;
            this->lat_deg = other.lat_deg;
            this->h_m = other.h_m;
            this->current_run_t = other.current_run_t;
            return *this;
        }
    };

    // ECEF坐标结构体
    struct ECEF {
        double X = 0.0;        // X轴(m)
        double Y = 0.0;        // Y轴(m)
        double Z = 0.0;        // Z轴(m)

        // 【加法运算符】ECEF + ECEF，对应分量逐元素相加
        friend ECEF operator+(const ECEF& lhs, const ECEF& rhs) {
            return { lhs.X + rhs.X,
                    lhs.Y + rhs.Y,
                    lhs.Z + rhs.Z };
        }

        // 【减法运算符】ECEF - ECEF，对应分量逐元素相减
        friend ECEF operator-(const ECEF& lhs, const ECEF& rhs) {
            return { lhs.X - rhs.X,
                    lhs.Y - rhs.Y,
                    lhs.Z - rhs.Z };
        }

        // 【点乘运算符】LLA · LLA，返回标量值（分量相乘后求和）
        // 点乘是向量运算核心，结果为double类型
        friend ECEF operator*(const ECEF& lhs, const double& other) {
            double x = lhs.X * other;
            double y = lhs.Y * other;
            double z = lhs.Z * other;
            return { x, y, z };
        }

        // 拓展：自增/自减运算符（可选，工程中常用，贴合使用习惯）
        ECEF& operator+=(const ECEF& other) {
            *this = *this + other;
            return *this;
        }
        ECEF& operator-=(const ECEF& other) {
            *this = *this - other;
            return *this;
        }
        ECEF& operator=(const ECEF& other) {
            this->X = other.X;
            this->Y = other.Y;
            this->Z = other.Z;
            return *this;
        }
    };

    /**
     * @brief 二维坐标结构体
     */
    struct Point2D {
        double x = 0.0;
        double y = 0.0;

        Point2D() = default;
        Point2D(double x_, double y_) : x(x_), y(y_) {}

        // 运算符重载：向量运算
        Point2D operator+(const Point2D& other) const {
            return { x + other.x, y + other.y };
        }

        Point2D operator-(const Point2D& other) const {
            return { x - other.x, y - other.y };
        }

        Point2D operator*(double scalar) const {
            return { x * scalar, y * scalar };
        }

        Point2D operator/(double scalar) const {
            if (std::fabs(scalar) < 1e-6) {
                throw std::runtime_error("Point2D division by zero");
            }
            return { x / scalar, y / scalar };
        }

        Point2D& operator=(const Point2D& other) {
            if (this == &other) {  // 处理自赋值
                return *this;
            }
            x = other.x;
            y = other.y;
            return *this;
        }

        // 向量长度
        double norm() const {
            return std::sqrt(x * x + y * y);
        }

        // 单位向量
        Point2D normalized() const {
            double n = norm();
            return n < 1e-6 ? Point2D(0, 0) : Point2D(x / n, y / n);
        }
    };

    // 新增：干扰范围计算结果结构体
    struct JammerRangeResult {
        int jammer_id;               // 干扰源ID
        double jammer_radius;        // 干扰半径(米)
        ECEF jammer_centre;          // 干扰源中心(ECEF)
        ECEF jammer_start_point;     // 干扰开始点
        ECEF jammer_end_point;       // 干扰结束点
        LLA jammer_centre_lla;       // 干扰源中心(LLA)
        LLA jammer_start_lla;        // 干扰开始点(LLA)
        LLA jammer_end_lla;          // 干扰结束点(LLA)
        UINT platform_id;            // 对应航迹平台ID
    };

    struct InputPlatParam {
        UINT plat_id;       // 平台ID
        LLA plat_initial_pos;
        LLA cur_plat_pos;   // 平台经纬高(°/km)
        LLA cur_plat_vec;   // 平台经纬高速度(°/km)
        LLA deception_pos;

        InputPlatParam()
            : plat_id(0)
            , plat_initial_pos()
            , cur_plat_pos()
            , cur_plat_vec()
            , deception_pos()
        {
        }

        InputPlatParam& operator=(const InputPlatParam& other) {
            this->plat_id = other.plat_id;
            this->plat_initial_pos = other.plat_initial_pos;
            this->cur_plat_pos = other.cur_plat_pos;
            this->cur_plat_vec = other.cur_plat_vec;
            this->deception_pos = other.deception_pos;
            return *this;
        }
    };

    struct CalcParam {
        UINT run_frames_cnt;
        UINT return_frames;  // 返回结果数据帧数
        UINT sim_time_;

        CalcParam()
            : run_frames_cnt(0)
            , return_frames(100)
            , sim_time_(2000)
        {
        }

        CalcParam& operator=(const CalcParam& other) {
            this->run_frames_cnt = other.run_frames_cnt;
            this->return_frames = other.return_frames;
            this->sim_time_ = other.sim_time_;
            return *this;
        }
    };

    struct CalcTempParam {
        Json::Value input;                       // 输入JSON数据
        Json::Value trajectory_result;           // 输出结果
        std::atomic<int> max_frames;			 // 运行最大帧数
        std::atomic<int> run_frames;			 // 每次运行帧数
        std::atomic<int> return_frames;			 // 每次运行帧数
        vector<InputPlatParam> serveral_plat;
    };

    // UUV节点结构体（对应Python的UUVNode）
    struct UUVNode {
        int id;
        //double lon;       // 经度
        //double lat;       // 纬度
        double speed;     // 速度 (m/s)
        double heading;   // 航向 (度)
        double rel_x;     // 相对东向坐标 (m)
        double rel_y;     // 相对北向坐标 (m)
        double target_x;  // 目标相对东向坐标 (m)
        double target_y;  // 目标相对北向坐标 (m)
        double last_rel_x;// 上一帧相对东向坐标
        double last_rel_y;// 上一帧相对北向坐标

        int join_total_frames = 0;
        double join_progress = 0.0;
        double leave_target_x = 0.0;
        double leave_target_y = 0.0;
        bool is_joining = false;
        bool is_leaving = false;

        LLA pos_;         // 经纬度

        UUVNode() : id(0), speed(0.0), heading(0.0),
            rel_x(0.0), rel_y(0.0), target_x(0.0), target_y(0.0),
            last_rel_x(0.0), last_rel_y(0.0), pos_() {}

        UUVNode(int id_, double lon_, double lat_, double speed_, double heading_)
            : id(id_), speed(speed_), heading(heading_),
            rel_x(0.0), rel_y(0.0), target_x(0.0), target_y(0.0),
            last_rel_x(0.0), last_rel_y(0.0), pos_() {}

        UUVNode& operator=(const UUVNode& other) {

            this->id = other.id;
            this->speed = other.speed;
            this->heading = other.heading;
            this->rel_x = other.rel_x;
            this->rel_y = other.rel_y;
            this->target_x = other.target_x;
            this->target_y = other.target_y;
            this->last_rel_x = other.last_rel_x;
            this->last_rel_y = other.last_rel_y;

            this->is_joining = other.is_joining;
            this->join_progress = other.join_progress;
            this->join_total_frames = other.join_total_frames;
            this->is_leaving = other.is_leaving;
            this->leave_target_x = other.leave_target_x;
            this->leave_target_y = other.leave_target_y;

            this->pos_ = other.pos_;

            return *this;
        }

    };

    /**
     * @brief 单帧轨迹数据结构体
     */
    struct TrajectoryFrame {
        int frame = 0;          // 帧数
        Formation_Type formation;  // 当前队形
        vector<UUVNode> nodes_;     //节点数据

        // 可选：添加默认构造/拷贝构造，确保赋值正常
        TrajectoryFrame() = default;
        TrajectoryFrame(const TrajectoryFrame&) = default;
        TrajectoryFrame& operator=(const TrajectoryFrame&) = default;
    };

    /**
    * @brief 初始化参数结构体
    */
    struct UAVFormationParams {
        int num_uavs = 8;                // 节点数量
        double interval = 5.0;           // 队形节点间间隔（米）
        double collision_radius = 2.0;   // 避碰半径（米）
        //double switch_interval = 5.0;    // 队形切换间隔（秒）
        double transition_alpha_base = 0.5; // 基础α值（近距时使用）
        double transition_alpha_max = 0.02;  // 最大α值（远距时使用）
        double far_dist_ratio = 0.35;          // 远距阈值：初始距离的80%
        double transition_alpha = 0.02;  // 位置过渡系数（越小越平滑）0.05
        double max_adjust_step = 0.1;     // 单次最大调整0.1米
        int max_collision_iter = 100;   // 最大避碰迭代次数
        int fps = 30;                    // 帧率（用于时间换算）
        int max_frames = 200;           // 最大运行帧数(1500)
        UINT return_frames = 100;        // 返回结果数据帧数
        bool isInitial = false;          // 是否进行编队初始化

        // 队形序列（默认：矩形→三角形→圆形→菱形→直线）
        //std::vector<std::string> formation_sequence = { "rectangle", "triangle", "circle", "diamond", "line" };
        Formation_Type trans_formation;  // 需要变换的队形
        Formation_Type current_formation;// 当前队形
        Point2D pos_center;              // 队形中心点位置

        UAVFormationParams& operator=(const UAVFormationParams& other) {

            num_uavs = other.num_uavs;
            interval = other.interval;
            collision_radius = other.collision_radius;
            transition_alpha = other.transition_alpha;
            fps = other.fps;
            max_frames = other.max_frames;
            isInitial = other.isInitial;

            trans_formation = other.trans_formation;
            current_formation = other.current_formation;
            pos_center = other.pos_center;

            return *this;
        }

        // 可选：添加默认构造/拷贝构造
        UAVFormationParams() = default;
        UAVFormationParams(const UAVFormationParams&) = default;

    };

    // 编队配置结构体
    struct FormationConfig {
        int node_num;                // 节点数量 (4~10)
        int max_frames;           // 最大运行帧数(1500)
        int return_frames;        // 返回结果数据帧数
        double rel_distance;         // 节点间距 (m)
        double collision_radius;     // 碰撞半径 (m)
        double init_speed;           // 初始速度 (m/s)
        double init_heading;         // 初始航向 (度)
        double heading_rate;         // 航向变化率 (度/秒，正=逆时针，负=顺时针)
        double acceleration;         // 加速度 (m/s²)
        double sim_step;             // 仿真步长 (s)
        LLA main_node;               // 主节点经纬度
        Formation_Type trans_formation;  // 需要变换的队形(line/rect/circle/diamond/triangle)
        Formation_Type current_formation;// 当前队形
        
        //double output_interval;      // 输出间隔 (s)

        FormationConfig() : node_num(10), max_frames(3000),
            return_frames(10), rel_distance(10.0), collision_radius(4.0),
            init_speed(2.0), init_heading(0.0),heading_rate(2.0), acceleration(0.0), sim_step(0.1),
            main_node(), trans_formation(Formation_Type::Line), current_formation(Formation_Type::Line){}

        FormationConfig(const FormationConfig&) = default;

        FormationConfig& operator=(const FormationConfig& other) {

            this->node_num = other.node_num;
            this->max_frames = other.max_frames;
            this->return_frames = other.return_frames;
            this->max_frames = other.max_frames;

            this->rel_distance = other.rel_distance;
            this->collision_radius = other.collision_radius;
            this->init_speed = other.init_speed;
            this->init_heading = other.init_heading;
            this->heading_rate = other.heading_rate;
            this->acceleration = other.acceleration;
            this->main_node = other.main_node;

            this->trans_formation = other.trans_formation;
            this->current_formation = other.current_formation;

            return *this;
        }
    };

    // 卫星参数结构体
    struct SatelliteParam {
        std::vector<LLA> sat_pos;  // 卫星经纬高列表(°/km)
        double carrier_power;      // 卫星信号载波功率(W)

        SatelliteParam& operator=(const SatelliteParam& other) {
            this->sat_pos = other.sat_pos;
            this->carrier_power = other.carrier_power;
            return *this;
        }
        // 可选：添加默认构造/拷贝构造（保证结构体使用完整性）
        SatelliteParam() = default;
        SatelliteParam(const SatelliteParam& other) = default;
    };

    // 干扰源参数结构体
    struct JammerParam {
        LLA pos;             // 干扰源经纬高(°/km)
        double power;        // 发射功率(W)
        std::string type;    // 干扰样式：continuous_wave/multi-tone/bandlimited_gaussian/pseudocode/pulse
        double bandwidth;    // 干扰带宽(Hz)
        double freq;         // 干扰频率(Hz)
        double pulse_width;  // 脉冲宽度(s)（仅脉冲干扰）
        double pulse_period; // 脉冲周期(s)（仅脉冲干扰）

        // 赋值运算符重载
        JammerParam& operator=(const JammerParam& other) {
            // 1. 自赋值检查（避免不必要的拷贝，防止自我赋值导致的错误）
            if (this == &other) {
                return *this;
            }

            // 2. 逐字段拷贝（覆盖所有成员变量）
            pos = other.pos;               // 嵌套结构体赋值（依赖LLA的operator=）
            power = other.power;           // 发射功率
            type = other.type;             // 干扰样式（std::string自带深拷贝）
            bandwidth = other.bandwidth;   // 干扰带宽
            freq = other.freq;             // 干扰频率
            pulse_width = other.pulse_width; // 脉冲宽度
            pulse_period = other.pulse_period; // 脉冲周期

            // 3. 返回自身引用（支持链式赋值，如a = b = c）
            return *this;
        }

        // 可选：添加默认构造/拷贝构造（保证结构体使用完整性）
        JammerParam() = default;
        JammerParam(const JammerParam& other) = default;
    };

    struct DecJammerParam {
        LLA pos;             // 干扰源经纬高(°/km)
        double freq;         // 干扰频率(Hz)

        DecJammerParam& operator=(const DecJammerParam& other) {
            // 1. 自赋值检查（避免不必要的拷贝，防止自我赋值导致的错误）
            if (this == &other) {
                return *this;
            }

            pos = other.pos;
            freq = other.freq;

            return *this;
        }

        // 可选：添加默认构造/拷贝构造（保证结构体使用完整性）
        DecJammerParam() = default;
        DecJammerParam(const DecJammerParam& other) = default;
    };

    // 核心仿真配置结构体
    struct SimConfig {
        // 基础参数
        double Td = 0.02;          // 相关积分时间(s)
        double c = 3e8;        // 光速(m/s)
        double fc = GNSS_FC;       // GNSS中心频率(Hz)

        // 导航装备参数
        std::string combined_nav = "loose";  // 组合导航方式：loose/tight/deep
        std::string anti_jam_filter = "frequency"; // 抗干扰滤波方式
        std::string pseudocode = "C/A";      // 伪码类型：C/A/P(Y)/M
        double Tc = 9.77e-7;       // 伪码码元宽度(s)
        double fs = 10.23e6;      // M码副载频(Hz)
        double d = 1.0 / 8;          // 码跟踪误差系数
        double beta = 2e5;         // 接收机等效预相关带宽(Hz)2e6
        double ins_drift = 0.01;   // 惯导漂移率(km/s)

        // 跟踪环参数
        double Bp = 18;            // PLL带宽(Hz)
        double Bd = 18;            // DLL带宽(Hz)

        // 失锁判定阈值
        double pll_unlock_thresh = 15;  // 载波环失锁阈值(°)
        double dll_unlock_thresh = 1.0 / 8 / 6; // 码环失锁阈值

        // 标称载噪比
        double C_N0_nom = 45;      // dB-Hz
        double Bn = 2.046e6;       // 噪声带宽 (Hz)
        double G_ant = 10;         // 抗干扰波束成形增益 (dB)

        //UINT return_frames = 100;  // 返回结果数据帧数
        //UINT run_frames_cnt = 0;   // 运行帧数总数

        // 干扰源列表
        std::vector<JammerParam> jammers = {};
        // 多平台参数
        std::vector<InputPlatParam> platsparam = {};

        // 卫星参数
        SatelliteParam satellite;

        SimConfig& operator=(const SimConfig& other) {

            this->beta = other.beta;
            this->jammers = other.jammers;          // vector自动深拷贝
            this->satellite = other.satellite;
            this->platsparam = other.platsparam;
            return *this;
        }

        // 可选：添加默认构造/拷贝构造
        SimConfig() = default;
        SimConfig(const SimConfig&) = default;
    };

    // 仿真参数结构体
    struct SimParams {
        // 基础阈值
        double GDOP_threshold = 3.0;
        double power_ratio_threshold = 5.0;  // 功率比阈值(dB)
        double jammer_freq = 1561.09e6;
        // 干扰装备参数
        int jammer_num = 4;

        std::vector<DecJammerParam> jammers = {};
        std::vector<LLA> jammer_pos = {};

        //平台初始位置、速度
        LLA plat_initial_pos = { 0.0, 0.0, 0.0};
        LLA cur_plat_vec = { 0.0, 0.0, 0.0};  // 平台经纬高速度(°/km)

        // 欺骗点位置
        LLA deception_pos = { 115.32, 29.15, 0.5, 0 };
        LLA target_deception_pos = { 115.32, 29.15, 0.5, 0 };

        // 卫星参数(默认10颗)
        std::vector<LLA> satellite_pos = {
            {120.0, 30.0, 20000},
            {110.0, 35.0, 20000},
            {130.0, 25.0, 20000},
            {105.0, 28.0, 20000},
            {140.0, 32.0, 20000},
            {100.0, 22.0, 20000},
            {125.0, 40.0, 20000},
            {115.0, 20.0, 20000},
            {135.0, 27.0, 20000},
            {95.0, 33.0, 20000}
        };

        // 多平台参数
        std::vector<InputPlatParam> platsparam = {};

        SimParams& operator=(const SimParams& other) {

            this->jammer_freq = other.jammer_freq;
            this->plat_initial_pos = other.plat_initial_pos;
            this->cur_plat_vec = other.cur_plat_vec;
            this->deception_pos = other.deception_pos;
            this->target_deception_pos = other.target_deception_pos;
            this->jammer_pos = other.jammer_pos;
            this->jammers = other.jammers;
            this->platsparam = other.platsparam;
            return *this;
        }

        // 可选：添加默认构造/拷贝构造
        SimParams() = default;
        SimParams(const SimParams&) = default;
    };

    class PlatformData
    {
    public:
        PlatformData(UINT platform_id, LLA pos);

        inline void setPlatformId(UINT id) {
            platform_id_ = id;
        }
        inline UINT getPlatformId() {
            return platform_id_;
        }

        inline void setPlatformPos(LLA pos) {
            position_ = pos;
        }
        inline LLA getPlatformPos() {
            return position_;
        }


    private:
        UINT platform_id_;
        LLA position_;
    };

}

#endif
