#include "process/CalcParamRes.hpp"

namespace seven
{
    CalcParamManager &CalcParamManager::Ins()
    {
        static CalcParamManager instance;
        return instance;
    }

    CalcParamManager::CalcParamManager()
        : sim_state_(SimState::ENDDING), mMutex_(), vCalcParam_()
    {
        vServerPlatformData_.clear();
    }

    CalcParamManager::~CalcParamManager()
    {
    }

    SimState CalcParamManager::getSimSimState()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        return sim_state_;
    }

    void CalcParamManager::setSimSimState(SimState state)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        sim_state_ = state;
    }

    bool CalcParamManager::ClearAllData()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.run_frames_cnt = 0;
        vServerPlatformData_.clear();
        return true;
    }

    bool CalcParamManager::SetReturnFramesCount(UINT vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.return_frames = vRes;
        return true;
    }

    bool CalcParamManager::SetRunFramesCnt(UINT vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.run_frames_cnt = vRes;
        return true;
    }

    bool CalcParamManager::SetSimTime(UINT vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.sim_time_ = vRes;
        return true;
    }

    CalcParam CalcParamManager::GetCalcParam()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        return vCalcParam_;
    }

    void CalcParamManager::SwapPlatform(std::vector<InputPlatParam>& vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vServerPlatformData_.clear();
        vServerPlatformData_.swap(vRes);
    }

    void CalcParamManager::PushPlatform(InputPlatParam vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vServerPlatformData_.push_back(vRes);
    }

    std::vector<InputPlatParam>& CalcParamManager::GetPlatform()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        return vServerPlatformData_;
    }


    ContextManager& ContextManager::Ins()
    {
        static ContextManager instance;
        return instance;
    }

    ContextManager::ContextManager()
        : mMutex_(), m_formationParams()
    {
        initial_trajectory.clear();
        end_trajectory.clear();
    }

    ContextManager::~ContextManager()
    {
    }

    // ====================== m_formationParams 相关接口 ======================
    /**
     * @brief 设置完整的编队参数（覆盖式）
     * @param params 新的编队参数
     */
    void ContextManager::SetFormationParams(const FormationConfig& params) {
        std::unique_lock<std::mutex> lk(mMutex_); // 线程安全锁
        m_formationParams = params;
    }

    /**
     * @brief 获取编队参数（拷贝返回，避免外部修改内部数据）
     * @return 编队参数的拷贝
     */
    FormationConfig ContextManager::GetFormationParams() {
        std::unique_lock<std::mutex> lk(mMutex_);
        return m_formationParams;
    }

    /**
     * @brief 批量更新编队参数（增量式，只修改指定字段）
     * @param num_uavs 节点数量（-1表示不修改）
     * @param interval 间隔（负数表示不修改）
     * @param trans_formation 目标队形（可选）
     * 可根据需要扩展更多参数
     */
    //void ContextManager::UpdateFormationParams(int num_uavs = -1,
    //    double interval = -1.0,
    //    double collision_radius = 2,
    //    Formation_Type* trans_formation = nullptr,
    //    Point2D* pos_center = nullptr) {
    //    std::unique_lock<std::mutex> lk(mMutex_);
    //    if (num_uavs >= 0) m_formationParams.num_uavs = num_uavs;
    //    if (interval >= 0.0) m_formationParams.interval = interval;
    //    if (interval >= 0.0) m_formationParams.collision_radius = collision_radius;
    //    if (trans_formation != nullptr) m_formationParams.trans_formation = *trans_formation;
    //    if (pos_center != nullptr) m_formationParams.pos_center = *pos_center;
    //    // 可继续扩展其他字段的增量更新
    //}

    // ====================== initial_trajectory 相关接口 ======================
    /**
     * @brief 设置初始轨迹数据（覆盖式）
     * @param trajectory 新的轨迹数据
     */
    void ContextManager::SetInitialTrajectory(const std::vector<TrajectoryFrame>& trajectory) {
        std::unique_lock<std::mutex> lk(mMutex_);
        initial_trajectory = trajectory;
    }

    ///**
    // * @brief 移动语义设置初始轨迹（避免拷贝，提升性能）
    // * @param trajectory 新的轨迹数据（右值引用）
    // */
    //void ContextManager::SetInitialTrajectory(std::vector<TrajectoryFrame>&& trajectory) {
    //    std::lock_guard<std::mutex> lock(mMutex_);
    //    initial_trajectory = std::move(trajectory);
    //}

    /**
     * @brief 获取初始轨迹数据（拷贝返回）
     * @return 初始轨迹的拷贝
     */
    std::vector<TrajectoryFrame> ContextManager::GetInitialTrajectory(){
        std::unique_lock<std::mutex> lk(mMutex_);
        return initial_trajectory;
    }

    /**
     * @brief 追加初始轨迹数据（增量式）
     * @param frame 单帧轨迹数据
     */
    void ContextManager::AddInitialTrajectoryFrame(const TrajectoryFrame& frame) {
        std::unique_lock<std::mutex> lk(mMutex_);
        initial_trajectory.push_back(frame);
    }

    /**
     * @brief 清空初始轨迹数据
     */
    void ContextManager::ClearInitialTrajectory() {
        std::unique_lock<std::mutex> lk(mMutex_);
        initial_trajectory.clear();
    }

    // ====================== end_trajectory 相关接口 ======================
    /**
     * @brief 设置最终轨迹数据（覆盖式）
     * @param trajectory 新的轨迹数据
     */
    void ContextManager::SetEndTrajectory(const std::vector<TrajectoryFrame>& trajectory) {
        std::unique_lock<std::mutex> lk(mMutex_);
        initial_trajectory = trajectory; // 原代码笔误，修正为end_trajectory
        end_trajectory = trajectory;
    }

    /**
     * @brief 移动语义设置最终轨迹（避免拷贝）
     * @param trajectory 新的轨迹数据（右值引用）
     */
    /*void ContextManager::SetEndTrajectory(std::vector<TrajectoryFrame>&& trajectory) {
        std::lock_guard<std::mutex> lock(mMutex_);
        end_trajectory = std::move(trajectory);
    }*/

    /**
     * @brief 获取最终轨迹数据（拷贝返回）
     * @return 最终轨迹的拷贝
     */
    std::vector<TrajectoryFrame> ContextManager::GetEndTrajectory() {
        std::unique_lock<std::mutex> lk(mMutex_);
        return end_trajectory;
    }

    /**
     * @brief 追加最终轨迹数据（增量式）
     * @param frame 单帧轨迹数据
     */
    void ContextManager::AddEndTrajectoryFrame(const TrajectoryFrame& frame) {
        std::unique_lock<std::mutex> lk(mMutex_);
        end_trajectory.push_back(frame);
    }

    /**
     * @brief 清空最终轨迹数据
     */
    void ContextManager::ClearEndTrajectory() {
        std::unique_lock<std::mutex> lk(mMutex_);
        end_trajectory.clear();
    }

    void ContextManager::SetBarrageParams(const SimConfig& params)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        barrage_config = params;
    }

    SimConfig ContextManager::GetBarrageParams()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        return barrage_config;
    }

    void ContextManager::SetDeceptionParams(const SimParams& params)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        deception_config = params;
    }

    SimParams ContextManager::GetDeceptionParams()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        return deception_config;
    }
}