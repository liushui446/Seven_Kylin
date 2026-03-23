#pragma once

#include <vector>
#include <string>
#include <map>
#include "core/CommonCore.hpp"

namespace seven
{
    
    class SEVEN_EXPORTS CalcParamManager
    {
    public:
        static CalcParamManager& Ins();

    private:
        CalcParamManager();
        ~CalcParamManager();

    public:
        SimState getSimSimState();
        void setSimSimState(SimState state);

        bool ClearAllData();

        bool SetReturnFramesCount(UINT vRes);
        bool SetRunFramesCnt(UINT vRes);
        bool SetSimTime(UINT vRes);
        CalcParam GetCalcParam();

        void SwapPlatform(std::vector<InputPlatParam>& vRes);
        void PushPlatform(InputPlatParam vRes);
        std::vector<InputPlatParam>& GetPlatform();

    private:
        SimState sim_state_;          // 仿真状态
        CalcParam vCalcParam_;
        std::vector<InputPlatParam> vServerPlatformData_;
        std::mutex mMutex_;

    public:
        CalcParamManager(const CalcParamManager&) = delete;
        CalcParamManager(CalcParamManager&&) = delete;
        CalcParamManager& operator=(const CalcParamManager&) = delete;
        CalcParamManager& operator=(CalcParamManager&&) = delete;
    };

    class SEVEN_EXPORTS ContextManager
    {
    public:
        static ContextManager& Ins();

    private:
        ContextManager();
        ~ContextManager();

    public:

        void SetFormationParams(const FormationConfig& params);

        FormationConfig GetFormationParams();

        //void UpdateFormationParams(int num_uavs, double interval, double collision_radius, Formation_Type* trans_formation, Point2D* pos_center);

        void SetInitialTrajectory(const std::vector<TrajectoryFrame>& trajectory);

        std::vector<TrajectoryFrame> GetInitialTrajectory();

        void AddInitialTrajectoryFrame(const TrajectoryFrame& frame);

        void ClearInitialTrajectory();

        void SetEndTrajectory(const std::vector<TrajectoryFrame>& trajectory);

        std::vector<TrajectoryFrame> GetEndTrajectory();

        void AddEndTrajectoryFrame(const TrajectoryFrame& frame);

        void ClearEndTrajectory();

        void SetBarrageParams(const SimConfig& params);
        SimConfig GetBarrageParams();

        void SetDeceptionParams(const SimParams& params);
        SimParams GetDeceptionParams();

    private:

        FormationConfig m_formationParams;
        vector<TrajectoryFrame> initial_trajectory;
        vector<TrajectoryFrame> end_trajectory;

        SimConfig barrage_config;  //barrage params
        SimParams deception_config;
        std::mutex mMutex_;

    public:
        ContextManager(const ContextManager&) = delete;
        ContextManager(ContextManager&&) = delete;
        ContextManager& operator=(const ContextManager&) = delete;
        ContextManager& operator=(ContextManager&&) = delete;
    };
}