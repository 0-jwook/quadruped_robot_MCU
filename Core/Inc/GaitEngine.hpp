#ifndef __GAIT_ENGINE_HPP
#define __GAIT_ENGINE_HPP

#include "Quadruped.hpp"
#include "ros_com.hpp"

class GaitEngine {
public:
    GaitEngine(Quadruped* quad);

    /**
     * @brief 보행 엔진 업데이트 (50Hz 고정 주기 호출 권장)
     * @param cmd ROS로부터 받은 속도 명령
     * @param dt 지난 시간 (초)
     */
    void Update(const VelocityCmd& cmd, float dt);

private:
    Quadruped* _quad;
    float      _phase;        // 현재 보행 위상 (0.0 ~ 1.0)
    float      _cycle_time;   // 한 걸음 주기 (초)

    // 각 다리의 기본 중립 위치 (mm)
    static constexpr float BASE_X[4] = { 80,  80, -80, -80};
    static constexpr float BASE_Y[4] = { 60, -60,  60, -60};

    /**
     * @brief 특정 다리의 발끝 위치 계산
     */
    void ComputeLegTrajectory(uint8_t leg_idx, const VelocityCmd& cmd, float phase, float& x, float& y, float& z);
};

#endif /* __GAIT_ENGINE_HPP */
