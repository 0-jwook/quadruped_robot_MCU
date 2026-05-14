#ifndef __QUADRUPED_HPP
#define __QUADRUPED_HPP

#include "PCA9685.hpp"
#include <cmath>

/**
 * @brief 4족보행 로봇 제어 클래스
 * 
 * 구조 개선:
 * 1. 역기하학(IK) 엔진 내장
 * 2. 서보 캘리브레이션 (방향, 오프셋) 관리
 * 3. 기하학적 파라미터 정의
 */
class Quadruped {
public:
    // 다리 인덱스
    enum LegIndex { FL = 0, FR = 1, BL = 2, BR = 3 };
    // 관절 인덱스
    enum JointIndex { HIP = 0, THIGH = 1, CALF = 2 };

    // 다리 기하학 상수 (mm)
    static constexpr float L_COXA  = 42.0f;
    static constexpr float L_FEMUR = 75.0f;
    static constexpr float L_TIBIA = 95.0f;

    // 다리를 아래로 완전히 폈을 때의 서보 기준 각도 [leg][joint]
    // FL/BL(왼쪽): 종아리 서보 물리 방향이 반대라 직선=180°
    // FR/BR(오른쪽): 종아리 서보 물리 방향이 반대라 직선=0°
    static const float HOME_ANGLES[4][3];

    struct JointConfig {
        int8_t  direction; // 1 또는 -1 (방향 반전)
        float   offset;    // 90도 기준 오프셋 (degree)
        float   min_limit; // 최소 안전 각도
        float   max_limit; // 최대 안전 각도
    };

    Quadruped(PCA9685* pca);

    // ── 저수준 제어 ─────────────────────────────────────────────
    void SetJointAngle(uint8_t leg_idx, uint8_t joint_idx, float angle);
    void SetLegAngle(uint8_t leg_idx, float hip, float thigh, float calf);
    float GetJointAngle(uint8_t leg_idx, uint8_t joint_idx) const;
    void SetDefaultPose();

    // ── 역기하학 (Inverse Kinematics) ───────────────────────────
    /**
     * @brief 발끝 좌표(x,y,z)를 관절 각도(h,t,c)로 변환
     * @param x, y, z : 힙 관절 기준 좌표 (mm)
     * @param h, t, c : 계산된 결과 각도 (degree)
     * @return true 면 성공, false 면 도달 불가능한 좌표
     */
    bool ComputeIK(float x, float y, float z, float& h, float& t, float& c);

    // 좌표를 기반으로 다리 위치 설정
    bool SetLegPosition(uint8_t leg_idx, float x, float y, float z);

    // 서보 캘리브레이션 설정
    void ConfigureJoint(uint8_t leg_idx, uint8_t joint_idx, int8_t dir, float offset);

private:
    PCA9685* _pca;
    float    _current_angles[4][3];
    JointConfig _configs[4][3];

    // PCA9685 채널 맵핑
    static const uint8_t JOINT_CHANNELS[4][3];

    // 수학 헬퍼
    static float RadToDeg(float rad) { return rad * 180.0f / M_PI; }
    static float DegToRad(float deg) { return deg * M_PI / 180.0f; }
};

#endif /* __QUADRUPED_HPP */
