#ifndef __QUADRUPED_HPP
#define __QUADRUPED_HPP

#include "PCA9685.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Quadruped
//   4족보행 로봇의 다리/관절 추상화 클래스
//   · 채널 매핑 : static const 로 ROM 에 배치
//   · 현재 각도 캐시 : 외부(ROS)에서 상태 조회 가능
// ─────────────────────────────────────────────────────────────────────────────
class Quadruped {
public:
    // 다리 인덱스 별칭 (가독성 향상)
    enum Leg  : uint8_t { FL = 0, FR = 1, BL = 2, BR = 3 };
    enum Joint: uint8_t { HIP = 0, THIGH = 1, CALF = 2 };

    explicit Quadruped(PCA9685* pca);

    // 특정 다리의 3개 관절을 한 번에 설정
    void SetLegAngle(uint8_t leg_idx, float hip, float thigh, float calf);

    // 특정 단일 관절 설정
    void SetJointAngle(uint8_t leg_idx, uint8_t joint_idx, float angle);

    // 현재 각도 조회 (ROS STATUS 응답, 안전장치 로직 등에서 사용)
    float GetJointAngle(uint8_t leg_idx, uint8_t joint_idx) const;

    // 모든 관절을 90° 기본 자세로 이동
    void SetDefaultPose();

private:
    PCA9685* _pca;

    // PCA9685 채널 매핑 — static const 로 선언해 ROM 에 배치
    // 인스턴스마다 복사되지 않음
    static const uint8_t JOINT_CHANNELS[4][3];

    // 현재 관절 각도 캐시 [leg][joint]
    float _current_angles[4][3];
};

#endif /* __QUADRUPED_HPP */
