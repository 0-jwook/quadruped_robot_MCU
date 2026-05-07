#include "Quadruped.hpp"
#include <cstring>

// ─────────────────────────────────────────────────────────────────────────────
// static 멤버 정의 — ROM(Flash) 에 배치되며 인스턴스마다 복사되지 않음
// [leg_idx][joint_idx] : 0=Hip, 1=Thigh, 2=Calf
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t Quadruped::JOINT_CHANNELS[4][3] = {
    { 0,  1,  2},   // FL : Front Left
    { 4,  5,  6},   // FR : Front Right
    { 8,  9, 10},   // BL : Back Left
    {12, 13, 14}    // BR : Back Right
};

// ─────────────────────────────────────────────────────────────────────────────
// 생성자 — 현재 각도 캐시를 90° 중립으로 초기화
// ─────────────────────────────────────────────────────────────────────────────
Quadruped::Quadruped(PCA9685* pca) : _pca(pca) {
    for (int leg = 0; leg < 4; leg++)
        for (int joint = 0; joint < 3; joint++)
            _current_angles[leg][joint] = 90.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// 특정 다리의 3개 관절을 한 번에 설정
// ─────────────────────────────────────────────────────────────────────────────
void Quadruped::SetLegAngle(uint8_t leg_idx, float hip, float thigh, float calf) {
    if (leg_idx >= 4) return;
    SetJointAngle(leg_idx, HIP,   hip);
    SetJointAngle(leg_idx, THIGH, thigh);
    SetJointAngle(leg_idx, CALF,  calf);
}

// ─────────────────────────────────────────────────────────────────────────────
// 특정 단일 관절 설정
//   · 각도 클램핑 (0~180°) — PCA9685 레벨에서도 하지만 여기서 한 번 더 검사
//   · 캐시 업데이트 — GetJointAngle() 조회 시 I2C 읽기 없이 반환 가능
// ─────────────────────────────────────────────────────────────────────────────
void Quadruped::SetJointAngle(uint8_t leg_idx, uint8_t joint_idx, float angle) {
    if (leg_idx >= 4 || joint_idx >= 3) return;

    // 각도 범위 클램핑
    if (angle <   0.0f) angle =   0.0f;
    if (angle > 180.0f) angle = 180.0f;

    _current_angles[leg_idx][joint_idx] = angle;
    _pca->SetAngle(JOINT_CHANNELS[leg_idx][joint_idx], angle);
    // HAL_Delay 제거: 400kHz I2C 에서 1 트랜잭션 ≈ 200μs 이므로 HAL 내부
    // 타임아웃(100ms)으로도 충분. Delay(1)×12 = 12ms 누적 지연이 50Hz 주기(20ms)의
    // 절반 이상을 소모해 walk 명령 무응답의 원인이 됐음.
}

// ─────────────────────────────────────────────────────────────────────────────
// 현재 각도 조회 — 캐시에서 읽기 (I2C 통신 없음)
// ─────────────────────────────────────────────────────────────────────────────
float Quadruped::GetJointAngle(uint8_t leg_idx, uint8_t joint_idx) const {
    if (leg_idx >= 4 || joint_idx >= 3) return -1.0f;
    return _current_angles[leg_idx][joint_idx];
}

// ─────────────────────────────────────────────────────────────────────────────
// 기본 자세 — 모든 관절을 90° 중립으로 이동 (ESTOP / 초기화 시 사용)
// ─────────────────────────────────────────────────────────────────────────────
void Quadruped::SetDefaultPose() {
    for (uint8_t leg = 0; leg < 4; leg++) {
        SetLegAngle(leg, 90.0f, 90.0f, 90.0f);
    }
}
