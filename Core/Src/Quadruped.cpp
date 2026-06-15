#include "Quadruped.hpp"
#include <cstring>
#include <algorithm>

const uint8_t Quadruped::JOINT_CHANNELS[4][3] = {
    { 8,  9, 10},   // FL (앞 좌)
    {12, 13, 14},   // FR (앞 우)
    { 4,  5,  6},   // BL (RL, 뒤 좌)
    { 0,  1,  2}    // BR (RR, 뒤 우)
};

// 다리를 아래로 완전히 폈을 때의 서보 기준 각도
// 왼쪽(FL, BL): 종아리=180° / 오른쪽(FR, BR): 종아리=0°
const float Quadruped::HOME_ANGLES[4][3] = {
    {90.0f,   0.0f, 180.0f},  // FL (앞왼쪽):  허벅지 0°=수직하
    {95.0f, 168.0f,   0.0f},  // FR (앞오른쪽): 허벅지 169°
    {90.0f,   10.0f, 180.0f},  // BL (뒷왼쪽):  허벅지 0°=수직하
    {95.0f, 180.0f,   10.0f},  // BR (뒷오른쪽): 허벅지 180°=수직하 (좌우 대칭 반전)
};

// 앉은 자세 (부팅 시) — ROS body_height=0.085 stand 자세의 서보각과 일치시켜
// ROS 가 연결되어 ramp 를 시작할 때 이음매 없이 부드럽게 기립하도록 함.
// (ROS hardware_bridge IK + SERVO_TRIMS 결과 기준으로 산출)
const float Quadruped::SIT_ANGLES[4][3] = {
    {91.0f, 102.0f,  26.0f},  // FL
    {97.0f,  69.0f, 145.0f},  // FR
    {90.0f, 105.0f,  31.0f},  // BL (RL)
    {96.0f,  89.0f, 156.0f},  // BR (RR)
};

Quadruped::Quadruped(PCA9685* pca) : _pca(pca) {
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            _current_angles[leg][joint] = 90.0f;
            _configs[leg][joint].direction = 1;
            _configs[leg][joint].offset = 0.0f;
            _configs[leg][joint].min_limit = 0.0f;
            _configs[leg][joint].max_limit = 180.0f;
        }
    }
}

void Quadruped::ConfigureJoint(uint8_t leg_idx, uint8_t joint_idx, int8_t dir, float offset) {
    if (leg_idx >= 4 || joint_idx >= 3) return;
    _configs[leg_idx][joint_idx].direction = dir;
    _configs[leg_idx][joint_idx].offset = offset;
}

void Quadruped::SetJointAngle(uint8_t leg_idx, uint8_t joint_idx, float angle) {
    if (leg_idx >= 4 || joint_idx >= 3) return;

    // ROS2에서 계산된 최종 서보 각도를 아무런 가공 없이 그대로 출력합니다.
    // 이를 통해 파이썬의 보정 로직과 STM32의 로직이 충돌하는 것을 방지합니다.
    float final_angle = angle;

    if (final_angle < 0.0f)   final_angle = 0.0f;
    if (final_angle > 180.0f) final_angle = 180.0f;

    _current_angles[leg_idx][joint_idx] = final_angle;
    _pca->SetAngle(JOINT_CHANNELS[leg_idx][joint_idx], final_angle);
}

void Quadruped::SetLegAngle(uint8_t leg_idx, float hip, float thigh, float calf) {
    if (leg_idx >= 4) return;
    SetJointAngle(leg_idx, HIP,   hip);
    SetJointAngle(leg_idx, THIGH, thigh);
    SetJointAngle(leg_idx, CALF,  calf);
}

float Quadruped::GetJointAngle(uint8_t leg_idx, uint8_t joint_idx) const {
    if (leg_idx >= 4 || joint_idx >= 3) return -1.0f;
    return _current_angles[leg_idx][joint_idx];
}

void Quadruped::SetDefaultPose() {
    for (uint8_t leg = 0; leg < 4; leg++) {
        SetLegAngle(leg, HOME_ANGLES[leg][HIP], HOME_ANGLES[leg][THIGH], HOME_ANGLES[leg][CALF]);
    }
}

void Quadruped::SetSitPose() {
    for (uint8_t leg = 0; leg < 4; leg++) {
        SetLegAngle(leg, SIT_ANGLES[leg][HIP], SIT_ANGLES[leg][THIGH], SIT_ANGLES[leg][CALF]);
    }
}

bool Quadruped::ComputeIK(float x, float y, float z, float& h, float& t, float& c) {
    h = RadToDeg(atan2(y, x)) + 90.0f;
    float r_xy = sqrt(x*x + y*y);
    float r_effective = r_xy - L_COXA;
    float d = sqrt(r_effective * r_effective + z * z);
    if (d > (L_FEMUR + L_TIBIA) || d < fabsf(L_FEMUR - L_TIBIA)) return false;
    float a1 = atan2(z, r_effective);
    float a2 = acos((L_FEMUR*L_FEMUR + d*d - L_TIBIA*L_TIBIA) / (2 * L_FEMUR * d));
    t = RadToDeg(a1 + a2) + 90.0f;
    float a3 = acos((L_FEMUR*L_FEMUR + L_TIBIA*L_TIBIA - d*d) / (2 * L_FEMUR * L_TIBIA));
    c = RadToDeg(a3);
    if (std::isnan(h) || std::isnan(t) || std::isnan(c)) return false;
    return true;
}

bool Quadruped::SetLegPosition(uint8_t leg_idx, float x, float y, float z) {
    float h, t, c;
    if (ComputeIK(x, y, z, h, t, c)) {
        SetLegAngle(leg_idx, h, t, c);
        return true;
    }
    return false;
}
