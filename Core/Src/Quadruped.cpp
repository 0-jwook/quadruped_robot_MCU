#include "Quadruped.hpp"
#include <cstring>
#include <algorithm>

const uint8_t Quadruped::JOINT_CHANNELS[4][3] = {
    { 0,  1,  2},   // FL
    { 4,  5,  6},   // FR
    { 8,  9, 10},   // BL (RL)
    {12, 13, 14}    // BR (RR)
};

// 다리를 아래로 완전히 폈을 때의 서보 기준 각도
// 왼쪽(FL, BL): 종아리=180° / 오른쪽(FR, BR): 종아리=0°
const float Quadruped::HOME_ANGLES[4][3] = {
    {90.0f,   0.0f, 180.0f},  // FL (앞왼쪽):  허벅지 0°=수직하
    {95.0f, 168.0f,   0.0f},  // FR (앞오른쪽): 허벅지 169°
    {90.0f,   10.0f, 180.0f},  // BL (뒷왼쪽):  허벅지 0°=수직하
    {95.0f, 180.0f,   10.0f},  // BR (뒷오른쪽): 허벅지 180°=수직하 (좌우 대칭 반전)
};

// 앉은 자세 — HOME 에서 thigh 와 calf 를 각 60° 굽혀 무릎 접음.
// 다리가 몸체 아래로 접혀 들어가 로봇이 바닥에 안정적으로 내려앉음.
// (※ 실제 로봇에서 무릎이 부딪히면 ±10° 단위로 fine-tune 권장)
const float Quadruped::SIT_ANGLES[4][3] = {
    {90.0f,  60.0f, 120.0f},  // FL: thigh +60°, calf -60°
    {95.0f, 108.0f,  60.0f},  // FR: thigh -60° (반대), calf +60° (반대)
    {90.0f,  70.0f, 120.0f},  // BL: thigh +60°, calf -60°
    {95.0f, 120.0f,  70.0f},  // BR: thigh -60° (반대), calf +60° (반대)
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
