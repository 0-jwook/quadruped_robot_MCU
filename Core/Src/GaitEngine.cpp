#include "GaitEngine.hpp"
#include <cmath>
#include <algorithm>

GaitEngine::GaitEngine(Quadruped* quad) 
    : _quad(quad), _phase(0.0f), _cycle_time(0.5f) // 0.5초당 한 걸음
{}

void GaitEngine::Update(const VelocityCmd& cmd, float dt) {
    // 1. 위상 업데이트 (속도가 0이면 정지)
    float speed = sqrt(cmd.v_x * cmd.v_x + cmd.v_y * cmd.v_y);
    if (speed > 0.01f || fabsf(cmd.v_yaw) > 0.01f) {
        _phase += dt / _cycle_time;
        if (_phase >= 1.0f) _phase -= 1.0f;
    } else {
        _phase = 0.0f; // 정지 자세
    }

    // 2. 각 다리별 궤적 계산 및 IK 적용
    for (uint8_t i = 0; i < 4; i++) {
        float x, y, z;
        float leg_phase = _phase;
        
        // Group B (FR, BL) 는 180도 위상차
        if (i == 1 || i == 2) {
            leg_phase += 0.5f;
            if (leg_phase >= 1.0f) leg_phase -= 1.0f;
        }

        ComputeLegTrajectory(i, cmd, leg_phase, x, y, z);
        _quad->SetLegPosition(i, x, y, z);
    }
}

void GaitEngine::ComputeLegTrajectory(uint8_t i, const VelocityCmd& cmd, float phase, float& x, float& y, float& z) {
    float step_len_x = cmd.v_x * _cycle_time;
    float step_len_y = cmd.v_y * _cycle_time;
    float swing_height = 30.0f; // 3cm 들어올림

    // 기본 위치 (힙 관절 기준 상대 좌표)
    // 실제 로봇 프레임에 맞춰 조정 필요. 여기서는 힙 아래 120mm 위치를 기본으로 함.
    float bx = 0; 
    float by = (i == 0 || i == 2) ? 40.0f : -40.0f; // 좌/우 오프셋
    float bz = cmd.height;

    if (phase < 0.5f) {
        // ① Stance Phase (지면 지지 및 몸체 추진)
        // 위상 0.0 -> 0.5 동안 발을 앞에서 뒤로 밀어냄 (+Step/2 -> -Step/2)
        float t = phase / 0.5f;
        x = bx + (step_len_x / 2.0f) * (1.0f - 2.0f * t);
        y = by + (step_len_y / 2.0f) * (1.0f - 2.0f * t);
        z = bz;
    } else {
        // ② Swing Phase (공중 이동)
        // 위상 0.5 -> 1.0 동안 발을 뒤에서 앞으로 옮김 (-Step/2 -> +Step/2)
        float t = (phase - 0.5f) / 0.5f;
        x = bx + (step_len_x / 2.0f) * (2.0f * t - 1.0f);
        y = by + (step_len_y / 2.0f) * (2.0f * t - 1.0f);
        
        // 포물선 궤적 (사인파)
        z = bz - swing_height * sinf(M_PI * t);
    }

    // Yaw 회전 보정 (간단화된 구현)
    if (fabsf(cmd.v_yaw) > 0.01f) {
        float angle = cmd.v_yaw * _cycle_time * (phase < 0.5f ? (0.5f - phase) : (phase - 0.75f));
        float rx = x * cosf(angle) - y * sinf(angle);
        float ry = x * sinf(angle) + y * cosf(angle);
        x = rx; y = ry;
    }
}
