#ifndef __MPU6050_HPP
#define __MPU6050_HPP

#include "stm32f1xx_hal.h"

// ─────────────────────────────────────────────────────────────────────────────
// MPU6050
//   6축 IMU (가속도계 + 자이로스코프) 드라이버
//   · 가속도 + 자이로를 함께 읽어 상보 필터(Complementary Filter)를 적용
//   · 단순 가속도 방식 대비 진동/충격에 강인한 Roll/Pitch 추정
//   · Yaw 는 자력계 없이는 불가 → 항상 0.0
// ─────────────────────────────────────────────────────────────────────────────
class MPU6050 {
public:
    explicit MPU6050(I2C_HandleTypeDef* hi2c);

    HAL_StatusTypeDef Init();

    // Roll / Pitch / Yaw (도, degree) 를 참조로 반환
    // 내부에서 상보 필터 적용 — 호출 주기가 일정할수록 정확도 높아짐
    void ReadData(float &roll, float &pitch, float &yaw);

    // 영점 캘리브레이션 — 로봇이 평지에 수평으로 있을 때 호출.
    // 현재 자세를 roll=pitch=0 기준으로 저장 (센서 bias + 장착 기울기 상쇄).
    // 부팅 시 1회 호출 권장. samples 만큼 평균.
    void CalibrateZero(uint16_t samples = 100);

private:
    I2C_HandleTypeDef* _hi2c;

    // I2C 7비트 주소 0x68 → HAL 용으로 1비트 좌측 시프트
    static const uint8_t ADDR = 0x68 << 1;

    // ── 상보 필터 상태 ────────────────────────────────────────
    // alpha 에 가까울수록 자이로 적분을 더 신뢰,
    // (1-alpha) 에 가까울수록 가속도계를 더 신뢰
    static constexpr float CF_ALPHA = 0.96f;

    float    _roll_cf;      // 필터링된 Roll  (도)
    float    _pitch_cf;     // 필터링된 Pitch (도)
    float    _roll_off;     // 영점 offset (도) — CalibrateZero 에서 설정
    float    _pitch_off;    // 영점 offset (도)
    uint32_t _last_tick;    // 마지막 호출 시각 (HAL_GetTick)
    bool     _filter_init;  // 첫 호출 여부 — 초기화에만 사용
};

#endif /* __MPU6050_HPP */
