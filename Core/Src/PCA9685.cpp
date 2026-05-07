#include "PCA9685.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// 생성자 — 7비트 주소를 STM32 HAL 용 8비트로 변환
// ─────────────────────────────────────────────────────────────────────────────
PCA9685::PCA9685(I2C_HandleTypeDef* hi2c, uint8_t addr)
    : _hi2c(hi2c), _addr(addr << 1), _err_count(0)
{}

// ─────────────────────────────────────────────────────────────────────────────
// 초기화 — 50 Hz PWM, Auto-Increment 활성화
// ─────────────────────────────────────────────────────────────────────────────
HAL_StatusTypeDef PCA9685::Init() {
    HAL_StatusTypeDef status;

    // 0. 장치 응답 확인 (3회 재시도)
    status = HAL_I2C_IsDeviceReady(_hi2c, _addr, 3, 100);
    if (status != HAL_OK) return status;

    // 1. Sleep 모드 진입 — Prescaler 를 변경하려면 반드시 Sleep 상태여야 함
    uint8_t cmd = 0x10;  // MODE1: SLEEP=1
    status = HAL_I2C_Mem_Write(_hi2c, _addr, 0x00, 1, &cmd, 1, 10);
    if (status != HAL_OK) return status;
    HAL_Delay(5);

    // 2. Prescaler 설정 — 50 Hz
    //    prescale = round(osc_clock / (4096 × freq)) - 1
    //             = round(25 000 000 / (4096 × 50)) - 1 = 121 (0x79)
    uint8_t prescale = 121;
    status = HAL_I2C_Mem_Write(_hi2c, _addr, 0xFE, 1, &prescale, 1, 10);
    if (status != HAL_OK) return status;
    HAL_Delay(5);

    // 3. Sleep 해제 + Auto-Increment 활성화
    //    MODE1: RESTART=0, AI=1(0x20), SLEEP=0
    cmd = 0x20;
    status = HAL_I2C_Mem_Write(_hi2c, _addr, 0x00, 1, &cmd, 1, 10);
    if (status != HAL_OK) return status;
    HAL_Delay(5);

    return HAL_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// SetPWM — RAW 카운트 직접 제어 (캘리브레이션 / 디버그용)
//   · on  : PWM 이 HIGH 가 되는 카운트 (0~4095)
//   · off : PWM 이 LOW  가 되는 카운트 (0~4095)
//   · 서보 제어 시 on=0, off=펄스폭에 해당하는 카운트 사용
// ─────────────────────────────────────────────────────────────────────────────
HAL_StatusTypeDef PCA9685::SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) return HAL_ERROR;

    // LEDn_ON_L, LEDn_ON_H, LEDn_OFF_L, LEDn_OFF_H 4바이트 한 번에 전송
    uint8_t buf[4];
    buf[0] = (uint8_t)(on  & 0xFF);    // ON_L
    buf[1] = (uint8_t)(on  >> 8);      // ON_H
    buf[2] = (uint8_t)(off & 0xFF);    // OFF_L
    buf[3] = (uint8_t)(off >> 8);      // OFF_H

    uint8_t reg = 0x06 + (4 * channel);
    return HAL_I2C_Mem_Write(_hi2c, _addr, reg, 1, buf, 4, 100);
}

// ─────────────────────────────────────────────────────────────────────────────
// SetAngle — 각도(0~180°)를 PWM 카운트로 변환 후 서보 제어
//
//   PWM 매핑 (50 Hz, 4096 카운트):
//     0°   → 펄스폭 ~1.0 ms → off 카운트 ≈ 150
//     180° → 펄스폭 ~2.0 ms → off 카운트 ≈ 500
//     수식 : off = 150 + angle × (350 / 180)
// ─────────────────────────────────────────────────────────────────────────────
HAL_StatusTypeDef PCA9685::SetAngle(uint8_t channel, float angle) {
    if (channel > 15) return HAL_ERROR;

    // 각도 클램핑 (Quadruped 레벨에서도 하지만 직접 호출 경로를 위해 재검사)
    if (angle <   0.0f) angle =   0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint16_t off = (uint16_t)(150.0f + angle * (350.0f / 180.0f));
    HAL_StatusTypeDef status = SetPWM(channel, 0, off);

    if (status == HAL_OK) {
        // 성공 — 에러 카운터 초기화
        _err_count = 0;
    } else {
        // 실패 — 연속 에러 카운터 증가
        _err_count++;

        if (_err_count >= 3) {
            // 3회 연속 실패 : 외부 전원 재인가 / 전압 강하 / I2C 버스 오류로
            // PCA9685 가 Sleep 모드로 복귀했을 가능성 → 재초기화 시도
            HAL_Delay(5);                          // 전원 안정화 대기
            if (Init() == HAL_OK) {
                _err_count = 0;
                status = SetPWM(channel, 0, off);  // 재초기화 후 1회 재시도
            }
            // Init() 실패 시 _err_count 유지 — 다음 호출에서도 재시도 계속
        }
    }

    return status;
}
