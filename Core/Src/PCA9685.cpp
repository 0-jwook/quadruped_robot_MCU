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
HAL_StatusTypeDef PCA9685::Init(bool start_outputs) {
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

    // 3. Auto-Increment 활성화. start_outputs 면 동시에 Sleep 해제(출력 ON).
    //    MODE1: RESTART=0, AI=1(0x20). SLEEP 비트(0x10)는 출력 보류 시 유지.
    //    start_outputs=true  → 0x20 (AI=1, SLEEP=0) : 즉시 출력
    //    start_outputs=false → 0x30 (AI=1, SLEEP=1) : 출력 보류 (WakeUp() 대기)
    cmd = start_outputs ? 0x20 : 0x30;
    status = HAL_I2C_Mem_Write(_hi2c, _addr, 0x00, 1, &cmd, 1, 10);
    if (status != HAL_OK) return status;
    HAL_Delay(5);

    return HAL_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// WakeUp — SLEEP 해제(출력 ON). Init(false) 로 적재한 펄스로 출력을 시작한다.
// ─────────────────────────────────────────────────────────────────────────────
HAL_StatusTypeDef PCA9685::WakeUp() {
    uint8_t cmd = 0x20;  // MODE1: AI=1, SLEEP=0
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(_hi2c, _addr, 0x00, 1, &cmd, 1, 10);
    HAL_Delay(1);        // 오실레이터 안정화 (>500us)
    return status;
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
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(_hi2c, _addr, reg, 1, buf, 4, 5);

    if (status != HAL_OK) {
        _err_count++;
        // STM32F1 I2C BUSY 플래그 버그 복구:
        //   서보 모터 전기 노이즈로 BUSY 플래그가 영구적으로 고착될 수 있음.
        //   CR1_SWRST 로 I2C 주변장치를 소프트 리셋한 뒤 PCA9685 재초기화.
        _hi2c->Instance->CR1 |= I2C_CR1_SWRST;
        HAL_Delay(1);
        _hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;
        HAL_I2C_Init(_hi2c);
        Init();   // PCA9685 레지스터 재설정 (50 Hz, Auto-Increment)
        status = HAL_I2C_Mem_Write(_hi2c, _addr, reg, 1, buf, 4, 5);
        if (status == HAL_OK) _err_count = 0;
    } else {
        _err_count = 0;
    }
    return status;
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
    return SetPWM(channel, 0, off);
}
