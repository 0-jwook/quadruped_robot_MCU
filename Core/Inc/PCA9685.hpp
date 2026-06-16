#ifndef __PCA9685_HPP
#define __PCA9685_HPP

#include "i2c.h"

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685
//   I2C 기반 16채널 PWM 드라이버 클래스
//   · 50 Hz 설정으로 서보 모터 제어에 사용
//   · SetAngle : 각도(0-180°)를 PWM 카운트로 변환하여 전송
//   · SetPWM   : RAW 카운트 직접 제어 (디버그 / 세밀 조정용)
// ─────────────────────────────────────────────────────────────────────────────
class PCA9685 {
public:
    // addr : 7비트 I2C 주소 (기본값 0x40), 내부에서 HAL 용 8비트로 변환
    explicit PCA9685(I2C_HandleTypeDef* hi2c, uint8_t addr = 0x40);

    // PCA9685 초기화 — 50 Hz PWM, Auto-Increment 활성화
    //   start_outputs=true  : SLEEP 해제까지 수행(출력 즉시 ON, 런타임/복구용 기본)
    //   start_outputs=false : SLEEP 유지(출력 OFF). 부팅 시 SIT 값을 먼저 적재한 뒤
    //                         WakeUp() 으로 켜기 위함 — 이전 자세가 잠깐 출력되는 것 방지.
    HAL_StatusTypeDef Init(bool start_outputs = true);

    // SLEEP 해제 — 출력 ON. Init(false) 로 적재한 값으로 출력을 시작한다.
    HAL_StatusTypeDef WakeUp();

    // 각도로 서보 제어 (0.0 ~ 180.0 도)
    HAL_StatusTypeDef SetAngle(uint8_t channel, float angle);

    // RAW PWM 카운트 직접 설정 (on: 0~4095, off: 0~4095)
    HAL_StatusTypeDef SetPWM(uint8_t channel, uint16_t on, uint16_t off);

private:
    I2C_HandleTypeDef* _hi2c;
    uint8_t            _addr;      // 8비트 주소 (7비트 << 1)
    uint8_t            _err_count; // 연속 I2C 실패 횟수 — 3회 초과 시 자동 재초기화
};

#endif /* __PCA9685_HPP */
