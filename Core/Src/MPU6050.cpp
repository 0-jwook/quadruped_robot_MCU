#include "MPU6050.hpp"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// 생성자 — 상보 필터 초기 상태 설정
// ─────────────────────────────────────────────────────────────────────────────
MPU6050::MPU6050(I2C_HandleTypeDef* hi2c)
    : _hi2c(hi2c),
      _roll_cf(0.0f), _pitch_cf(0.0f),
      _roll_off(0.0f), _pitch_off(0.0f),
      _last_tick(0),  _filter_init(false)
{}

// ─────────────────────────────────────────────────────────────────────────────
// 초기화
// ─────────────────────────────────────────────────────────────────────────────
HAL_StatusTypeDef MPU6050::Init() {
    uint8_t check  = 0;
    uint8_t data   = 0x00;
    HAL_StatusTypeDef status = HAL_ERROR;

    // 센서 전원 안정화 대기
    HAL_Delay(50);

    // WHO_AM_I 확인 — 최대 5회 재시도 (I2C 버스가 아직 준비 중일 수 있음)
    for (int i = 0; i < 5; i++) {
        status = HAL_I2C_Mem_Read(_hi2c, ADDR, 0x75, 1, &check, 1, 100);
        if (status == HAL_OK &&
            (check == 0x68 || check == 0x70 || check == 0x71 || check == 0x72)) {
            break;
        }
        HAL_Delay(20);
    }

    // ── 레지스터 설정 ─────────────────────────────────────────
    // 가속도 풀스케일 ±2g (0x1C = 0x00, AFS_SEL=0)
    data = 0x00;
    HAL_I2C_Mem_Write(_hi2c, ADDR, 0x1C, 1, &data, 1, 100);

    // 자이로 풀스케일 ±250 °/s (0x1B = 0x00, FS_SEL=0)
    data = 0x00;
    HAL_I2C_Mem_Write(_hi2c, ADDR, 0x1B, 1, &data, 1, 100);

    // 디지털 저역통과 필터 (DLPF) — ~44 Hz (0x1A = 0x03)
    // 진동 노이즈를 줄여 상보 필터 성능 향상
    data = 0x03;
    HAL_I2C_Mem_Write(_hi2c, ADDR, 0x1A, 1, &data, 1, 100);

    // Sleep 모드 해제 (0x6B = 0x00)
    data = 0x00;
    HAL_I2C_Mem_Write(_hi2c, ADDR, 0x6B, 1, &data, 1, 100);

    HAL_Delay(50);  // 설정 반영 대기

    _last_tick   = HAL_GetTick();
    _filter_init = false;   // 첫 ReadData 호출 시 필터 재초기화

    return (status == HAL_OK && check != 0x00) ? HAL_OK : HAL_ERROR;
}

// ─────────────────────────────────────────────────────────────────────────────
// 데이터 읽기 + 상보 필터 적용
//
//   상보 필터 (Complementary Filter)
//   ─────────────────────────────────────────────────────────
//   가속도계 : 저주파(중력 방향)에 강인, 진동·충격에 약함
//   자이로   : 단기 변화에 강인, 시간이 지나면 드리프트 누적
//
//   filtered = α × (filtered + gyro × dt) + (1-α) × accel_angle
//              ─────────────────────────────   ──────────────────
//                  자이로 적분 (단기 신뢰)     가속도 보정 (장기)
//
//   α = CF_ALPHA = 0.96 → 자이로 96%, 가속도 4% 가중
//   호출 주기가 일정할수록 정확도가 높아짐 (권장 50 ms = 20 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void MPU6050::ReadData(float &roll, float &pitch, float &yaw) {
    // 가속도(6B) + 온도(2B) + 자이로(6B) = 14바이트 한 번에 읽기
    uint8_t data[14];

    if (HAL_I2C_Mem_Read(_hi2c, ADDR, 0x3B, 1, data, 14, 10) != HAL_OK) {
        // I2C 실패 시 마지막 유효 값 그대로 반환 (영점 보정 적용)
        roll  = _roll_cf  - _roll_off;
        pitch = _pitch_cf - _pitch_off;
        yaw   = 0.0f;
        return;
    }

    // ── 원시 데이터 변환 (빅엔디안, MSB first) ────────────────
    int16_t ax = (int16_t)(data[0]  << 8 | data[1]);
    int16_t ay = (int16_t)(data[2]  << 8 | data[3]);
    int16_t az = (int16_t)(data[4]  << 8 | data[5]);
    // data[6..7] : 온도 (사용 안 함)
    int16_t gx = (int16_t)(data[8]  << 8 | data[9]);   // Roll  축 각속도
    int16_t gy = (int16_t)(data[10] << 8 | data[11]);  // Pitch 축 각속도

    // ── 가속도 기반 Roll / Pitch (단위: 도) ───────────────────
    float roll_acc  = atan2f((float)ay, (float)az) * (180.0f / M_PI);
    float pitch_acc = atan2f(-(float)ax,
                             sqrtf((float)ay*(float)ay + (float)az*(float)az))
                      * (180.0f / M_PI);

    // ── 자이로 각속도 변환 ────────────────────────────────────
    // FS_SEL=0 → 1 LSB = 1/131 °/s
    float gx_dps = (float)gx / 131.0f;
    float gy_dps = (float)gy / 131.0f;

    // ── 경과 시간 dt 계산 ──────────────────────────────────────
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - _last_tick) / 1000.0f;  // ms → s
    _last_tick = now;

    // 첫 호출이거나 dt 가 비정상이면 가속도 값으로 필터 초기화
    if (!_filter_init || dt <= 0.0f || dt > 1.0f) {
        _roll_cf     = roll_acc;
        _pitch_cf    = pitch_acc;
        _filter_init = true;
    } else {
        // 상보 필터 적용
        _roll_cf  = CF_ALPHA * (_roll_cf  + gx_dps * dt)
                  + (1.0f - CF_ALPHA) * roll_acc;
        _pitch_cf = CF_ALPHA * (_pitch_cf + gy_dps * dt)
                  + (1.0f - CF_ALPHA) * pitch_acc;
    }

    // 영점 offset 적용 (평지 수평 = 0)
    roll  = _roll_cf  - _roll_off;
    pitch = _pitch_cf - _pitch_off;
    yaw   = 0.0f;  // 자력계 없이는 계산 불가
}

// ─────────────────────────────────────────────────────────────────────────────
// 영점 캘리브레이션
//   로봇이 평지에 수평으로 있을 때 1회 호출 (부팅 시).
//   상보 필터를 워밍업한 뒤 samples 만큼 평균내어 현재 자세를 영점으로 저장.
//   이후 ReadData 는 (필터값 - 영점) 을 반환 → 센서 bias·장착 기울기 상쇄.
// ─────────────────────────────────────────────────────────────────────────────
void MPU6050::CalibrateZero(uint16_t samples) {
    float r, p, y;

    // 기존 offset 제거 후 측정
    _roll_off  = 0.0f;
    _pitch_off = 0.0f;

    // 상보 필터 워밍업 (정착)
    for (int i = 0; i < 50; i++) {
        ReadData(r, p, y);
        HAL_Delay(5);
    }

    // 평균
    float rs = 0.0f, ps = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        ReadData(r, p, y);
        rs += r;
        ps += p;
        HAL_Delay(5);
    }

    _roll_off  = rs / (float)samples;
    _pitch_off = ps / (float)samples;
}
