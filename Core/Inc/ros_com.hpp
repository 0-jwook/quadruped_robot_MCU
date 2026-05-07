#ifndef __ROS_COM_HPP
#define __ROS_COM_HPP

#include "usart.h"
#include "PCA9685.hpp"
#include "Quadruped.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// 버퍼 크기 상수
// ─────────────────────────────────────────────────────────────────────────────
#define ROS_DMA_BUF_SIZE  128   // DMA 원형 수신 버퍼 — 하드웨어가 직접 쓴다
#define ROS_RX_RING_SIZE  256   // 소프트웨어 링 버퍼  — ISR 콜백이 채움, 메인루프가 비움
#define ROS_LINE_SIZE     128   // 라인 파싱 버퍼 — 메인루프 전용
#define ROS_TX_SIZE       128   // 송신 포맷 버퍼

// ─────────────────────────────────────────────────────────────────────────────
// C 링크 선언 — HAL 콜백은 C 심볼이어야 링커가 연결해줌
// ─────────────────────────────────────────────────────────────────────────────
#ifdef __cplusplus
extern "C" {
#endif
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// RosCom
//   STM32 ↔ ROS 시리얼 브릿지 클래스
//
//   수신 파이프라인 (이중 버퍼):
//     [DMA 원형 버퍼 _dma_buf] ──ISR 콜백──▶ [링 버퍼 _ring_buf] ──메인루프──▶ 파싱·실행
//
//   DMA가 하드웨어로 직접 수신하므로 I2C 블로킹 중에도 데이터가 보존된다.
//   IDLE 인터럽트로 패킷 경계(줄바꿈)를 즉시 감지, HT/TC로 버퍼 오버플로우 방지.
//
//   · 송신 : ACK / NACK / IMU 텔레메트리 / Heartbeat
// ─────────────────────────────────────────────────────────────────────────────
class RosCom {
public:
    RosCom(UART_HandleTypeDef* huart, PCA9685* pca, Quadruped* quad);

    // 부팅 시 주변장치 초기화 결과를 전달 — STATUS 명령 응답에 사용
    void SetDeviceStatus(bool pca_ok, bool imu_ok);

    // 부팅 시 1회 호출 — DMA 수신 시작 (_dma_pos 초기화 포함)
    void StartReceive();

    // ── ISR 전용 ─────────────────────────────────────────────
    // HAL_UARTEx_RxEventCallback(HT/TC/IDLE) 에서 위임받아
    // DMA 버퍼에서 새로 수신된 바이트를 링 버퍼로 복사한다.
    // size : HAL이 전달하는 DMA 쓰기 위치 (0..ROS_DMA_BUF_SIZE)
    void OnRxEvent(uint16_t size);

    // ── 메인 루프 전용 ────────────────────────────────────────
    // 링 버퍼를 소비해 라인 완성 시 파싱·실행
    void Process();

    // ── 텔레메트리 (main.cpp 에서 주기적으로 호출) ───────────
    void SendIMU(float roll, float pitch, float yaw);   // IMU 데이터 전송
    void SendHeartbeat();                               // 1 Hz 생존 신호

private:
    UART_HandleTypeDef* _huart;
    PCA9685*            _pca9685;
    Quadruped*          _quad;

    // ── DMA 수신 버퍼 (이중 버퍼 1단) ────────────────────────
    // DMA가 직접 쓰는 원형 버퍼. CPU 개입 없이 항상 수신 가능.
    uint8_t            _dma_buf[ROS_DMA_BUF_SIZE];
    volatile uint16_t  _dma_pos;   // 마지막으로 처리한 DMA 쓰기 위치

    // ── 소프트웨어 링 버퍼 (이중 버퍼 2단) ───────────────────
    // head : ISR 콜백이 증가 / tail : 메인루프가 증가
    // volatile 로 선언해 컴파일러 캐싱에 의한 레이스 컨디션 방지
    volatile uint8_t  _ring_buf[ROS_RX_RING_SIZE];
    volatile uint16_t _ring_head;
    volatile uint16_t _ring_tail;

    // ── 라인 파싱 버퍼 (메인루프 전용) ───────────────────────
    char     _line_buf[ROS_LINE_SIZE];
    uint16_t _line_idx;

    // ── 장치 상태 플래그 ──────────────────────────────────────
    bool _pca_ok;
    bool _imu_ok;

    // ── 내부 헬퍼 ────────────────────────────────────────────
    void    ProcessLine(const char* line);  // 완성된 한 줄 파싱·실행
    void    Send(const char* msg);          // 블로킹 UART 송신
    void    SendFmt(const char* fmt, ...);  // printf 스타일 송신
    void    SendACK();                      // "OK\n"
    void    SendNACK(const char* reason);   // "ERR:<reason>\n"

    bool    RingEmpty() const;   // 소프트웨어 링 버퍼가 비었는지 확인
    uint8_t RingRead();          // 소프트웨어 링 버퍼에서 1바이트 꺼냄
};

// 인터럽트 콜백에서 접근하기 위한 전역 포인터 (ros_com.cpp 에서 정의)
extern RosCom* g_ros_com;

#endif /* __ROS_COM_HPP */
