#ifndef __ROS_COM_HPP
#define __ROS_COM_HPP

#include "usart.h"
#include "PCA9685.hpp"
#include "Quadruped.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// 버퍼 및 상수 정의
// ─────────────────────────────────────────────────────────────────────────────
#define ROS_DMA_BUF_SIZE  128
#define ROS_RX_RING_SIZE  512
#define ROS_TX_SIZE       128

#pragma pack(push, 1)
/**
 * @brief 속도 기반 명령 구조체 (추후 GaitEngine 사용 시 활용)
 */
struct VelocityCmd {
    float v_x;
    float v_y;
    float v_yaw;
    float height;
    uint8_t gait_type;
};

/**
 * @brief 12개 관절 각도 명령 구조체 (현재 사용 중)
 * FL(H,T,C), FR(H,T,C), BL(H,T,C), BR(H,T,C) 순서
 */
struct JointAngleCmd {
    float angles[12];
};
#pragma pack(pop)

// ─────────────────────────────────────────────────────────────────────────────
// RosCom 클래스
// ─────────────────────────────────────────────────────────────────────────────
class RosCom {
public:
    enum ParseState {
        IDLE,
        HEADER_1,   // 0xAA
        HEADER_2,   // 0x55
        ID,
        LENGTH,
        PAYLOAD,
        CHECKSUM
    };

    RosCom(UART_HandleTypeDef* huart, PCA9685* pca, Quadruped* quad);

    void StartReceive();
    void OnRxEvent(uint16_t size);
    void Process();

    // 텔레메트리 전송
    void SendIMU(float roll, float pitch, float yaw);
    void SendHeartbeat();
    void SetDeviceStatus(bool pca_ok, bool imu_ok);

    // 최신 명령 조회
    bool GetJointCmd(JointAngleCmd& cmd);
    bool GetVelocityCmd(VelocityCmd& cmd); // 보행 엔진용

private:
    UART_HandleTypeDef* _huart;
    PCA9685*            _pca9685;
    Quadruped*          _quad;

    // DMA 및 링 버퍼
    uint8_t            _dma_buf[ROS_DMA_BUF_SIZE];
    volatile uint16_t  _dma_pos;
    volatile uint8_t   _ring_buf[ROS_RX_RING_SIZE];
    volatile uint16_t  _ring_head;
    volatile uint16_t  _ring_tail;

    // 바이너리 파서 상태 변수
    ParseState _state;
    uint8_t    _current_id;
    uint8_t    _payload_len;
    uint8_t    _payload_idx;
    uint8_t    _payload_buf[128];
    uint8_t    _checksum;

    // 최신 명령 저장소
    JointAngleCmd _last_joint_cmd;
    VelocityCmd   _last_vel_cmd;
    bool          _new_joint_available;
    uint32_t      _last_cmd_tick;

    // 장치 상태
    bool _pca_ok;
    bool _imu_ok;

    // 내부 헬퍼
    void HandleBinaryPacket(uint8_t id, uint8_t* payload, uint8_t len);
    void Send(const char* msg);
    void SendFmt(const char* fmt, ...);

    bool    RingEmpty() const;
    uint8_t RingRead();
};

extern RosCom* g_ros_com;

#ifdef __cplusplus
extern "C" {
#endif
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif

#endif /* __ROS_COM_HPP */
