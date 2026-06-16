#ifndef __ROS_COM_HPP
#define __ROS_COM_HPP

#include "usart.h"
#include "PCA9685.hpp"
#include "Quadruped.hpp"

#define ROS_DMA_BUF_SIZE  512
#define ROS_RX_RING_SIZE  512
#define ROS_TX_SIZE       128

#pragma pack(push, 1)
struct VelocityCmd {
    float v_x;
    float v_y;
    float v_yaw;
    float height;
    uint8_t gait_type;
};

struct JointAngleCmd {
    float angles[12];
};
#pragma pack(pop)

class RosCom {
public:
    enum ParseState {
        IDLE,
        HEADER_1,
        HEADER_2,
        ID,
        LENGTH,
        PAYLOAD,
        CHECKSUM
    };

    RosCom(UART_HandleTypeDef* huart, PCA9685* pca, Quadruped* quad);

    void StartReceive();
    void OnRxEvent(uint16_t size);
    void OnUartError();
    void Process();

    void SendIMU(float roll, float pitch, float yaw);
    void SendHeartbeat();
    void SetDeviceStatus(bool pca_ok, bool imu_ok);

    bool GetJointCmd(JointAngleCmd& cmd);
    bool GetVelocityCmd(VelocityCmd& cmd);

    // ID 0x04 (IMU 영점 명령) 수신 시 true 1회 반환 후 클리어.
    // 메인 루프가 이걸 보고 imu.CalibrateZero() 호출.
    bool ConsumeImuZeroRequest();

private:
    UART_HandleTypeDef* _huart;
    PCA9685*            _pca9685;
    Quadruped*          _quad;

    uint8_t            _dma_buf[ROS_DMA_BUF_SIZE];
    volatile uint16_t  _dma_pos;
    volatile uint8_t   _ring_buf[ROS_RX_RING_SIZE];
    volatile uint16_t  _ring_head;
    volatile uint16_t  _ring_tail;

    ParseState _state;
    uint8_t    _current_id;
    uint8_t    _payload_len;
    uint8_t    _payload_idx;
    uint8_t    _payload_buf[128];
    uint8_t    _checksum;

    JointAngleCmd _last_joint_cmd;
    VelocityCmd   _last_vel_cmd;
    bool          _new_joint_available;
    bool          _imu_zero_request;     // ID 0x04 수신 플래그
    bool          _cmd_received;         // 실제 joint 명령(ID 0x03)을 한 번이라도 받았는가
    uint32_t      _last_cmd_tick;

    uint16_t          _crc_err_count;
    uint16_t          _uart_err_count;
    uint16_t          _pkt_ok_count;
    uint16_t          _wdg_count;
    volatile uint32_t _last_rx_tick;

    bool _pca_ok;
    bool _imu_ok;

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
