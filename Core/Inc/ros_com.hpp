#ifndef __ROS_COM_HPP
#define __ROS_COM_HPP

#include "stm32f1xx_hal.h"
#include "PCA9685.hpp"
#include "Quadruped.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// 버퍼 및 상수 정의
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// RosCom 클래스 (USB CDC 버전)
// ─────────────────────────────────────────────────────────────────────────────
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

    RosCom(PCA9685* pca, Quadruped* quad);

    // USB CDC 수신 콜백에서 호출 (ISR 컨텍스트 안전)
    void FeedBytes(uint8_t* buf, uint32_t len);

    void Process();

    // 텔레메트리 전송
    void SendIMU(float roll, float pitch, float yaw);
    void SendHeartbeat();
    void SetDeviceStatus(bool pca_ok, bool imu_ok);

    // 최신 명령 조회
    bool GetJointCmd(JointAngleCmd& cmd);
    bool GetVelocityCmd(VelocityCmd& cmd);

private:
    PCA9685*  _pca9685;
    Quadruped* _quad;

    // 링 버퍼 (CDC 콜백 → 메인 루프)
    volatile uint8_t  _ring_buf[ROS_RX_RING_SIZE];
    volatile uint16_t _ring_head;
    volatile uint16_t _ring_tail;

    // 바이너리 파서
    ParseState _state;
    uint8_t    _current_id;
    uint8_t    _payload_len;
    uint8_t    _payload_idx;
    uint8_t    _payload_buf[128];
    uint8_t    _checksum;

    // 최신 명령
    JointAngleCmd _last_joint_cmd;
    VelocityCmd   _last_vel_cmd;
    bool          _new_joint_available;
    uint32_t      _last_cmd_tick;

    // 진단 카운터
    uint16_t _crc_err_count;
    uint16_t _usb_err_count;

    // 장치 상태
    bool _pca_ok;
    bool _imu_ok;

    void HandleBinaryPacket(uint8_t id, uint8_t* payload, uint8_t len);
    void Send(const char* msg);
    void SendFmt(const char* fmt, ...);

    bool    RingEmpty() const;
    uint8_t RingRead();
};

extern RosCom* g_ros_com;

// USB CDC 수신 콜백에서 호출하는 C 인터페이스
#ifdef __cplusplus
extern "C" {
#endif
    void RosCom_FeedBytes(uint8_t* buf, uint32_t len);
#ifdef __cplusplus
}
#endif

#endif /* __ROS_COM_HPP */
