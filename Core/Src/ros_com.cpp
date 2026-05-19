#include "ros_com.hpp"
#include "usbd_cdc_if.h"   // CubeMX 생성: CDC_Transmit_FS()
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>

static uint8_t CRC8Update(uint8_t crc, uint8_t byte) {
    crc ^= byte;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    return crc;
}

RosCom* g_ros_com = nullptr;

RosCom::RosCom(PCA9685* pca, Quadruped* quad)
    : _pca9685(pca), _quad(quad),
      _ring_head(0), _ring_tail(0),
      _state(HEADER_1), _current_id(0), _payload_len(0), _payload_idx(0),
      _checksum(0), _new_joint_available(false), _pca_ok(false), _imu_ok(false),
      _crc_err_count(0), _usb_err_count(0)
{
    memset((void*)_ring_buf, 0, sizeof(_ring_buf));
    memset(&_last_joint_cmd, 0, sizeof(_last_joint_cmd));
    memset(&_last_vel_cmd, 0, sizeof(_last_vel_cmd));
    _last_vel_cmd.height = 120.0f;

    for (int leg = 0; leg < 4; leg++)
        for (int joint = 0; joint < 3; joint++)
            _last_joint_cmd.angles[leg * 3 + joint] = Quadruped::HOME_ANGLES[leg][joint];

    _last_cmd_tick = 0;
    g_ros_com = this;
}

// ISR 안전: CDC_Receive_FS 콜백에서 호출됨
void RosCom::FeedBytes(uint8_t* buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (_ring_head + 1) % ROS_RX_RING_SIZE;
        if (next != _ring_tail) {
            _ring_buf[_ring_head] = buf[i];
            _ring_head = next;
        }
    }
}

void RosCom::Process() {
    while (!RingEmpty()) {
        uint8_t byte = RingRead();

        switch (_state) {
            case HEADER_1:
                if (byte == 0xAA) { _state = HEADER_2; _checksum = 0; }
                break;
            case HEADER_2:
                if (byte == 0x55) _state = ID;
                else _state = HEADER_1;
                break;
            case ID:
                _current_id = byte;
                _checksum = CRC8Update(_checksum, byte);
                _state = LENGTH;
                break;
            case LENGTH:
                _payload_len = byte;
                _checksum = CRC8Update(_checksum, byte);
                _payload_idx = 0;
                if (_payload_len > 0) _state = PAYLOAD;
                else _state = CHECKSUM;
                break;
            case PAYLOAD:
                if (_payload_idx < sizeof(_payload_buf)) {
                    _payload_buf[_payload_idx++] = byte;
                    _checksum = CRC8Update(_checksum, byte);
                }
                if (_payload_idx >= _payload_len) _state = CHECKSUM;
                break;
            case CHECKSUM:
                if (byte == _checksum)
                    HandleBinaryPacket(_current_id, _payload_buf, _payload_len);
                else
                    _crc_err_count++;
                _state = HEADER_1;
                break;
            default:
                _state = HEADER_1;
                break;
        }
    }
}

void RosCom::HandleBinaryPacket(uint8_t id, uint8_t* payload, uint8_t len) {
    if (id == 0x01 && len == sizeof(VelocityCmd)) {
        memcpy(&_last_vel_cmd, payload, sizeof(VelocityCmd));
        _last_cmd_tick = HAL_GetTick();
    }
    else if (id == 0x03 && len == sizeof(JointAngleCmd)) {
        memcpy(&_last_joint_cmd, payload, sizeof(JointAngleCmd));
        _new_joint_available = true;
        _last_cmd_tick = HAL_GetTick();
    }
}

bool RosCom::GetJointCmd(JointAngleCmd& cmd) {
    cmd = _last_joint_cmd;
    return (HAL_GetTick() - _last_cmd_tick < 2000);
}

bool RosCom::GetVelocityCmd(VelocityCmd& cmd) {
    cmd = _last_vel_cmd;
    return (HAL_GetTick() - _last_cmd_tick < 2000);
}

void RosCom::SendIMU(float roll, float pitch, float yaw) {
    SendFmt("IMU:%.2f,%.2f,%.2f\n", roll, pitch, yaw);
}

void RosCom::SendHeartbeat() {
    SendFmt("HB:%lu,CRC:%u,ERR:%u\n",
            (unsigned long)HAL_GetTick(),
            (unsigned)_crc_err_count,
            (unsigned)_usb_err_count);
    _crc_err_count = 0;
    _usb_err_count = 0;
}

void RosCom::SetDeviceStatus(bool pca_ok, bool imu_ok) {
    _pca_ok = pca_ok;
    _imu_ok = imu_ok;
}

void RosCom::Send(const char* msg) {
    uint16_t len = (uint16_t)strlen(msg);
    // CDC_Transmit_FS는 내부 버퍼를 사용하므로 busy 시 잠시 대기
    uint32_t t = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t*)msg, len) == USBD_BUSY) {
        if (HAL_GetTick() - t > 5) { _usb_err_count++; return; }
    }
}

void RosCom::SendFmt(const char* fmt, ...) {
    char buf[ROS_TX_SIZE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Send(buf);
}

bool RosCom::RingEmpty() const { return _ring_head == _ring_tail; }
uint8_t RosCom::RingRead() {
    uint8_t val = _ring_buf[_ring_tail];
    _ring_tail = (_ring_tail + 1) % ROS_RX_RING_SIZE;
    return val;
}

// usbd_cdc_if.c의 CDC_Receive_FS에서 호출하는 C 인터페이스
extern "C" void RosCom_FeedBytes(uint8_t* buf, uint32_t len) {
    if (g_ros_com) g_ros_com->FeedBytes(buf, len);
}
