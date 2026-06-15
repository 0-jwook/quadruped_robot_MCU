#include "ros_com.hpp"
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

RosCom::RosCom(UART_HandleTypeDef* huart, PCA9685* pca, Quadruped* quad)
    : _huart(huart), _pca9685(pca), _quad(quad),
      _dma_pos(0), _ring_head(0), _ring_tail(0),
      _state(HEADER_1), _current_id(0), _payload_len(0), _payload_idx(0),
      _checksum(0), _new_joint_available(false),
      _imu_zero_request(false),
      _last_cmd_tick(0),
      _crc_err_count(0), _uart_err_count(0),
      _pkt_ok_count(0), _wdg_count(0),
      _last_rx_tick(0),
      _pca_ok(false), _imu_ok(false)
{
    memset(_dma_buf, 0, sizeof(_dma_buf));
    memset((void*)_ring_buf, 0, sizeof(_ring_buf));
    memset(&_last_joint_cmd, 0, sizeof(_last_joint_cmd));
    memset(&_last_vel_cmd, 0, sizeof(_last_vel_cmd));
    _last_vel_cmd.height = 120.0f;

    for (int leg = 0; leg < 4; leg++)
        for (int joint = 0; joint < 3; joint++)
            _last_joint_cmd.angles[leg * 3 + joint] = Quadruped::HOME_ANGLES[leg][joint];

    g_ros_com = this;
}

void RosCom::StartReceive() {
    HAL_UART_AbortReceive(_huart);
    _dma_pos = 0;
    _state       = HEADER_1;
    _payload_idx = 0;
    _checksum    = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _dma_buf, ROS_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);  // HT 인터럽트 비활성화
    _last_rx_tick = HAL_GetTick();
}

void RosCom::OnRxEvent(uint16_t size) {
    bool is_tc = (size >= ROS_DMA_BUF_SIZE);
    uint16_t pos   = is_tc ? 0 : (size % ROS_DMA_BUF_SIZE);
    uint16_t count = is_tc ? (ROS_DMA_BUF_SIZE - _dma_pos)
                           : ((pos + ROS_DMA_BUF_SIZE - _dma_pos) % ROS_DMA_BUF_SIZE);

    for (uint16_t i = 0; i < count; i++) {
        uint16_t src  = (_dma_pos + i) % ROS_DMA_BUF_SIZE;
        uint16_t next = (_ring_head + 1) % ROS_RX_RING_SIZE;
        if (next != _ring_tail) {
            _ring_buf[_ring_head] = _dma_buf[src];
            _ring_head = next;
        }
    }
    if (count > 0)
        _last_rx_tick = HAL_GetTick();
    _dma_pos = pos;
}

void RosCom::OnUartError() {
    _uart_err_count++;
}

void RosCom::Process() {
    if (HAL_GetTick() - _last_rx_tick > 1000) {
        _wdg_count++;
        StartReceive();
    }

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
        _pkt_ok_count++;
    }
    else if (id == 0x04) {
        // IMU 영점 재캘리브 명령 (payload 없음). 메인 루프가 처리.
        _imu_zero_request = true;
        _pkt_ok_count++;
    }
}

bool RosCom::ConsumeImuZeroRequest() {
    if (_imu_zero_request) {
        _imu_zero_request = false;
        return true;
    }
    return false;
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
    uint8_t cmd_timeout = (HAL_GetTick() - _last_cmd_tick >= 2000) ? 1 : 0;
    SendFmt("HB:%lu,CRC:%u,ERR:%u,PKT:%u,WDG:%u,TO:%u\n",
            (unsigned long)HAL_GetTick(),
            (unsigned)_crc_err_count,
            (unsigned)_uart_err_count,
            (unsigned)_pkt_ok_count,
            (unsigned)_wdg_count,
            (unsigned)cmd_timeout);
    _crc_err_count  = 0;
    _uart_err_count = 0;
    _pkt_ok_count   = 0;
    _wdg_count      = 0;
}

void RosCom::SetDeviceStatus(bool pca_ok, bool imu_ok) {
    _pca_ok = pca_ok;
    _imu_ok = imu_ok;
}

void RosCom::Send(const char* msg) {
    HAL_UART_Transmit(_huart, (uint8_t*)msg, strlen(msg), 10);
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

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2 && g_ros_com != nullptr) {
        g_ros_com->OnRxEvent(Size);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && g_ros_com != nullptr) {
        g_ros_com->OnUartError();
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_AbortReceive(huart);
        g_ros_com->StartReceive();
    }
}
