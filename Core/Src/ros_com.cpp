#include "ros_com.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>

// CRC-8 (polynomial 0x07, init 0x00) — 단순 합산보다 오류 검출률 대폭 향상
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
      _checksum(0), _new_joint_available(false), _pca_ok(false), _imu_ok(false)
{
    memset(_dma_buf, 0, sizeof(_dma_buf));
    memset((void*)_ring_buf, 0, sizeof(_ring_buf));
    memset(&_last_joint_cmd, 0, sizeof(_last_joint_cmd));
    memset(&_last_vel_cmd, 0, sizeof(_last_vel_cmd));
    _last_vel_cmd.height = 120.0f; // 기본 높이
    
    // 초기값을 조립 기준 홈 포지션으로 설정 (FL/BL 종아리=180, FR/BR 종아리=0)
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            _last_joint_cmd.angles[leg * 3 + joint] = Quadruped::HOME_ANGLES[leg][joint];
        }
    }
    _last_cmd_tick = 0;
    g_ros_com = this;
}

void RosCom::StartReceive() {
    // 에러 복구: HAL 상태 강제 초기화 후 재시작
    HAL_UART_AbortReceive(_huart);
    _dma_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _dma_buf, ROS_DMA_BUF_SIZE);
}

void RosCom::RestartReceive() {
    // TC 이벤트 후 재시작: DMA가 이미 완료된 상태이므로 Abort 불필요
    // AbortReceive 생략으로 재시작 간격(바이트 유실 구간)을 최소화
    _dma_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _dma_buf, ROS_DMA_BUF_SIZE);
}

void RosCom::OnRxEvent(uint16_t size) {
    // TC 이벤트(버퍼 가득): size==ROS_DMA_BUF_SIZE → pos=0
    // 이 때 (0+256-_dma_pos)%256 수식은 _dma_pos==0이면 count=0이 되어
    // 버퍼 전체(256바이트)를 유실하는 버그 → TC 여부를 분기해서 처리
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
    _dma_pos = pos;
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
                if (byte == _checksum) {
                    HandleBinaryPacket(_current_id, _payload_buf, _payload_len);
                }
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
    return (HAL_GetTick() - _last_cmd_tick < 500);
}

bool RosCom::GetVelocityCmd(VelocityCmd& cmd) {
    cmd = _last_vel_cmd;
    return (HAL_GetTick() - _last_cmd_tick < 500);
}

void RosCom::SendIMU(float roll, float pitch, float yaw) {
    SendFmt("IMU:%.2f,%.2f,%.2f\n", roll, pitch, yaw);
}

void RosCom::SendHeartbeat() {
    SendFmt("HB:%lu\n", (unsigned long)HAL_GetTick());
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
        // TC 이벤트(버퍼 가득 참): DMA 이미 완료 → Abort 없이 즉시 재시작
        if (Size >= ROS_DMA_BUF_SIZE) {
            g_ros_com->RestartReceive();
        }
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && g_ros_com != nullptr) {
        HAL_UART_AbortReceive(huart);
        g_ros_com->StartReceive();
    }
}
