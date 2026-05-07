#include "ros_com.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>

// ─────────────────────────────────────────────────────────────────────────────
// 전역 포인터 — HAL 콜백(C 함수)에서 C++ 객체로 접근하기 위함
// ─────────────────────────────────────────────────────────────────────────────
RosCom* g_ros_com = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// 생성자
// ─────────────────────────────────────────────────────────────────────────────
RosCom::RosCom(UART_HandleTypeDef* huart, PCA9685* pca, Quadruped* quad)
    : _huart(huart), _pca9685(pca), _quad(quad),
      _dma_pos(0),
      _ring_head(0), _ring_tail(0),
      _line_idx(0),
      _pca_ok(false), _imu_ok(false)
{
    memset(_dma_buf,          0, sizeof(_dma_buf));
    memset((void*)_ring_buf,  0, sizeof(_ring_buf));
    memset(_line_buf,         0, sizeof(_line_buf));
    g_ros_com = this;
}

// ─────────────────────────────────────────────────────────────────────────────
// 장치 상태 설정 — main.cpp 에서 Init() 결과를 전달, STATUS 응답에 사용
// ─────────────────────────────────────────────────────────────────────────────
void RosCom::SetDeviceStatus(bool pca_ok, bool imu_ok) {
    _pca_ok = pca_ok;
    _imu_ok = imu_ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// DMA 수신 시작 (부팅 시 1회, 에러 복구 시 재호출)
//
// HAL_UARTEx_ReceiveToIdle_DMA 는 DMA를 순환 모드로 구동하면서
// HT(반충전) / TC(완충전) / IDLE(회선 무음) 세 가지 이벤트로
// HAL_UARTEx_RxEventCallback 을 호출한다.
//   · HT  : _dma_buf[0..63]  수신 완료
//   · TC  : _dma_buf[64..127] 수신 완료 → DMA 위치 0으로 복귀
//   · IDLE: 그 외 임의 위치에서 회선 무음 감지 (패킷 경계 감지)
// ─────────────────────────────────────────────────────────────────────────────
void RosCom::StartReceive() {
    _dma_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _dma_buf, ROS_DMA_BUF_SIZE);
}

// ─────────────────────────────────────────────────────────────────────────────
// ISR 전용 : DMA 버퍼의 새 데이터를 소프트웨어 링 버퍼로 복사
//
// size 파라미터 (HAL 이 전달하는 DMA 쓰기 절대 위치):
//   HT 이벤트  → size = ROS_DMA_BUF_SIZE / 2  (= 64)
//   TC 이벤트  → size = ROS_DMA_BUF_SIZE       (= 128, 랩어라운드)
//   IDLE 이벤트 → size = 0..ROS_DMA_BUF_SIZE-1 (현재 쓰기 위치)
//
// size % ROS_DMA_BUF_SIZE 로 정규화하면 TC(128→0)를 자연스럽게 처리한다.
// HT 이벤트가 활성화되어 있으므로 count 는 항상 0..64 범위로 제한되어
// pos == _dma_pos 의 "빈 버퍼 vs 가득 찬 버퍼" 모호성이 발생하지 않는다.
// ─────────────────────────────────────────────────────────────────────────────
void RosCom::OnRxEvent(uint16_t size) {
    uint16_t pos   = size % ROS_DMA_BUF_SIZE;
    uint16_t count = (pos + ROS_DMA_BUF_SIZE - _dma_pos) % ROS_DMA_BUF_SIZE;

    for (uint16_t i = 0; i < count; i++) {
        uint16_t src  = (_dma_pos + i) % ROS_DMA_BUF_SIZE;
        uint16_t next = (_ring_head + 1) % ROS_RX_RING_SIZE;
        if (next != _ring_tail) {   // 링 버퍼 가득 찼을 때는 드롭 (오버플로우 방지)
            _ring_buf[_ring_head] = _dma_buf[src];
            _ring_head = next;
        }
    }

    _dma_pos = pos;
}

// ─────────────────────────────────────────────────────────────────────────────
// 소프트웨어 링 버퍼 헬퍼
// ─────────────────────────────────────────────────────────────────────────────
bool RosCom::RingEmpty() const {
    return _ring_head == _ring_tail;
}

uint8_t RosCom::RingRead() {
    uint8_t val = _ring_buf[_ring_tail];
    _ring_tail = (_ring_tail + 1) % ROS_RX_RING_SIZE;
    return val;
}

// ─────────────────────────────────────────────────────────────────────────────
// 메인 루프 처리
//   링 버퍼에서 바이트를 꺼내 라인 버퍼에 쌓고,
//   개행(\n or \r) 수신 시 ProcessLine() 호출
// ─────────────────────────────────────────────────────────────────────────────
void RosCom::Process() {
    while (!RingEmpty()) {
        char byte = (char)RingRead();

        if (byte == '\n' || byte == '\r') {
            if (_line_idx > 0) {
                _line_buf[_line_idx] = '\0';
                ProcessLine(_line_buf);
                _line_idx = 0;
                // 명령 1개 처리 후 즉시 반환.
                // 이유: A: 명령 1회는 I2C ×12 ≈ 2.7ms 블로킹.
                //       while 루프로 한꺼번에 처리하면 버퍼에 쌓인 N개 명령 ×2.7ms
                //       동안 메인 루프가 멈춰 다음 명령이 50ms 이상 늦어진다.
                //       1개만 처리하고 나머지는 다음 loop() 에서 처리하면
                //       IMU·Heartbeat·LED 타이밍을 지킬 수 있다.
                return;
            }
        } else if (_line_idx < ROS_LINE_SIZE - 1) {
            _line_buf[_line_idx++] = byte;
        } else {
            // 라인 버퍼 초과 — 현재 라인 폐기 후 에러 응답
            _line_idx = 0;
            SendNACK("BUF_OVERFLOW");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 명령 파싱 & 실행
// ─────────────────────────────────────────────────────────────────────────────
void RosCom::ProcessLine(const char* line) {
    int   leg_idx, joint_idx, channel;
    float angle, hip, thigh, calf;
    float a[12];

    // ── PING ─────────────────────────────────────────────────────────────────
    // ROS 연결 확인 및 왕복 지연 측정용
    // 요청 : "PING\n"   응답 : "PONG\n"
    if (strncmp(line, "PING", 4) == 0) {
        Send("PONG\n");
        return;
    }

    // ── ESTOP (긴급 정지) ─────────────────────────────────────────────────────
    // 모든 관절을 즉시 90° 중립 위치로 복귀
    // 요청 : "ESTOP\n"  응답 : "ESTOP_OK\n"
    if (strncmp(line, "ESTOP", 5) == 0) {
        _quad->SetDefaultPose();
        Send("ESTOP_OK\n");
        return;
    }

    // ── STATUS ────────────────────────────────────────────────────────────────
    // 장치 초기화 상태 및 업타임 반환
    // 요청 : "STATUS\n"
    // 응답 : "STATUS:pca=1,imu=1,tick=12345\n"
    if (strncmp(line, "STATUS", 6) == 0) {
        SendFmt("STATUS:pca=%d,imu=%d,tick=%lu\n",
                (int)_pca_ok,
                (int)_imu_ok,
                (unsigned long)HAL_GetTick());
        return;
    }

    // ── 전체 다리 제어 : A:h0,t0,c0,h1,t1,c1,h2,t2,c2,h3,t3,c3 ─────────────
    // 12개 값을 한 번에 전송해 모든 다리를 동기화할 때 사용
    if (sscanf(line, "A:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
               &a[0],  &a[1],  &a[2],
               &a[3],  &a[4],  &a[5],
               &a[6],  &a[7],  &a[8],
               &a[9],  &a[10], &a[11]) == 12)
    {
        for (int i = 0; i < 4; i++) {
            _quad->SetLegAngle((uint8_t)i, a[i*3], a[i*3+1], a[i*3+2]);
        }
        SendACK();
        return;
    }

    // ── 단일 다리 제어 : L<0-3>:hip,thigh,calf ───────────────────────────────
    // 예) "L0:90,45,90\n" → 앞왼쪽(FL) 다리 이동
    if (sscanf(line, "L%d:%f,%f,%f", &leg_idx, &hip, &thigh, &calf) == 4) {
        if (leg_idx < 0 || leg_idx > 3) { SendNACK("BAD_LEG"); return; }
        _quad->SetLegAngle((uint8_t)leg_idx, hip, thigh, calf);
        SendACK();
        return;
    }

    // ── 단일 관절 제어 : J<leg>,<joint>:angle ────────────────────────────────
    // joint : 0=Hip, 1=Thigh, 2=Calf
    // 예) "J1,1:60.0\n" → 앞오른쪽(FR) Thigh 를 60° 로
    if (sscanf(line, "J%d,%d:%f", &leg_idx, &joint_idx, &angle) == 3) {
        if (leg_idx < 0 || leg_idx > 3)     { SendNACK("BAD_LEG");   return; }
        if (joint_idx < 0 || joint_idx > 2) { SendNACK("BAD_JOINT"); return; }
        _quad->SetJointAngle((uint8_t)leg_idx, (uint8_t)joint_idx, angle);
        SendACK();
        return;
    }

    // ── 직접 채널 제어 (디버그) : C<0-15>:angle ──────────────────────────────
    // PCA9685 채널을 직접 제어 — 서보 캘리브레이션, 하드웨어 테스트에 사용
    if (sscanf(line, "C%d:%f", &channel, &angle) == 2) {
        if (channel < 0 || channel > 15) { SendNACK("BAD_CH"); return; }
        HAL_StatusTypeDef st = _pca9685->SetAngle((uint8_t)channel, angle);
        if (st == HAL_OK) SendACK();
        else              SendNACK("I2C_ERR");
        return;
    }

    // ── 알 수 없는 명령 ───────────────────────────────────────────────────────
    SendNACK("UNKNOWN_CMD");
}

// ─────────────────────────────────────────────────────────────────────────────
// 텔레메트리 송신
// ─────────────────────────────────────────────────────────────────────────────
void RosCom::SendIMU(float roll, float pitch, float yaw) {
    SendFmt("IMU:%.2f,%.2f,%.2f\n", roll, pitch, yaw);
}

// 생존 신호 (1 Hz) — ROS 가 수신 타임아웃으로 연결 상태를 감지
void RosCom::SendHeartbeat() {
    SendFmt("HB:%lu\n", (unsigned long)HAL_GetTick());
}

// ─────────────────────────────────────────────────────────────────────────────
// 송신 헬퍼
// ─────────────────────────────────────────────────────────────────────────────
void RosCom::Send(const char* msg) {
    // 타임아웃 계산 근거:
    //   최대 전송 ≈ 32자 (STATUS 응답)
    //   115200 bps → 1자 ≈ 0.087ms → 32자 ≈ 2.8ms
    //   여유 포함 10ms 로 설정.
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

void RosCom::SendACK() {
    Send("OK\n");
}

void RosCom::SendNACK(const char* reason) {
    SendFmt("ERR:%s\n", reason);
}

// ─────────────────────────────────────────────────────────────────────────────
// HAL UART/DMA 콜백 (C 링크, ISR 컨텍스트에서 호출됨)
// ─────────────────────────────────────────────────────────────────────────────

// HT / TC / IDLE 이벤트 통합 콜백
// · HT   (size = ROS_DMA_BUF_SIZE/2) : DMA 버퍼 앞쪽 절반 수신 완료
// · TC   (size = ROS_DMA_BUF_SIZE)   : DMA 버퍼 전체 수신 완료, DMA 위치 0 복귀
// · IDLE (size = 임의 위치)           : 회선 무음 — 패킷 경계 감지
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2 && g_ros_com != nullptr) {
        g_ros_com->OnRxEvent(Size);
    }
}

// UART 에러 발생 시 DMA 수신 재시작 (노이즈·프레이밍·오버런 에러 복구)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && g_ros_com != nullptr) {
        HAL_UART_AbortReceive(huart);   // DMA 및 UART 수신 중단
        g_ros_com->StartReceive();      // _dma_pos 초기화 후 DMA 재시작
    }
}
