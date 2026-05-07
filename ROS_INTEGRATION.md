# Quadrupedal Robot - ROS 통합 문서

## 개요

이 문서는 STM32F103RB 기반 4족보행 로봇과 ROS(Robot Operating System)를 연동하는 방법을 설명합니다.

STM32는 ROS 미들웨어를 직접 구동하지 않으며, **UART를 통한 ASCII 텍스트 프로토콜**로 통신합니다.
ROS 측에서 시리얼 드라이버 노드를 통해 명령을 송신하고 텔레메트리를 수신합니다.

---

## 하드웨어 연결

### UART 인터페이스

| 항목 | 값 |
|------|-----|
| 포트 | USART2 |
| STM32 TX 핀 | PA2 |
| STM32 RX 핀 | PA3 |
| 보드레이트 | **115200 bps** |
| 데이터 비트 | 8 bit |
| 스톱 비트 | 1 bit |
| 패리티 | None |
| 흐름 제어 | None |

### PC 연결 방법

NUCLEO-F103RB 보드의 USB-to-Serial 변환칩(ST-Link)을 통해 PC와 연결됩니다.

```raw
[ROS PC] <---USB---> [NUCLEO ST-Link] <---UART2 (PA2/PA3)---> [STM32F103RB]
```

Linux 기준 포트: `/dev/ttyACM0` 또는 `/dev/ttyUSB0`

---

## 통신 프로토콜

### 명령 형식 (ROS → STM32)

모든 명령은 **ASCII 텍스트**이며 `\n` 또는 `\r`로 종료됩니다.

---

### 명령어 목록

#### 1. 전체 다리 제어 (`A` 명령)

모든 4개 다리의 12개 관절을 한 번에 제어합니다.

```raw
A:<h0>,<t0>,<c0>,<h1>,<t1>,<c1>,<h2>,<t2>,<c2>,<h3>,<t3>,<c3>\n
```

| 파라미터 | 설명 | 범위 |
|----------|------|------|
| h0~h3 | 각 다리의 Hip(고관절) 각도 | 0.0 ~ 180.0 |
| t0~t3 | 각 다리의 Thigh(허벅지) 각도 | 0.0 ~ 180.0 |
| c0~c3 | 각 다리의 Calf(종아리) 각도 | 0.0 ~ 180.0 |

다리 순서: 0=앞왼쪽(FL), 1=앞오른쪽(FR), 2=뒤왼쪽(BL), 3=뒤오른쪽(BR)

**예시:**
```raw
A:90,45,90,90,45,90,90,45,90,90,45,90\n
```

---

#### 2. 단일 다리 제어 (`L` 명령)

특정 다리의 3개 관절을 한 번에 제어합니다.

```raw
L<leg_idx>:<hip>,<thigh>,<calf>\n
```

| 파라미터 | 설명 |
|----------|------|
| leg_idx | 다리 번호 (0~3) |
| hip | Hip 각도 (0.0~180.0) |
| thigh | Thigh 각도 (0.0~180.0) |
| calf | Calf 각도 (0.0~180.0) |

**다리 번호:**
| 번호 | 위치 |
|------|------|
| 0 | 앞 왼쪽 (Front Left, FL) |
| 1 | 앞 오른쪽 (Front Right, FR) |
| 2 | 뒤 왼쪽 (Back Left, BL) |
| 3 | 뒤 오른쪽 (Back Right, BR) |

**예시:**
```raw
L0:90,45,90\n        # 앞 왼쪽 다리 이동
L2:80,60,100\n       # 뒤 왼쪽 다리 이동
```

---

#### 3. 단일 관절 제어 (`J` 명령)

특정 다리의 특정 관절 하나만 제어합니다.

```raw
J<leg_idx>,<joint_idx>:<angle>\n
```

| 파라미터 | 설명 |
|----------|------|
| leg_idx | 다리 번호 (0~3) |
| joint_idx | 관절 번호 (0=Hip, 1=Thigh, 2=Calf) |
| angle | 목표 각도 (0.0~180.0) |

**예시:**
```raw
J0,0:90.0\n      # 앞왼쪽 다리 Hip → 90°
J1,1:60.0\n      # 앞오른쪽 다리 Thigh → 60°
J3,2:120.5\n     # 뒤오른쪽 다리 Calf → 120.5°
```

---

#### 4. 직접 채널 제어 (`C` 명령, 레거시)

PCA9685 PWM 채널을 직접 제어합니다. 디버깅 목적으로 사용합니다.

```raw
C<channel>:<angle>\n
```

| 파라미터 | 설명 |
|----------|------|
| channel | PWM 채널 번호 (0~15) |
| angle | 목표 각도 (0.0~180.0) |

**예시:**
```raw
C0:90.0\n        # 채널 0번 서보 → 90°
C5:45.0\n        # 채널 5번 서보 → 45°
```

---

### 텔레메트리 형식 (STM32 → ROS)

STM32가 자동으로 ROS 측으로 전송하는 데이터입니다.

#### IMU 데이터 (5초 주기)

```raw
IMU:<roll>,<pitch>,<yaw>\n
```

| 필드 | 설명 | 단위 |
|------|------|------|
| roll | 롤 각도 | 도(°) |
| pitch | 피치 각도 | 도(°) |
| yaw | 요 각도 (항상 0.0, 자력계 없음) | 도(°) |

**예시:**
```raw
IMU:-2.34,1.56,0.00\n
```

#### 시스템 메시지 (부팅 시)

```raw
[SYSTEM] PCA9685 Initialized OK\r\n
[SYSTEM] MPU6050 Initialized OK\r\n
```

#### 에러 메시지

```raw
[ERROR] I2C Write Failed (Status: X)\r\n
```

---

## 서보 채널 매핑

```raw
        앞(Front)
   Left        Right
  ┌─────────────────┐
  │  FL (0)  FR (1) │
  │  BL (2)  BR (3) │
  └─────────────────┘
        뒤(Back)
```

| 다리 | 인덱스 | Hip 채널 | Thigh 채널 | Calf 채널 |
|------|--------|----------|-----------|----------|
| Front Left (FL) | 0 | 0 | 1 | 2 |
| Front Right (FR) | 1 | 4 | 5 | 6 |
| Back Left (BL) | 2 | 8 | 9 | 10 |
| Back Right (BR) | 3 | 12 | 13 | 14 |

---
```bash
sudo apt install ros-<distro>-serial
# 또는
pip install pyserial
```

### Python ROS 노드 예시

```python
#!/usr/bin/env python3
"""
Quadrupedal Robot Serial Bridge Node
STM32 <-> ROS UART 브릿지 노드
"""

import rospy
import serial
import threading
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200


class QuadrupedBridge:
    def __init__(self):
        rospy.init_node('quadruped_serial_bridge')

        # 시리얼 포트 오픈
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        rospy.loginfo(f"Serial port opened: {SERIAL_PORT} @ {BAUD_RATE}")

        # Publishers
        self.imu_pub = rospy.Publisher('/quadruped/imu_raw', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/quadruped/cmd_all_legs', Float32MultiArray, self.cb_all_legs)
        rospy.Subscriber('/quadruped/cmd_leg', Float32MultiArray, self.cb_single_leg)
        rospy.Subscriber('/quadruped/cmd_joint', Float32MultiArray, self.cb_single_joint)
        rospy.Subscriber('/quadruped/cmd_raw', String, self.cb_raw)

        # 수신 스레드 시작
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def send_command(self, cmd: str):
        """STM32로 명령 전송"""
        if not cmd.endswith('\n'):
            cmd += '\n'
        self.ser.write(cmd.encode('ascii'))
        rospy.logdebug(f"Sent: {cmd.strip()}")

    def cb_all_legs(self, msg: Float32MultiArray):
        """
        토픽: /quadruped/cmd_all_legs
        데이터: [h0,t0,c0, h1,t1,c1, h2,t2,c2, h3,t3,c3] (12개 float)
        """
        if len(msg.data) != 12:
            rospy.logwarn("cmd_all_legs requires 12 values")
            return
        d = msg.data
        cmd = f"A:{d[0]:.1f},{d[1]:.1f},{d[2]:.1f}," \
              f"{d[3]:.1f},{d[4]:.1f},{d[5]:.1f}," \
              f"{d[6]:.1f},{d[7]:.1f},{d[8]:.1f}," \
              f"{d[9]:.1f},{d[10]:.1f},{d[11]:.1f}"
        self.send_command(cmd)

    def cb_single_leg(self, msg: Float32MultiArray):
        """
        토픽: /quadruped/cmd_leg
        데이터: [leg_idx, hip, thigh, calf]
        """
        if len(msg.data) != 4:
            rospy.logwarn("cmd_leg requires 4 values: [leg, hip, thigh, calf]")
            return
        leg = int(msg.data[0])
        cmd = f"L{leg}:{msg.data[1]:.1f},{msg.data[2]:.1f},{msg.data[3]:.1f}"
        self.send_command(cmd)

    def cb_single_joint(self, msg: Float32MultiArray):
        """
        토픽: /quadruped/cmd_joint
        데이터: [leg_idx, joint_idx, angle]
          joint_idx: 0=Hip, 1=Thigh, 2=Calf
        """
        if len(msg.data) != 3:
            rospy.logwarn("cmd_joint requires 3 values: [leg, joint, angle]")
            return
        leg = int(msg.data[0])
        joint = int(msg.data[1])
        cmd = f"J{leg},{joint}:{msg.data[2]:.1f}"
        self.send_command(cmd)

    def cb_raw(self, msg: String):
        """
        토픽: /quadruped/cmd_raw
        데이터: 원시 명령 문자열 (e.g. "C0:90.0")
        """
        self.send_command(msg.data)

    def read_loop(self):
        """STM32에서 데이터 수신 및 파싱"""
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('IMU:'):
                    # IMU:-2.34,1.56,0.00
                    self.imu_pub.publish(line)
                    parts = line[4:].split(',')
                    if len(parts) == 3:
                        roll, pitch, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                        rospy.logdebug(f"IMU -> roll={roll}, pitch={pitch}, yaw={yaw}")

                elif line.startswith('[SYSTEM]'):
                    rospy.loginfo(f"STM32: {line}")

                elif line.startswith('[ERROR]'):
                    rospy.logerr(f"STM32: {line}")

            except Exception as e:
                rospy.logwarn(f"Serial read error: {e}")

    def spin(self):
        rospy.spin()
        self.ser.close()


if __name__ == '__main__':
    node = QuadrupedBridge()
    node.spin()
```

---

## ROS 토픽 정의

| 토픽 | 방향 | 메시지 타입 | 설명 |
|------|------|------------|------|
| `/quadruped/cmd_all_legs` | PC→STM32 | `Float32MultiArray` | 12관절 전체 제어 |
| `/quadruped/cmd_leg` | PC→STM32 | `Float32MultiArray` | 단일 다리 제어 |
| `/quadruped/cmd_joint` | PC→STM32 | `Float32MultiArray` | 단일 관절 제어 |
| `/quadruped/cmd_raw` | PC→STM32 | `String` | 원시 ASCII 명령 |
| `/quadruped/imu_raw` | STM32→PC | `String` | IMU 데이터 (5초 주기) |

---

## 사용 예시 (rostopic pub)

```bash
# 모든 다리를 기본 자세로
rostopic pub /quadruped/cmd_all_legs std_msgs/Float32MultiArray \
  "data: [90,45,90, 90,45,90, 90,45,90, 90,45,90]"

# 앞 왼쪽 다리(0번) 이동
rostopic pub /quadruped/cmd_leg std_msgs/Float32MultiArray \
  "data: [0, 90, 60, 110]"

# 앞 왼쪽(0번) 다리의 Thigh(1번) 관절을 45도로
rostopic pub /quadruped/cmd_joint std_msgs/Float32MultiArray \
  "data: [0, 1, 45.0]"

# 원시 명령 전송
rostopic pub /quadruped/cmd_raw std_msgs/String "data: 'C0:90.0'"

# IMU 데이터 수신 모니터링
rostopic echo /quadruped/imu_raw
```

---

## 런치 파일 예시

```xml
<!-- quadruped_bridge.launch -->
<launch>
  <node name="quadruped_serial_bridge"
        pkg="quadruped_ros"
        type="serial_bridge.py"
        output="screen">
    <param name="port" value="/dev/ttyACM0"/>
    <param name="baud" value="115200"/>
  </node>
</launch>
```
```bash
roslaunch quadruped_ros quadruped_bridge.launch
```

---

## 시리얼 포트 권한 설정 (Linux)

```bash
# 사용자를 dialout 그룹에 추가 (1회)
sudo usermod -a -G dialout $USER

# 즉시 권한 적용 (재부팅 전 임시)
sudo chmod 666 /dev/ttyACM0
```

---

## 각도 기준 및 서보 스펙

| 항목 | 값 |
|------|-----|
| 서보 동작 주파수 | 50 Hz (20ms 주기) |
| 각도 범위 | 0° ~ 180° |
| 중립 위치 | 90° |
| PWM 펄스폭 | 1.0ms (0°) ~ 2.0ms (180°) |
| PCA9685 카운트 | 150 (0°) ~ 500 (180°) |

---

## 통신 흐름 다이어그램

```raw
[ROS Node]
    │  rostopic pub /quadruped/cmd_leg
    ▼
[serial_bridge.py]
    │  "L0:90,45,90\n"  (ASCII, UART 115200bps)
    ▼
[STM32 USART2 인터럽트]
    │
    ▼
[ros_com.cpp: ParseAndExecute()]
    │  L0 → leg=0, hip=90, thigh=45, calf=90
    ▼
[Quadruped::SetLegAngle(0, 90, 45, 90)]
    │
    ▼
[PCA9685::SetAngle(ch, angle)] × 3관절
    │  I2C1 (400kHz)
    ▼
[PCA9685 PWM 출력]
    │  50Hz PWM 신호
    ▼
[서보 모터 12개]
```

---

## 주의사항

1. **명령 종료 문자**: 모든 명령은 반드시 `\n`으로 끝나야 합니다.
2. **각도 범위**: 0~180° 범위를 벗어나면 PCA9685 내부에서 클램핑됩니다.
3. **IMU Yaw**: MPU6050만 사용하므로 Yaw 값은 항상 0.0입니다. 정확한 Yaw가 필요하면 자력계(Magnetometer) 추가가 필요합니다.
4. **명령 처리 속도**: STM32는 수신된 명령을 메인 루프에서 처리하므로, 명령을 너무 빠르게 연속 전송하면 버퍼(128바이트)가 초과될 수 있습니다. 명령 간 최소 20ms 간격을 권장합니다.
5. **포트 이름**: Linux에서 NUCLEO 보드 포트는 `/dev/ttyACM0`이며, USB 허브나 다른 장치가 연결된 경우 번호가 달라질 수 있습니다.
