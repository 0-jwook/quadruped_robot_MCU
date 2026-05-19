/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.cpp
  * @brief   4족보행 로봇 메인 애플리케이션 (USB CDC 버전)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"    // CubeMX 생성: MX_USB_DEVICE_Init()
#include "PCA9685.hpp"
#include "ros_com.hpp"
#include "Quadruped.hpp"
#include "MPU6050.hpp"
#include <cstring>
#include <cstdio>

/* Private define ------------------------------------------------------------*/
static const uint32_t CONTROL_PERIOD_MS   =   20;   // 50 Hz
static const uint32_t IMU_PERIOD_MS       =  100;   // 10 Hz
static const uint32_t HEARTBEAT_PERIOD_MS = 1000;   //  1 Hz
static const uint32_t LED_BLINK_MS        =  500;   //  2 Hz

/* Private variables ---------------------------------------------------------*/
PCA9685    pca(&hi2c1, 0x40);
Quadruped  quad(&pca);
MPU6050    imu(&hi2c2);
RosCom     ros(&pca, &quad);   // USB CDC — UART 인자 없음

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

extern "C" {
  void MX_GPIO_Init(void);
  void MX_I2C1_Init(void);
  void MX_I2C2_Init(void);
  void Error_Handler(void);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();   // USB CDC 초기화 (CubeMX 생성)
  MX_I2C1_Init();
  MX_I2C2_Init();

  // USB 열거(Enumeration) 완료 대기 (~500ms)
  HAL_Delay(500);

  // ── 초기화 ────────────────────────────────────────────────────────────────
  bool pca_ok = (pca.Init() == HAL_OK);
  bool imu_ok = (imu.Init() == HAL_OK);
  ros.SetDeviceStatus(pca_ok, imu_ok);

  if (pca_ok) {
      quad.SetDefaultPose();
  }

  uint32_t last_control_tick   = HAL_GetTick();
  uint32_t last_imu_tick       = HAL_GetTick();
  uint32_t last_heartbeat_tick = HAL_GetTick();
  uint32_t last_led_tick       = HAL_GetTick();

  /* Infinite loop */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      // ① 통신 처리
      ros.Process();

      // ② 고정 주기 관절 제어 루프 (50 Hz)
      if (now - last_control_tick >= CONTROL_PERIOD_MS) {
          JointAngleCmd cmd;
          if (ros.GetJointCmd(cmd)) {
              for (int i = 0; i < 4; i++) {
                  quad.SetLegAngle(i, cmd.angles[i*3], cmd.angles[i*3+1], cmd.angles[i*3+2]);
              }
          }
          last_control_tick = now;
      }

      // ③ IMU 텔레메트리 전송 (10 Hz)
      if (imu_ok && (now - last_imu_tick >= IMU_PERIOD_MS)) {
          float r, p, y;
          imu.ReadData(r, p, y);
          ros.SendIMU(r, p, y);
          last_imu_tick = now;
      }

      // ④ Heartbeat 전송 (1 Hz)
      if (now - last_heartbeat_tick >= HEARTBEAT_PERIOD_MS) {
          ros.SendHeartbeat();
          last_heartbeat_tick = now;
      }

      // ⑤ 상태 LED 토글
      if (now - last_led_tick >= LED_BLINK_MS) {
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          last_led_tick = now;
      }
  }
}

/**
  * @brief Error_Handler 구현
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      for (volatile uint32_t i = 0; i < 200000; i++);
  }
}

/**
  * @brief 시스템 클럭 — HSI/2 × 12 = 48 MHz, USB = 48 MHz (/1)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL12;   // 4MHz × 12 = 48MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  // USB 클럭 = SYSCLK / 1 = 48 MHz
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // APB1 = 24 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // APB2 = 48 MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}
