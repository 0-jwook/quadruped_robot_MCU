/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.cpp
  * @brief   4족보행 로봇 메인 애플리케이션 (Direct Joint Control 버전)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
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
RosCom     ros(&huart2, &pca, &quad);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// C 파일들과의 링크를 위해 extern "C" 사용
extern "C" {
  void MX_GPIO_Init(void);
  void MX_USART2_UART_Init(void);
  void MX_I2C1_Init(void);
  void MX_I2C2_Init(void);
  void Error_Handler(void);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  // ── 초기화 ────────────────────────────────────────────────────────────────
  bool pca_ok = (pca.Init() == HAL_OK);
  bool imu_ok = (imu.Init() == HAL_OK);
  ros.SetDeviceStatus(pca_ok, imu_ok);

  if (pca_ok) {
      quad.SetDefaultPose();
  }

  ros.StartReceive();

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
  * @brief 시스템 클럭 설정 — HSI 8 MHz × PLL 16 = 64 MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
