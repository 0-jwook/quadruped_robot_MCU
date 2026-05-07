/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.cpp
  * @brief   4족보행 로봇 메인 애플리케이션
  *
  *  하드웨어 구성
  *  ─────────────────────────────────────────────────────────────
  *  MCU   : STM32F103RB (NUCLEO-F103RB, Cortex-M3, 64 MHz)
  *  UART2 : PA2(TX) / PA3(RX) — ROS 시리얼 브릿지, 115200 bps
  *  I2C1  : PB6(SCL) / PB7(SDA) — PCA9685 서보 드라이버
  *  I2C2  : PB10(SCL) / PB11(SDA) — MPU6050 IMU
  *
  *  루프 타이밍
  *  ─────────────────────────────────────────────────────────────
  *  IMU 전송  : 50 ms 주기 (20 Hz) — 상보 필터 정확도를 위해 고정
  *  Heartbeat :  1 s  주기  (1 Hz) — ROS 연결 감시용
  *  LED 토글  : 500 ms 주기  (2 Hz) — 정상 동작 시각 표시
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 루프 주기 상수 (ms)
static const uint32_t IMU_PERIOD_MS       =  100;   // 10 Hz — 20Hz 대비 UART 부하 절반으로 감소
                                                    //         보행 제어에는 10Hz 로도 충분
static const uint32_t HEARTBEAT_PERIOD_MS = 1000;   //  1 Hz
static const uint32_t LED_BLINK_MS        =  500;   //  2 Hz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PCA9685   pca(&hi2c1, 0x40);
Quadruped quad(&pca);
MPU6050   imu(&hi2c2);
RosCom    ros(&huart2, &pca, &quad);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief 애플리케이션 진입점
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  // ── 주변장치 초기화 ────────────────────────────────────────────────────────
  bool pca_ok = (pca.Init() == HAL_OK);
  bool imu_ok = (imu.Init() == HAL_OK);

  // 장치 상태를 RosCom 에 전달 — "STATUS\n" 명령 응답에 사용
  ros.SetDeviceStatus(pca_ok, imu_ok);

  // ── 부팅 메시지 전송 ────────────────────────────────────────────────────────
  char msg[64];
  snprintf(msg, sizeof(msg), "[SYSTEM] PCA9685 %s\r\n", pca_ok ? "OK" : "FAILED");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  snprintf(msg, sizeof(msg), "[SYSTEM] MPU6050 %s\r\n", imu_ok ? "OK" : "FAILED");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  // ── 기본 자세 초기화 (모든 관절 90°) ──────────────────────────────────────
  if (pca_ok) {
      quad.SetDefaultPose();
  }

  // ── UART 수신 인터럽트 시작 ─────────────────────────────────────────────────
  ros.StartReceive();

  // ── 타이밍 기준 시각 초기화 ─────────────────────────────────────────────────
  uint32_t last_imu_tick       = HAL_GetTick();
  uint32_t last_heartbeat_tick = HAL_GetTick();
  uint32_t last_led_tick       = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      // ① UART 명령 처리
      //    원형 버퍼에서 바이트를 꺼내 라인을 완성하고 ParseAndExecute 호출
      ros.Process();

      // ② IMU 텔레메트리 전송 (20 Hz)
      //    상보 필터는 호출 주기가 일정할수록 정확 → 50 ms 고정 권장
      if (imu_ok && (now - last_imu_tick >= IMU_PERIOD_MS)) {
          float r, p, y;
          imu.ReadData(r, p, y);
          ros.SendIMU(r, p, y);
          last_imu_tick = now;
      }

      // ③ Heartbeat 전송 (1 Hz)
      //    ROS 측에서 이 신호가 끊기면 STM32 연결 끊김으로 판단 가능
      if (now - last_heartbeat_tick >= HEARTBEAT_PERIOD_MS) {
          ros.SendHeartbeat();
          last_heartbeat_tick = now;
      }

      // ④ 상태 LED 토글 (2 Hz)
      //    점멸 중 → 메인루프 정상 동작 / 멈춤 → 루프 블로킹 의심
      if (now - last_led_tick >= LED_BLINK_MS) {
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          last_led_tick = now;
      }

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

/**
  * @brief GPIO 초기화
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief 오류 핸들러 — LED 고속 점멸로 오류 상태 시각 표시
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  // HAL_Delay 는 IRQ 비활성화 후 동작하지 않으므로 단순 루프로 대기
  while (1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      for (volatile uint32_t i = 0; i < 200000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
