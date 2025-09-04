    /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "hc-sr04.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ECHO_Pin GPIO_PIN_0
#define ECHO_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_1
#define TRIG_GPIO_Port GPIOA
#define PB0_Pin GPIO_PIN_0
#define PB0_GPIO_Port GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t IC_Val1 = 0; // 發射時間點
uint32_t IC_Val2 = 0; // 接收時間點
uint32_t Difference = 0;  // 發射接收時間差 ( IC_Val2 - IC_Val1 )
uint8_t Is_First_Captured = 0;  //標記訊號上升下降狀態
float Distance = 0.0;         // 物距 ( 時差Difference * 聲速 )
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void pwm_motor_control(int dutyA, int dutyB) {
    // 控制 enableA 和 enableB 的 PWM 佔空比
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, dutyA);  // 控制 enableA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, dutyB);  // 控制 enableB
}

void move_forward() {
    // 設定馬達正轉
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  // IN1 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // IN2 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);  // IN3 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // IN4 方向設定

    // 設定 PWM 佔空比，讓車輛前進
    pwm_motor_control(500, 500);  // 前進的速度，可根據需要調整
}

void turn_left() {
    // 設定馬達左轉
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // IN1 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // IN2 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // IN3 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // IN4 方向設定

    // 設定 PWM 佔空比，讓車輛左轉
    pwm_motor_control(500, 500);  // 轉向的速度，可根據需要調整
}

void turn_right() {
    // 設定馬達右轉
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  // IN1 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);    // IN2 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);    // IN3 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  // IN4 方向設定

    // 設定 PWM 佔空比，讓車輛右轉
    pwm_motor_control(500, 500);  // 轉向的速度，可根據需要調整
}

void stop_motor() {
    // 停止馬達
    pwm_motor_control(0, 0);  // 停止車輛
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // IN1 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);    // IN2 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);    // IN3 方向設定
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  // IN4 方向設定

}

void control(int distance) {
    if (Distance > 0 && Distance < 20) {  // 假設距離小於 20 cm 時需要避開障礙物
        stop_motor();   // 停止前進
        HAL_Delay(500); // 等待一段時間
        turn_left();    // 左轉
        HAL_Delay(1000); // 轉向後停留一段時間
        turn_right();
        HAL_Delay(1000)
        move_forward(); // 重新前進
    }
    else
    	move_forward();
}
void delay_us(uint16_t TRIGGER_t) {  //因為函式庫內 delay() 單位為毫秒ms，因此需自定義微秒延遲器
    __HAL_TIM_SET_COUNTER(&htim4, 0);  // 設定計數器歸零
    while (__HAL_TIM_GET_COUNTER(&htim4) < TRIGGER_t ); // while持續進行直到計數器 = TRIGGER_t
}
    // 發送 Trig 信號 ( 長度為10微渺的高電位 )
    void HCSR04_Trigger() {
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // 高電位
        delay_us(10);  // 保持 10us
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // 低電位
    }
    // 接收 Echo 信號
    void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            if (Is_First_Captured == 0) {  // 第一次捕獲上升沿
            	// 這次中斷是 TIM2_CH1 觸發的，執行距離計算
                IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                Is_First_Captured = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            } else {  // 第二次捕獲下降沿
                IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                Difference = IC_Val2 - IC_Val1;
                Distance = (Difference * 0.0343) / 2;
                Is_First_Captured = 0;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
    }

/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//定时器中断函数
{
  Hcsr04TimIcIsr(htim);
}
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)//定时器溢出中断
{
  Hcsr04TimOverflowIsr(htim);
}
*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start(&htim4);
  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  	HAL_TIM_Base_Start_IT(&htim3);
//HAL_TIM_Base_Start_IT(&htim4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    Hcsr04Init(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  	  HCSR04_Trigger();
	          HAL_Delay(100);
	          control(Distance);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
