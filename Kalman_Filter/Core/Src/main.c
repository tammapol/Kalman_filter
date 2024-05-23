/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ***************************** 	*************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
float diffTime;
int32_t diffPosition;
int start = 0;
uint32_t QEIReadRaw;
int PWM;
int timerCount;
int timerPWM;
int timerPWMCurve;
int timerPWMCurveMode;
float Voltage;
float Velo_Kalman_Lnw;
float Velo_Kalman_Wre[2];
float Acc_Kalman_Wre;
float Pos_Kalman_Wre[2];
float diffPosKalman;
float diffVelocity;
float diffVeloKalman;

typedef struct
{
// for record New / Old value to calculate dx / dt
	float Position[2];
	int QEIPosition;
	int TimeStamp[2];
	float Velocity[2];
	int QEIPostion_1turn;
	float QEIAngularVelocity;
	float QEIAngularVeloKalman;
	float QEIAcceleration;
}QEI_StructureTypeDef;
QEI_StructureTypeDef QEIdata = {0};
uint64_t _micros = 0;
enum
{
NEW,OLD
};

// �?ำห�?ดตัว�?�?ร Kalman
float32_t W_k[3] = {0.0f, 0.0f, 0.0f};
arm_matrix_instance_f32 W_k_matrix;

float32_t GW[3] = {0.0f, 0.0f, 0.0f};
arm_matrix_instance_f32 GW_matrix;

float32_t X_k[3] = {0.0f, 0.0f, 0.0f};
arm_matrix_instance_f32 X_k_matrix;

float32_t P_k[9] = {0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f};
arm_matrix_instance_f32 P_k_matrix;

float32_t A[9] = {1.0f, 0.01f, 0.5*0.01*0.01f,
                  0.0f, 1.0f, 0.01f,
                  0.0f, 0.0f, 1.0f};
arm_matrix_instance_f32 A_matrix;

float32_t A_transpose[9] = {1.0f, 0.0f, 0.0f,
                            0.01f, 1.0f, 0.0f,
                            0.5*0.01*0.01f, 0.01f, 1.0f};
arm_matrix_instance_f32 A_transpose_matrix;

float32_t I[9] = {1.0f, 0.0f, 0.0f,
                  0.0f, 1.0f, 0.0f,
                  0.0f, 0.0f, 1.0f};
arm_matrix_instance_f32 I_matrix;

float32_t B[3] = {0.0f, 0.0f, 1.0f};
arm_matrix_instance_f32 B_matrix;

float32_t C[3] = {0.0f, 1.0f, 0.0f};
arm_matrix_instance_f32 C_matrix;

float32_t C_transpose[3] = {0.0f,
							1.0f,
							0.0f};
arm_matrix_instance_f32 C_transpose_matrix;

//float32_t G[3] = {0.0f, 0.0f, 1.0f};
float32_t G[3] = {0.5*0.01*0.01f,
				  0.01f,
				  1.0f};
arm_matrix_instance_f32 G_matrix;

float32_t G_transpose[3] = {0.5*0.01*0.01f, 0.01f, 1.0f};
arm_matrix_instance_f32 G_transpose_matrix;

float32_t Es_velocity[1] = {0.0f};
arm_matrix_instance_f32 Output_matrix;

arm_matrix_instance_f32 GGT_matrix;
float32_t GGT[9];

arm_matrix_instance_f32 GQGT_matrix;
float32_t GQGT[9];
//------------------------------------------- for equa ------------------------------------
// Compute Xk = Ax + Bu
arm_matrix_instance_f32 Bu_matrix;
float32_t Bu_data[3];
arm_matrix_instance_f32 Ax_matrix;
float32_t Ax_data[3];

// Compute (C * P_k * C^T + R)
arm_matrix_instance_f32 CP_matrix;
float32_t CP[3];
arm_matrix_instance_f32 CPCT_matrix;
float32_t CPCT[1];
arm_matrix_instance_f32 CPCTR_matrix;
float32_t CPCTR[1];

// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
// Compute Xk = Ax + Bu
arm_matrix_instance_f32 Bu_matrix;
float32_t Bu_data[3];
arm_matrix_instance_f32 Ax_matrix;
float32_t Ax_data[3];

// Compute (C * P_k * C^T + R)
arm_matrix_instance_f32 CP_matrix;
float32_t CP[3];
arm_matrix_instance_f32 CPCT_matrix;
float32_t CPCT[1];
arm_matrix_instance_f32 CPCTR_matrix;
float32_t CPCTR[1];

// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
arm_matrix_instance_f32 K_matrix;
float32_t K[3];
arm_matrix_instance_f32 PCT_matrix;
float32_t PCT[3];

// Compute inverse of (C * P_k * C^T + R)
arm_matrix_instance_f32 CPCTRinv_matrix;
float32_t CPCTRinv[1];

// Computation of the estimated state
arm_matrix_instance_f32 Cx_matrix;
float32_t Cx[1];
arm_matrix_instance_f32 yCx_matrix;
float32_t yCx[1];
arm_matrix_instance_f32 KyCx_matrix;
float32_t KyCx[3];

float32_t Q = 100.0f;
arm_matrix_instance_f32 R_matrix;
float32_t R[1] = {1.0f};

arm_matrix_instance_f32 Z_matrix;
float32_t Z[1];

arm_matrix_instance_f32 Velocity_matrix;

float Kalman_Speed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
void QEIEncoderPosVel_Update();
void Kalman_Start();
float SteadyStateKalmanFilter(float32_t Vin,float32_t Velocity);
float KalmanFilter(float32_t Vin,float32_t Velocity);
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
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Kalman_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Voltage = 5.0*PWM/1000.0;
//	  PWM = 100 * sin(HAL_GetTick()/1000.0);

//	  if (start == 1)
//	  {
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM);
//	  }
//	  else if (start == 0)
//	  {
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	  }

//	  QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim3);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64511;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//signed long kalman_filter(signed long ADC_Value)
//{
//    float x_k1_k1,x_k_k1;
//    static float ADC_OLD_Value;
//    float Z_k;
//    static float P_k1_k1;
//
//    static float Q = 0.01;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
//    static float R = 1; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
//    static float Kg = 0;
//    static float P_k_k1 = 1;
//
//    float kalman_adc;
//    static float kalman_adc_old=0;
//    Z_k = ADC_Value;
//    x_k1_k1 = kalman_adc_old;
//
//    x_k_k1 = x_k1_k1;
//    P_k_k1 = P_k1_k1 + Q;
//
//    Kg = P_k_k1/(P_k_k1 + R);
//
//    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
//    P_k1_k1 = (1 - Kg)*P_k_k1;
//    P_k_k1 = P_k1_k1;
//
//    ADC_OLD_Value = ADC_Value;
//    kalman_adc_old = kalman_adc;
//
//    return kalman_adc;
//}


//MicroSecondTimer Implement
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		_micros += UINT32_MAX;
	}
	if(htim == &htim4)
	{
//		QEIEncoderPosVel_Update();
		timerCount++;
		timerPWM++;
		timerPWMCurve++;

		if(timerCount == 1000)
		{
			QEIEncoderPosVel_Update();
			timerCount = 0;
		}
		if(timerPWM == 100)
		{
			if(timerPWMCurve == 100000)
			{
				PWM = 0;
			}
			else if (timerPWMCurve == 200000)
			{
				PWM = 400;
			}
			else if (timerPWMCurve == 300000)
			{
				PWM = 200;
			}
			else if (timerPWMCurve == 400000)
			{
				PWM = 1000;
			}
			else if (timerPWMCurve == 500000)
			{
				PWM = 0;
				timerPWMCurve = 0;
			}

			if (start == 1)
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM);
			}
			else if (start == 0)
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			}
			timerPWM = 0;
		}
	}
}
uint64_t micros()
{
	return __HAL_TIM_GET_COUNTER(&htim5)+_micros;
}

void QEIEncoderPosVel_Update()
{
    QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim3);
    Velo_Kalman_Wre[NEW] = (diffPosKalman / diffTime)* 2 * 3.14159265359 * 9.5492968 / 3072.0;
    Pos_Kalman_Wre[NEW] = KalmanFilter(0,QEIdata.Position[NEW]);
    QEIdata.TimeStamp[NEW] = micros();
    QEIdata.Velocity[NEW] = QEIdata.QEIAngularVelocity;
    diffTime = (QEIdata.TimeStamp[NEW] - QEIdata.TimeStamp[OLD]) / 1000000.0f; // seconds
    diffPosition = QEIdata.Position[NEW] - QEIdata.Position[OLD];
    diffVelocity = QEIdata.Velocity[NEW] - QEIdata.Velocity[OLD];
    diffPosKalman = Pos_Kalman_Wre[NEW] - Pos_Kalman_Wre[OLD];
    diffVeloKalman = Velo_Kalman_Wre[NEW] - Velo_Kalman_Wre[OLD];
    // Handle wrap-around
    if (diffPosition > 32256)
        diffPosition -= 64512;
    else if (diffPosition < -32256)
        diffPosition += 64512;

    if (diffPosKalman > 32256)
		diffPosKalman -= 64512;
	else if (diffPosKalman < -32256)
		diffPosKalman += 64512;


    Acc_Kalman_Wre = (diffVeloKalman / diffTime);
    // Calculate the angular velocity
    QEIdata.QEIAngularVelocity = (diffPosition / diffTime) * 2 * 3.14159265359 * 9.5492968 / 3072.0; // rad/s (assuming 3072 counts per revolution)
    QEIdata.QEIAcceleration = (diffVelocity / diffTime);

    // Use the Kalman filter to smooth the angular velocity
//    Velo_Kalman_Wre = KalmanFilter(0,QEIdata.QEIAngularVelocity);
//    Acc_Kalman_Wre = KalmanFilter(0,QEIdata.QEIAcceleration);

    // Update old values
    Pos_Kalman_Wre[OLD] =  Pos_Kalman_Wre[NEW];
    QEIdata.Position[OLD] = QEIdata.Position[NEW];
    QEIdata.TimeStamp[OLD] = QEIdata.TimeStamp[NEW];
    QEIdata.Velocity[OLD] = QEIdata.Velocity[NEW];
    Velo_Kalman_Wre[OLD] = Velo_Kalman_Wre[NEW];

	 // Collect data
//	    QEIdata.TimeStamp[NEW] = micros();
//	    QEIdata.Position[NEW] = QEIReadRaw;
//
//	    // Calculate position within one turn
//	    QEIdata.QEIPostion_1turn = QEIdata.Position[NEW] % 3072;
//
//	    // Calculate dx (difference in position)
//	    diffPosition = QEIdata.Position[NEW] - QEIdata.Position[OLD];
//
//	    // Handle wrap-around
//	    if (diffPosition > 32256)
//	        diffPosition -= 64512;
//	    else if (diffPosition < -32256)
//	        diffPosition += 64512;
//
//	    // Calculate dt (difference in time)
//	    diffTime = (float)(QEIdata.TimeStamp[NEW] - QEIdata.TimeStamp[OLD]) * 0.000001f; // Convert to seconds
//
//	    // Avoid division by zero
//	    if (diffTime > 0.0f)
//	    {
//	        // Calculate angular velocity
//	        QEIdata.QEIAngularVelocity = (float)diffPosition / diffTime;
//
//	        // Apply Kalman filter
////	        QEIdata.QEIAngularVeloKalman = kalman_filter(QEIdata.QEIAngularVelocity);
//	    }
//	    else
//	    {
//	        QEIdata.QEIAngularVelocity = 0.0f;
////	        QEIdata.QEIAngularVeloKalman = kalman_filter(0.0f);
//	    }
//
//	    // Store values for the next loop
//	    QEIdata.Position[OLD] = QEIdata.Position[NEW];
//	    QEIdata.TimeStamp[OLD] = QEIdata.TimeStamp[NEW];
}

volatile arm_status Calst;
float checkVal;

float KalmanFilter(float32_t Vin,float32_t Velocity){
	  arm_mat_init_f32(&Velocity_matrix, 1, 1,(float32_t*) &Velocity);
	  // Compute Xk = Ax + Bu + Gw
	  arm_mat_scale_f32(&B_matrix, 0, &Bu_matrix); 		   				// Bu
	  arm_mat_mult_f32(&A_matrix, &X_k_matrix, &Ax_matrix);  		   		// Ax
	  arm_mat_add_f32(&Ax_matrix, &Bu_matrix, &X_k_matrix); 		   		// Xk = Ax + Bu
	  arm_mat_add_f32(&X_k_matrix, &GW_matrix, &X_k_matrix);			// Xk = Ax + Bu + Gw

	  // Compute (A * P_pk * A^T + G * Q * G^T)
	  arm_mat_mult_f32(&A_matrix, &P_k_matrix, &P_k_matrix);  		   		// Pk = A * P_pk
	  arm_mat_mult_f32(&P_k_matrix, &A_transpose_matrix, &P_k_matrix); 		// Pk = A * P_pk * A^T
	  arm_mat_mult_f32(&G_matrix, &G_transpose_matrix, &GGT_matrix);        // G * G^T
	  arm_mat_scale_f32(&GGT_matrix, Q, &GQGT_matrix); 				   	   	// G * Q
	  arm_mat_add_f32(&P_k_matrix, &GQGT_matrix, &P_k_matrix); 	       		// A * P_pk * A^T + G * Q * G^T

	  // Compute (C * P_k * C^T + R)
	  arm_mat_mult_f32(&C_matrix, &P_k_matrix, &CP_matrix);			     // C * Pk
	  arm_mat_mult_f32(&CP_matrix, &C_transpose_matrix, &CPCT_matrix);   // C * Pk * C^T
	  arm_mat_add_f32(&CPCT_matrix, &R_matrix, &CPCTR_matrix);			 // C * P_k * C^T + R

	  // Compute inverse of (C * P_k * C^T + R)
	  arm_mat_inverse_f32(&CPCTR_matrix, &CPCTRinv_matrix);					 // inverse of (C * P_k * C^T + R)

	  // Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
	  arm_mat_mult_f32(&P_k_matrix, &C_transpose_matrix, &PCT_matrix); 		 // P_k * C^T
	  arm_mat_mult_f32(&PCT_matrix, &CPCTRinv_matrix, &K_matrix);  			 // P_k * C^T * inv(C * P_k * C^T + R)

	  // Computation of the estimated state
	  arm_mat_mult_f32(&C_matrix, &X_k_matrix, &Cx_matrix);				 // C * X_k
	  arm_mat_sub_f32(&Velocity_matrix,  &Cx_matrix, &yCx_matrix);			  // y - ( C * X_k )
	  arm_mat_mult_f32(&K_matrix, &yCx_matrix, &KyCx_matrix);		     // K( y - ( C * X_k ) )
	  arm_mat_add_f32(&X_k_matrix, &KyCx_matrix, &X_k_matrix);		 	 // X_k + K( y - ( C * X_k ) )

	  // Computation of the estimated output
	  arm_mat_mult_f32(&C_matrix, &X_k_matrix, &Output_matrix);

	  // Computation of the state covariance error
	  arm_matrix_instance_f32 temp_matrix4;
	  float32_t temp_data4[9];
	  arm_mat_init_f32(&temp_matrix4, 3, 3,(float32_t*) &temp_data4);

	  arm_mat_mult_f32(&K_matrix, &C_matrix, &temp_matrix4);				// K * C
	  arm_mat_sub_f32(&I_matrix, &temp_matrix4, &temp_matrix4);			// (I - (K * C))
	  arm_mat_mult_f32(&temp_matrix4, &P_k_matrix, &P_k_matrix);			// (I - (K * C)) * P_k
	  Kalman_Speed = X_k[1];
	  return  Kalman_Speed;
}
void Kalman_Start(){
	arm_mat_init_f32(&X_k_matrix, 3, 1,(float32_t*) &X_k);
	arm_mat_init_f32(&P_k_matrix, 3, 3,(float32_t*) &P_k);
	arm_mat_init_f32(&W_k_matrix, 3, 1,(float32_t*) &W_k);

	arm_mat_init_f32(&A_matrix, 3, 3,(float32_t*) &A);
	arm_mat_init_f32(&B_matrix, 3, 1,(float32_t*) &B);
	arm_mat_init_f32(&C_matrix, 1, 3,(float32_t*) &C);
	arm_mat_init_f32(&G_matrix, 3, 1,(float32_t*) &G);

	arm_mat_init_f32(&A_transpose_matrix, 3, 3,(float32_t*) &A_transpose);
	arm_mat_init_f32(&C_transpose_matrix, 3, 1,(float32_t*) &C_transpose);
	arm_mat_init_f32(&G_transpose_matrix, 1, 3,(float32_t*) &G_transpose);

	arm_mat_init_f32(&GGT_matrix, 3, 3,(float32_t*) &GGT);
	arm_mat_init_f32(&GQGT_matrix, 3, 3,(float32_t*) &GQGT);

	// Compute Xk = Ax + Bu
	arm_mat_init_f32(&Bu_matrix, 3, 1,(float32_t*) &Bu_data);
	arm_mat_init_f32(&Ax_matrix, 3, 1,(float32_t*) &Ax_data);
//	arm_mat_init_f32(&GW_matrix, 3, 1,(float32_t*) &GW_);

	// Compute (C * P_k * C^T + R)
	arm_mat_init_f32(&CP_matrix, 1, 3,(float32_t*) &CP);
	arm_mat_init_f32(&CPCT_matrix, 1, 1,(float32_t*) &CPCT);
	arm_mat_init_f32(&CPCTR_matrix, 1, 1,(float32_t*) &CPCTR);

	// Compute Kalman Gain: K = P_k * C^T * inv(C * P_k * C^T + R)
	arm_mat_init_f32(&K_matrix, 3, 1,(float32_t*) &K);
	arm_mat_init_f32(&PCT_matrix, 3, 1,(float32_t*) &PCT);

	// Compute inverse of (C * P_k * C^T + R)
	arm_mat_init_f32(&CPCTRinv_matrix, 1, 1,(float32_t*) &CPCTRinv);

	// Computation of the estimated state
	arm_mat_init_f32(&Cx_matrix, 1, 1,(float32_t*) &Cx);
	arm_mat_init_f32(&yCx_matrix, 1, 1,(float32_t*) &yCx);
	arm_mat_init_f32(&KyCx_matrix, 3, 1,(float32_t*) &KyCx);

	arm_mat_init_f32(&Output_matrix, 1, 1,(float32_t*) &Es_velocity);

	arm_mat_init_f32(&I_matrix, 3, 3,(float32_t*) &I);

	arm_mat_init_f32(&R_matrix, 1, 1,(float32_t*) &R);
	arm_mat_init_f32(&Z_matrix, 1, 1,(float32_t*) &Z);
}


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
