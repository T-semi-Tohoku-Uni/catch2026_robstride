/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "can_init.h"
#include "robstride_app.h"
#include "cybergear.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HOST_ID 0xfe

#define RIGHT_RS03_ID 3
#define LEFT_RS03_ID 4

#define EL05_ID 5

#define CYBER_GEAR_ID 0x7f


#define RIGHT_RS03_INDEX 0
#define LEFT_RS03_INDEX 1

#define EL05_INDEX 2

#define CANID 0x200
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef inter_board_txheader;
FDCAN_TxHeaderTypeDef motor_txheader;
RobstrideMotor robstride_handler[3] = {0};
CyberGearMotor cybergear_base;

volatile float target_angle[4] = {0,0,2.0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */
void u8_to_int(uint8_t *req, int32_t *des, uint32_t uint8_len);
void u8_to_float(uint8_t *req, float *des, uint32_t uint8_len);
void float_to_u8(float *req, uint8_t *des, uint32_t float_len);
void int_to_u8(int32_t *req, uint8_t *des, uint32_t int_len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool cybergear_homing(void)
{
  // 1. 速度制御モードに変更して有効化
  if (!cybergear_set_run_mode(&cybergear_base, CYBERGEAR_RUN_MODE_SPEED)) 
  {
    return false;
  }
  HAL_Delay(10);
  if (!cybergear_enable(&cybergear_base)) 
  {
    return false;
  }

  GPIO_PinState initial_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

  cybergear_set_velocity(&cybergear_base, 1.0f);

  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == initial_state)
  {
    HAL_Delay(10); 
  }

  cybergear_stop(&cybergear_base);
  HAL_Delay(100); 


  cybergear_set_zero(&cybergear_base);
  HAL_Delay(10);


  if (!cybergear_set_run_mode(&cybergear_base, CYBERGEAR_RUN_MODE_OPERATION)) 
  {
    return false;
  }

  HAL_Delay(10);
  return cybergear_enable(&cybergear_base);
}

bool robstride_init(void)
{
  robstride_handler[LEFT_RS03_INDEX].host_id  = HOST_ID;
  robstride_handler[RIGHT_RS03_INDEX].host_id = HOST_ID;
  robstride_handler[EL05_INDEX].host_id       = HOST_ID;
  robstride_handler[LEFT_RS03_INDEX].motor_id  = LEFT_RS03_ID;
  robstride_handler[RIGHT_RS03_INDEX].motor_id = RIGHT_RS03_ID;
  robstride_handler[EL05_INDEX].motor_id       = EL05_ID;

  robstride_handler[LEFT_RS03_INDEX].run_mode  = POSITION_PP;
  robstride_handler[RIGHT_RS03_INDEX].run_mode = POSITION_PP;
  robstride_handler[EL05_INDEX].run_mode       = POSITION_PP;

  robstride_handler[LEFT_RS03_INDEX].txheader  = motor_txheader;
  robstride_handler[RIGHT_RS03_INDEX].txheader = motor_txheader;
  robstride_handler[EL05_INDEX].txheader       = motor_txheader;

  for (uint32_t i = 0; i < (sizeof(robstride_handler) / sizeof(robstride_handler[0])); ++i)
  {
    // robstride_stop(&robstride_handler[i]);
    // HAL_Delay(10);

    // // 2. 現在位置をゼロ点に設定する
    // robstride_set_zero(&robstride_handler[i]);
    // HAL_Delay(10);

    if (!robstride_start_position_pp_mode(&robstride_handler[i], 10, 1, 10))
    {
      return false;
    }
  }

  return true;
}

bool cybergear_base_init(void)
{
  if (!cybergear_init(
    &cybergear_base,
    &hfdcan3,
    CYBER_GEAR_ID,
    HOST_ID
  ))
  {
    return false;
  }

  if (!cybergear_stop(&cybergear_base))
  {
    return false;
  }
  HAL_Delay(10);


  // if (!cybergear_set_zero(&cybergear_base))
  //   return false;
  // HAL_Delay(10);

  if (!cybergear_set_run_mode(
    &cybergear_base,
    CYBERGEAR_RUN_MODE_OPERATION
  ))
  {
    return false;
  }
  HAL_Delay(10);

  return cybergear_enable(&cybergear_base);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (hfdcan->Instance != FDCAN3 ||
      (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    FDCAN_RxHeaderTypeDef rxheader;
    uint8_t rxdata[8];
    //printf("0x%03lX\r\n",rxheader.Identifier);

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxheader, rxdata) != HAL_OK)
    {
      break;
    }

    if (rxheader.IdType != FDCAN_EXTENDED_ID ||
        rxheader.RxFrameType != FDCAN_DATA_FRAME ||
        rxheader.DataLength != FDCAN_DLC_BYTES_8 ||
        robstride_get_communication_type(rxheader.Identifier) != FeedbackId ||
        robstride_get_destination_id(rxheader.Identifier) != HOST_ID)
    {
      continue;
    }

    if (cybergear_parse_feedback(
      &cybergear_base,
      rxheader.Identifier,
      rxdata
    ))
    {
      continue;
    }

    const uint8_t motor_id = (uint8_t)(robstride_get_area_2(rxheader.Identifier) & 0xffU);
    for (uint32_t i = 0; i < (sizeof(robstride_handler) / sizeof(robstride_handler[0])); i++)
    {
      if (robstride_handler[i].motor_id == motor_id)
      {
        robstride_parse_feedback(rxheader.Identifier, rxdata, &robstride_handler[i].feedback);
        break;
      }
    }
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if (hfdcan->Instance != FDCAN1 ||
      (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
  {
    FDCAN_RxHeaderTypeDef rxheader;
    uint8_t rxdata[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxheader, rxdata) != HAL_OK)
    {
      break;
    }

    switch (rxheader.Identifier)
    {
      case CANID: 
        float received_floats[4];
        u8_to_float(rxdata, received_floats, 16);
        for (int i = 0;i<4;i++){
          target_angle[i] = received_floats[i];
        }

        break;
      default:
        // printf("unknown CAN ID received: 0x%03lX\r\n", RxHeader.Identifier); // printf should be commented out within Callback
        break;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == limit_sw_0_Pin) // right
  {

  }
  else if (GPIO_Pin == limit_sw_1_Pin) // left
  {

  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim == &htim6) {
    float send_angles[4] = {0};
    uint8_t txdata[16] = {0};

    // 1. 各モーターのフィードバックから現在角度(position_rad)を取得
    send_angles[2] = robstride_handler[RIGHT_RS03_INDEX].feedback.position_rad + 1.884;
    send_angles[1] = -(robstride_handler[LEFT_RS03_INDEX].feedback.position_rad + 1.0);
    send_angles[3] = robstride_handler[EL05_INDEX].feedback.position_rad;
    send_angles[0] = cybergear_base.feedback.position_rad;

    // 2. float (4つ) を uint8_t配列 (16バイト) に変換
    float_to_u8(send_angles, txdata, 4);

    // 3. 送信設定の変更 (16バイトのCAN FDフレームとして送信)
    inter_board_txheader.Identifier = 0x210; // 必要に応じてIDを変更してください
    inter_board_txheader.DataLength = FDCAN_DLC_BYTES_16; 

    // 4. FDCAN1から送信
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &inter_board_txheader, txdata);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  setbuf(stdout, NULL);

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
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
  inter_board_CAN_RxTxSettings_init(&inter_board_txheader);
  motor_CAN_RxTxSettings_init(&motor_txheader);
  if (!robstride_init())
  {
    Error_Handler();
  }
  if (!cybergear_base_init())
  {
    Error_Handler();
  }

  if (!cybergear_homing())
  {
    printf("CyberGear Homing Failed\r\n");
    Error_Handler();
  }
  printf("a\r\n");
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float target_angle1 = 0.785;
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    robstride_set_position(&robstride_handler[RIGHT_RS03_INDEX], (target_angle[2]- 1.884));
    HAL_Delay(1);
    robstride_set_position(&robstride_handler[LEFT_RS03_INDEX], (-target_angle[1]-1.0f));
    HAL_Delay(1);
    robstride_set_position(&robstride_handler[EL05_INDEX], 0.0f);
    HAL_Delay(1);
    target_angle1 = -target_angle1; 
    float target_pos = target_angle[0];
    float target_vel = 0.0f; // 目標速度は0 (位置決め)
    float kp = 8.0f;        // 位置ゲイン (バネの硬さ) 範囲: 0.0 ~ 500.0
    float kd = 4.0f;         // 速度ゲイン (ダンピング/粘性) 範囲: 0.0 ~ 5.0
    float ff_torque = 0.0f;  // フィードフォワードトルクは0

    cybergear_control(&cybergear_base, target_pos, target_vel, kp, kd, ff_torque);
    // printf("Right: %f, Left: %f, EL: %f\r\n",
    //        (target_angle[2]- 2.878),
    //        (-target_angle[1]-1.0f),
    //        0.0f);
    
    HAL_Delay(10);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 1;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : limit_sw_0_Pin limit_sw_1_Pin */
  GPIO_InitStruct.Pin = limit_sw_0_Pin|limit_sw_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : limit_Pin */
  GPIO_InitStruct.Pin = limit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(limit_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void u8_to_float(uint8_t *req, float *des, uint32_t uint8_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for(int i = 0; i < uint8_len/4; i++){
    uint32_t f32_u32 = ((req[i*4] << 24) | (req[i*4+1] << 16) | (req[i*4+2] << 8) | (req[i*4+3]));
    union IntAndFloat target;
    target.ival = f32_u32;
    des[i] = target.fval;
  }
}

void u8_to_int(uint8_t *req, int32_t *des, uint32_t uint8_len)
{
  for(int i = 0; i < uint8_len/4; i++){
    uint32_t u32 = ((req[i*4] << 24) | (req[i*4+1] << 16) | (req[i*4+2] << 8) | (req[i*4+3]));
    des[i] = (int32_t)u32;
  }
}

void float_to_u8(float *req, uint8_t *des, uint32_t float_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for (int i = 0; i < float_len; i++)
  {
    union IntAndFloat target;
    target.fval = req[i];
    uint32_t val = target.ival;
    des[i*4    ] = (uint8_t)((val >> 24) & 0xff);
    des[i*4 + 1] = (uint8_t)((val >> 16) & 0xff);
    des[i*4 + 2] = (uint8_t)((val >>  8) & 0xff);
    des[i*4 + 3] = (uint8_t)((val      ) & 0xff);
  }
}

void int_to_u8(int32_t *req, uint8_t *des, uint32_t int_len)
{
  for (int i = 0; i < int_len; i++)
  {
    uint32_t val = (uint32_t)req[i];
    des[i*4    ] = (uint8_t)((val >> 24) & 0xff);
    des[i*4 + 1] = (uint8_t)((val >> 16) & 0xff);
    des[i*4 + 2] = (uint8_t)((val >>  8) & 0xff);
    des[i*4 + 3] = (uint8_t)((val      ) & 0xff);
  }
}

int _write(int file,char *ptr,int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 10);
  return len;
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
#ifdef USE_FULL_ASSERT
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
