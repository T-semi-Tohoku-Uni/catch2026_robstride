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
#include "canrpc_stm32.h"
#include "canrpc_board.h"
#include "canrpc_debug.h"
#include "arm_board.h"
#include <math.h>
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


#define CYBER_GEAR_ID 0x7f


#define RIGHT_RS03_INDEX 0
#define LEFT_RS03_INDEX 1


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef inter_board_txheader;
FDCAN_TxHeaderTypeDef motor_txheader;
RobstrideMotor robstride_handler[2] = {0};
CyberGearMotor cybergear_base;

volatile float target_angle[3] = {0,0,2.0};
static volatile bool motors_ready = false;
static uint8_t current_seq;
static volatile uint32_t axis_rx_count[3];
static uint32_t published_rx_count[3];
static volatile bool reset_stop_queued, reset_stop_confirmed;
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
static uint8_t arm_initialization_requested;

/* Serialize the reset STOP handshake with cyclic native commands. */
static void arm_reset_monitor(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (reset_stop_queued && !reset_stop_confirmed &&
      cybergear_poll_stop_event(&cybergear_base, 0xa5U)) {
    reset_stop_confirmed = true;
    cybergear_base.adrc.active = false;
    cybergear_base.adrc.initialized = false;
  }
  __set_PRIMASK(mask);
}

static int arm_reset_prepare(void)
{
  if (!arm_initialization_requested) return 1;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  cybergear_base.preserve_adrc_on_tx_failure = true;
  if (!reset_stop_queued) {
    /* Only cyclic NO_TX_EVENTS frames precede the first reset STOP. */
    (void)cybergear_poll_stop_event(&cybergear_base, 0xa5U);
    reset_stop_queued = cybergear_request_stop_event(&cybergear_base, 0xa5U);
  }
  arm_reset_monitor();
  int prepared = reset_stop_confirmed;
  __set_PRIMASK(mask);
  return prepared;
}

static void arm_reset_hold(void)
{
  arm_reset_monitor();
  if (!reset_stop_confirmed) {
    cybergear_base.preserve_adrc_on_tx_failure = true;
    cybergear_control_position_adrc(&cybergear_base, target_angle[0]);
  }
}

static void arm_control_enable(void);
static void arm_native_poll(void);
static int arm_target_rpc(uint8_t,const uint8_t *,uint8_t,uint8_t *,uint8_t *);
#include "arm_boot.h"
#include "arm_control.h"
#include "arm_debug.h"

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
      ++axis_rx_count[0];
      continue;
    }

    const uint8_t motor_id = (uint8_t)(robstride_get_area_2(rxheader.Identifier) & 0xffU);
    for (uint32_t i = 0; i < (sizeof(robstride_handler) / sizeof(robstride_handler[0])); i++)
    {
      if (robstride_handler[i].motor_id == motor_id)
      {
        robstride_parse_feedback(rxheader.Identifier, rxdata, &robstride_handler[i].feedback);
        ++axis_rx_count[i == LEFT_RS03_INDEX ? 1U : 2U];
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

    rpc_management(&rxheader, rxdata);

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
    rpc_poll();
    arm_native_poll();
    if (!arm_boot_complete || !motors_ready) return;
    if (rpc_state != CANRPC_READY) {
      arm_reset_monitor();
      static uint8_t reset_hold_phase;
      if (++reset_hold_phase >= 10U) {
        reset_hold_phase = 0U;
        arm_reset_hold();
      }
      return;
    }
    static uint8_t control_phase = 0U;

    /* Stagger three commands across 1 ms ticks; each motor runs at 10 ms. */
    switch (control_phase)
    {
      case 0U:
        cybergear_control_position_adrc(&cybergear_base, target_angle[0]);
        break;
      case 1U:
        robstride_set_position(&robstride_handler[RIGHT_RS03_INDEX], target_angle[2] - 1.884f);
        break;
      case 2U:
        robstride_set_position(&robstride_handler[LEFT_RS03_INDEX], -target_angle[1] - 1.0f);
        break;
      default:
        break;
    }

    control_phase++;
    if (control_phase < 10U)
    {
      return;
    }
    control_phase = 0U;

    for (unsigned i = 0; i < 3U; ++i)
      if (axis_rx_count[i] == published_rx_count[i]) return;
    uint32_t counts[3];
    for (unsigned i = 0; i < 3U; ++i) counts[i] = axis_rx_count[i];
    float send_angles[3];
    uint8_t txdata[12];
    send_angles[2] = robstride_handler[RIGHT_RS03_INDEX].feedback.position_rad + 1.884;
    send_angles[1] = -(robstride_handler[LEFT_RS03_INDEX].feedback.position_rad + 1.0);
    send_angles[0] = cybergear_base.feedback.position_rad;
    for (unsigned i = 0; i < 3U; ++i) canrpc_put_float(txdata + 4U * i, send_angles[i]);
    if (rpc_send(canrpc_publish_id(CANRPC_SUBJECT_CURRENT, CANRPC_NODE_ARM, current_seq, CANRPC_PROJECT_CURRENT_PRIORITY),
                 FDCAN_DLC_BYTES_12, txdata)) {
      current_seq = (current_seq + 1U) & 31U;
      for (unsigned i = 0; i < 3U; ++i) published_rx_count[i] = counts[i];
    }
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
  debug_uart = &huart2;
  debug_stage = "board-preflight";
  printf("boot node=arm uart=%lu\r\n", (unsigned long)huart2.Init.BaudRate);
  if (!arm_board_config_valid(&hfdcan3, &htim6)) Error_Handler();
  if (!canrpc_board_valid(&hfdcan1, 0)) Error_Handler();
  debug_stage = "rpc-entropy";
  if (!rpc_init(&hfdcan1, CANRPC_NODE_ARM)) Error_Handler();
  rpc_prepare_reset = arm_reset_prepare;
  printf("identity node=%u boot=", (unsigned)rpc_node);
  for (unsigned i = 0; i < sizeof(rpc_boot); ++i) printf("%02x", (unsigned)rpc_boot[i]);
  printf("\r\n");
  debug_stage = "can-start";
  if (inter_board_CAN_RxTxSettings_init(&inter_board_txheader) != HAL_OK) Error_Handler();
  if (!canrpc_board_valid(&hfdcan1, 1)) Error_Handler();
  rpc_poll();
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
  debug_stage = "motor-can-start";
  if(motor_CAN_RxTxSettings_init(&motor_txheader)!=HAL_OK) Error_Handler();
  rpc_service_handler=arm_rpc;
  rpc_service_start=arm_control_start;
  arm_boot_initialize();
  rpc_ready();
  debug_stage = "running";
  printf("boot stage=running rpc=%u\r\n", (unsigned)rpc_state);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t debug_last_ms = 0U;
  uint8_t debug_last_step = 255U, debug_last_state = 255U;
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    arm_reset_monitor();
    debug_drain();
    static uint32_t debug_error_last_ms, debug_error_last_count;
    uint32_t error_now = HAL_GetTick();
    if ((uint32_t)(error_now - debug_error_last_ms) >= 1000U &&
        debug_error_last_count != debug_can_tx_errors) {
      debug_error_last_ms = error_now; debug_error_last_count = debug_can_tx_errors;
      printf("can legacy_tx_error_count=%lu\r\n", (unsigned long)debug_error_last_count);
    }
    static uint32_t debug_can_last_ms, debug_last_psr = UINT32_MAX, debug_last_ecr = UINT32_MAX;
    uint32_t can_now = HAL_GetTick();
    if ((uint32_t)(can_now - debug_can_last_ms) >= 1000U) {
      debug_can_last_ms = can_now;
      /* PSR LEC/DLEC are read-to-clear diagnostics, not recovery actions. */
      uint32_t psr = hfdcan1.Instance->PSR, ecr = hfdcan1.Instance->ECR;
      if (psr != debug_last_psr || ecr != debug_last_ecr) {
        debug_last_psr = psr; debug_last_ecr = ecr;
        printf("can bus=rpc psr=%08lx ecr=%08lx busoff=%u tec=%lu rec=%lu\r\n",
               (unsigned long)psr, (unsigned long)ecr, (unsigned)((psr >> 7U) & 1U),
               (unsigned long)(ecr & 255U), (unsigned long)((ecr >> 8U) & 127U));
      }
    }
    static uint32_t debug_native_last_ms, debug_native_psr = UINT32_MAX, debug_native_ecr = UINT32_MAX;
    uint32_t native_now = HAL_GetTick();
    if ((uint32_t)(native_now - debug_native_last_ms) >= 1000U) {
      debug_native_last_ms = native_now;
      /* PSR LEC/DLEC are read-to-clear diagnostics, not recovery actions. */
      uint32_t psr = hfdcan3.Instance->PSR, ecr = hfdcan3.Instance->ECR;
      if (psr != debug_native_psr || ecr != debug_native_ecr) {
        debug_native_psr = psr; debug_native_ecr = ecr;
        printf("can bus=motor psr=%08lx ecr=%08lx busoff=%u tec=%lu rec=%lu\r\n",
               (unsigned long)psr, (unsigned long)ecr, (unsigned)((psr >> 7U) & 1U),
               (unsigned long)(ecr & 255U), (unsigned long)((ecr >> 8U) & 127U));
      }
    }
    rpc_reset_poll();
    rpc_application_poll();
    arm_boot_poll();
    arm_control_poll();
    const uint32_t now = HAL_GetTick();
    if (debug_last_step != arm_boot_step || debug_last_state != rpc_state ||
        (uint32_t)(now - debug_last_ms) >= 1000U) {
      debug_last_step = arm_boot_step; debug_last_state = rpc_state; debug_last_ms = now;
      uint8_t axes[36];
      uint32_t mask = __get_PRIMASK();
      __disable_irq();
      if (arm_boot_failed) memcpy(axes, arm_fault_snapshot, sizeof(axes));
      else for (unsigned i = 0; i < 3U; ++i) arm_axis_record(axes + i * 12U, i, now);
      __set_PRIMASK(mask);
      printf("arm rpc=%u stage=%u boot_complete=%u reason=%u axis=%u drop=%lu\r\n",
             (unsigned)rpc_state, (unsigned)(arm_initialization_requested ? arm_boot_step : 31U), (unsigned)arm_boot_complete,
             (unsigned)arm_failure_reason, (unsigned)arm_failure_axis, (unsigned long)debug_dropped);
      for (unsigned i = 0; i < 3U; ++i) {
        const uint8_t *a = axes + i * 12U;
        printf("axis=%u id=%u mode=%u fault=%u flags=%u age_ms=%lu cmd=%u response=%u\r\n",
               i, a[3], a[0], a[1], a[2], (unsigned long)canrpc_get_u32(a + 4), a[8], a[9]);
      }
    }
    arm_debug_angles_poll();
    /* Preserve the control cadence while servicing UART without waiting for TXE. */
    uint32_t wait_started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - wait_started) < 10U) debug_drain();

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
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
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
  htim6.Init.Period = 999;
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
  (void)file;
  return debug_write(ptr, len);
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
  if (__get_IPSR() == 0U && __get_PRIMASK() == 0U)
    printf("fatal stage=%s\r\n", debug_stage);
  __disable_irq();
  while (1)
  {
    debug_drain();
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
