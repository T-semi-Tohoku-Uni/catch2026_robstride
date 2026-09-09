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
#include <math.h>
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
#define ANGLE_FEEDBACK_CANID 0x210
#define CYBERGEAR_STARTUP_TIMEOUT_MS 3000U
#define CYBERGEAR_HOMING_REVERSE_ANGLE_RAD 1.0471975512f /* 60 degrees */
#define CYBERGEAR_HOMING_SLOW_SPEED_RAD_S 0.4f
#define CYBERGEAR_HOMING_CLEARANCE_RAD 0.034906585f /* 2 degrees beyond each edge */
#define CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS 100U
#define CYBERGEAR_HOMING_PHASE_TIMEOUT_MS 15000U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef inter_board_txheader;
FDCAN_TxHeaderTypeDef motor_txheader;
RobstrideMotor robstride_handler[3] = {
  {.motor_id = RIGHT_RS03_ID, .host_id = HOST_ID},
  {.motor_id = LEFT_RS03_ID, .host_id = HOST_ID},
  {.motor_id = EL05_ID, .host_id = HOST_ID}
};
CyberGearMotor cybergear_base;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */
void float_to_u8(const float *req, uint8_t *des, uint32_t float_len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool cybergear_wait_for_stopped_feedback(void)
{
  const uint32_t started_ms = HAL_GetTick();
  uint32_t last_probe_ms = started_ms - 100U;
  while ((uint32_t)(HAL_GetTick() - started_ms) < CYBERGEAR_STARTUP_TIMEOUT_MS)
  {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const CyberGearFeedback feedback = cybergear_base.feedback;
    const uint32_t now_ms = HAL_GetTick();
    __set_PRIMASK(interrupt_mask);
    if (feedback.online && feedback.mode == 0U &&
        (uint32_t)(now_ms - feedback.last_received_ms) < CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS)
    {
      return feedback.fault_flags == 0U;
    }
    if ((uint32_t)(now_ms - last_probe_ms) >= 100U)
    {
      (void)cybergear_stop(&cybergear_base);
      last_probe_ms = now_ms;
    }
    HAL_Delay(10);
  }
  return false;
}

static bool cybergear_homing_feedback(CyberGearFeedback *feedback)
{
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  *feedback = cybergear_base.feedback;
  const uint32_t now_ms = HAL_GetTick();
  __set_PRIMASK(interrupt_mask);
  const bool valid = feedback->online && feedback->fault_flags == 0U &&
      isfinite(feedback->position_rad) && isfinite(feedback->velocity_rad_s) &&
      (uint32_t)(now_ms - feedback->last_received_ms) < CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS;
  if (!valid)
  {
    cybergear_stop(&cybergear_base);
    printf("CG home feedback: online=%u fault=%u age=%lu\r\n",
           (unsigned int)feedback->online, (unsigned int)feedback->fault_flags,
           (unsigned long)(now_ms - feedback->last_received_ms));
  }
  return valid;
}

/* Continue past the edge so the next measurement approaches from the other side. */
static bool cybergear_homing_clear_edge(float direction)
{
  CyberGearFeedback feedback;
  if (!cybergear_homing_feedback(&feedback))
  {
    return false;
  }
  const float start_rad = feedback.position_rad;
  const uint32_t started_ms = HAL_GetTick();
  while ((feedback.position_rad - start_rad) * direction < CYBERGEAR_HOMING_CLEARANCE_RAD)
  {
    if ((uint32_t)(HAL_GetTick() - started_ms) >= CYBERGEAR_HOMING_PHASE_TIMEOUT_MS)
    {
      cybergear_stop(&cybergear_base);
      printf("CG home clearance timeout\r\n");
      return false;
    }
    if (!cybergear_set_velocity(&cybergear_base, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
    {
      cybergear_stop(&cybergear_base);
      printf("CG home clearance TX failed\r\n");
      return false;
    }
    HAL_Delay(10);
    if (!cybergear_homing_feedback(&feedback))
    {
      return false;
    }
  }
  return true;
}

static bool cybergear_homing_measure_edge(float direction, float *edge_rad)
{
  /* Clearance can cross a narrow detection region completely. Use the actual
     state here rather than assuming the state seen at the previous edge. */
  const GPIO_PinState from_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  if (!cybergear_set_velocity(&cybergear_base, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
  {
    cybergear_stop(&cybergear_base);
    printf("CG home edge TX failed\r\n");
    return false;
  }
  const uint32_t started_ms = HAL_GetTick();
  bool edge_pending = false;
  uint32_t edge_started_ms = 0U;
  HAL_Delay(10);
  while ((uint32_t)(HAL_GetTick() - started_ms) < CYBERGEAR_HOMING_PHASE_TIMEOUT_MS)
  {
    CyberGearFeedback feedback;
    if (!cybergear_homing_feedback(&feedback))
    {
      return false;
    }
    /* Ignore an edge crossed by residual motion in the previous direction. */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != from_state &&
        feedback.velocity_rad_s * direction > 0.0f)
    {
      if (!edge_pending)
      {
        /* Keep the first crossing angle, then confirm the switch for 20 ms. */
        *edge_rad = feedback.position_rad;
        edge_started_ms = HAL_GetTick();
        edge_pending = true;
      }
      if ((uint32_t)(HAL_GetTick() - edge_started_ms) >= 20U)
      {
        return true;
      }
    }
    else
    {
      edge_pending = false;
    }
    if (!cybergear_set_velocity(&cybergear_base, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
    {
      cybergear_stop(&cybergear_base);
      printf("CG home edge TX failed\r\n");
      return false;
    }
    HAL_Delay(10);
  }
  cybergear_stop(&cybergear_base);
  printf("CG home edge timeout\r\n");
  return false;
}

static bool cybergear_homing_move_to_center(float center_rad)
{
  if (!isfinite(center_rad))
  {
    cybergear_stop(&cybergear_base);
    printf("CG home center invalid target\r\n");
    return false;
  }
  const uint32_t started_ms = HAL_GetTick();
  CyberGearFeedback feedback = {0};
  if (!cybergear_homing_feedback(&feedback))
  {
    return false;
  }
  const float direction = (center_rad >= feedback.position_rad) ? 1.0f : -1.0f;
  const float speed_rad_s = direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S;
  while ((uint32_t)(HAL_GetTick() - started_ms) < CYBERGEAR_HOMING_PHASE_TIMEOUT_MS)
  {
    if (!cybergear_homing_feedback(&feedback))
    {
      return false;
    }
    const float error_rad = center_rad - feedback.position_rad;
    /* Finish on reaching/crossing the average, without hunting or settling. */
    if (error_rad * direction <= 0.0f)
    {
      return true;
    }
    if (!cybergear_set_velocity(&cybergear_base, speed_rad_s))
    {
      cybergear_stop(&cybergear_base);
      printf("CG home center TX failed\r\n");
      return false;
    }
    HAL_Delay(10);
  }
  cybergear_stop(&cybergear_base);
  printf("CG home center timeout: pos=%.5f target=%.5f rad\r\n",
         (double)feedback.position_rad, (double)center_rad);
  printf("CG home center vel=%.5f cmd=%.5f rad/s\r\n",
         (double)feedback.velocity_rad_s, (double)speed_rad_s);
  return false;
}

static bool cybergear_homing(void)
{
  // 1. 速度制御モードに変更して有効化
  if (!cybergear_set_run_mode(&cybergear_base, CYBERGEAR_RUN_MODE_SPEED)) 
  {
    return false;
  }
  HAL_Delay(10);
  if (!cybergear_set_velocity(&cybergear_base, 0.0f))
  {
    cybergear_stop(&cybergear_base);
    return false;
  }
  const uint32_t feedback_wait_started_ms = HAL_GetTick();
  uint32_t last_enable_ms = feedback_wait_started_ms;
  if (!cybergear_enable(&cybergear_base)) 
  {
    return false;
  }

  /* Obtain a fresh starting angle before beginning the homing motion. */
  CyberGearFeedback feedback;
  while (1)
  {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    feedback = cybergear_base.feedback;
    const uint32_t now_ms = HAL_GetTick();
    __set_PRIMASK(interrupt_mask);
    if (feedback.online && feedback.fault_flags != 0U)
    {
      cybergear_stop(&cybergear_base);
      return false;
    }
    if (feedback.online && feedback.mode == 2U &&
        (uint32_t)(now_ms - feedback.last_received_ms) < CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS &&
        (uint32_t)(now_ms - feedback.last_received_ms) <=
        (uint32_t)(now_ms - feedback_wait_started_ms))
    {
      break;
    }
    if ((uint32_t)(now_ms - feedback_wait_started_ms) >= CYBERGEAR_STARTUP_TIMEOUT_MS)
    {
      cybergear_stop(&cybergear_base);
      return false;
    }
    /* Retry a lost enable response while the speed reference is still zero. */
    if ((uint32_t)(now_ms - last_enable_ms) >= 100U)
    {
      if (!cybergear_enable(&cybergear_base))
      {
        cybergear_stop(&cybergear_base);
        return false;
      }
      last_enable_ms = now_ms;
    }
    HAL_Delay(10);
  }
  if (feedback.fault_flags != 0U)
  {
    cybergear_stop(&cybergear_base);
    return false;
  }

  const float start_position_rad = feedback.position_rad;
  bool reversed = false;
  float direction = 1.0f;
  const uint32_t search_started_ms = HAL_GetTick();
  const GPIO_PinState initial_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

  if (!cybergear_set_velocity(&cybergear_base, direction))
  {
    cybergear_stop(&cybergear_base);
    return false;
  }

  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == initial_state)
  {
    if (!cybergear_homing_feedback(&feedback) ||
        (uint32_t)(HAL_GetTick() - search_started_ms) >= CYBERGEAR_HOMING_PHASE_TIMEOUT_MS)
    {
      cybergear_stop(&cybergear_base);
      return false;
    }

    /* Check the amount moved in either encoder direction. A signed positive
       delta alone never reaches the threshold when the angle decreases. */
    const float moved_rad = fabsf(feedback.position_rad - start_position_rad);
    const bool reverse_now = !reversed &&
        moved_rad >= CYBERGEAR_HOMING_REVERSE_ANGLE_RAD &&
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == initial_state;
    if (reverse_now)
    {
      direction = -direction;
      reversed = true;
    }
    if (!cybergear_set_velocity(&cybergear_base, direction))
    {
      cybergear_stop(&cybergear_base);
      return false;
    }
    if (reverse_now)
    {
      printf("CG home reverse: start=%.3f pos=%.3f rad\r\n",
             (double)start_position_rad, (double)feedback.position_rad);
      printf("CG home reverse: moved=%.3f rad cmd=%.1f rad/s\r\n",
             (double)moved_rad, (double)direction);
    }
    HAL_Delay(10); 
  }

  /* Discard the fast crossing; measure switch crossings at low speed both ways. */
  float reverse_edge_rad;
  float forward_edge_rad;
  /* clear_edge sends the slow command; avoid two back-to-back CAN writes. */
  if (!cybergear_homing_clear_edge(direction))
  {
    cybergear_stop(&cybergear_base);
    printf("CG home failed: first clearance\r\n");
    return false;
  }
  if (!cybergear_homing_measure_edge(-direction, &reverse_edge_rad))
  {
    cybergear_stop(&cybergear_base);
    printf("CG home failed: reverse edge\r\n");
    return false;
  }
  if (!cybergear_homing_clear_edge(-direction))
  {
    cybergear_stop(&cybergear_base);
    printf("CG home failed: reverse clearance\r\n");
    return false;
  }
  if (!cybergear_homing_measure_edge(direction, &forward_edge_rad))
  {
    cybergear_stop(&cybergear_base);
    printf("CG home failed: forward edge\r\n");
    return false;
  }
  const float center_rad = 0.5f * (reverse_edge_rad + forward_edge_rad);
  if (!cybergear_homing_move_to_center(center_rad))
  {
    cybergear_stop(&cybergear_base);
    printf("CG home failed: center move\r\n");
    return false;
  }

  if (!cybergear_stop(&cybergear_base))
  {
    return false;
  }
  /* Allow STOP to be sent before setting zero; no position/settling check. */
  HAL_Delay(10);

  if (!cybergear_set_zero(&cybergear_base))
  {
    return false;
  }
  HAL_Delay(10);

  printf("CG home edges=%.4f,%.4f center=%.4f rad\r\n",
         (double)reverse_edge_rad, (double)forward_edge_rad, (double)center_rad);
  /* Remain disabled after homing; cyclic STOP requests only read angles. */
  return true;
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
    uint8_t rxdata[64];
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
    if (cybergear_parse_feedback(&cybergear_base, rxheader.Identifier, rxdata))
    {
      continue;
    }

    const uint8_t motor_id = (uint8_t)(robstride_get_area_2(rxheader.Identifier) & 0xffU);
    for (uint32_t i = 0; i < 3U; ++i)
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

  /* Drain incoming board commands without changing motor state. */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
  {
    FDCAN_RxHeaderTypeDef rxheader;
    uint8_t rxdata[64];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxheader, rxdata) != HAL_OK)
    {
      break;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim != &htim6)
  {
    return;
  }

  static uint8_t feedback_phase = 0U;
  /* STOP (type 4, zero payload) returns type 2 feedback while disabled.
     Stagger requests over 1 ms ticks; each motor is polled every 10 ms.
     Failed enqueue attempts are retried on the next cycle. */
  switch (feedback_phase)
  {
    case 0U:
      (void)cybergear_stop(&cybergear_base);
      break;
    case 1U:
      (void)robstride_stop(&robstride_handler[RIGHT_RS03_INDEX]);
      break;
    case 2U:
      (void)robstride_stop(&robstride_handler[LEFT_RS03_INDEX]);
      break;
    case 3U:
      (void)robstride_stop(&robstride_handler[EL05_INDEX]);
      break;
    default:
      break;
  }

  if (++feedback_phase < 10U)
  {
    return;
  }
  feedback_phase = 0U;

  float send_angles[4];
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  send_angles[0] = cybergear_base.feedback.position_rad;
  send_angles[1] = -(robstride_handler[LEFT_RS03_INDEX].feedback.position_rad + 1.0f);
  send_angles[2] = robstride_handler[RIGHT_RS03_INDEX].feedback.position_rad + 1.884f;
  send_angles[3] = -(robstride_handler[EL05_INDEX].feedback.position_rad + 2.963f);
  __set_PRIMASK(interrupt_mask);

  /* Preserve the board protocol: four big-endian float angles in radians. */
  uint8_t txdata[16];
  float_to_u8(send_angles, txdata, 4U);
  (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &inter_board_txheader, txdata);
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
  /* Configure software handles before enabling CAN reception. */
  if (!cybergear_init(&cybergear_base, &hfdcan3, CYBER_GEAR_ID, HOST_ID))
  {
    Error_Handler();
  }
  if (inter_board_CAN_RxTxSettings_init(&inter_board_txheader) != HAL_OK ||
      motor_CAN_RxTxSettings_init(&motor_txheader) != HAL_OK)
  {
    Error_Handler();
  }
  for (uint32_t i = 0; i < 3U; ++i)
  {
    robstride_handler[i].txheader = motor_txheader;
  }
  inter_board_txheader.Identifier = ANGLE_FEEDBACK_CANID;
  inter_board_txheader.DataLength = FDCAN_DLC_BYTES_16;

  /* Only CyberGear moves for homing. Keep the other motors disabled. */
  for (uint32_t i = 0; i < 3U; ++i)
  {
    (void)robstride_stop(&robstride_handler[i]);
    HAL_Delay(10);
  }
  const bool homed = cybergear_wait_for_stopped_feedback() && cybergear_homing();
  /* Also stop after any homing failure. Do not reset and restart homing. */
  (void)cybergear_stop(&cybergear_base);
  HAL_Delay(10);

  /* Start cyclic STOP/angle reporting after homing to avoid interrupting it. */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    /* Keep retrying STOP if the reporting timer cannot start. */
    while (1)
    {
      (void)cybergear_stop(&cybergear_base);
      HAL_Delay(10);
    }
  }
  printf("CG homing %s; motors disabled, angle feedback only\r\n",
         homed ? "complete" : "failed");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
void float_to_u8(const float *req, uint8_t *des, uint32_t float_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for (uint32_t i = 0; i < float_len; i++)
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

int _write(int file,char *ptr,int len)
{
  (void)file;
  /* Blocking UART timeouts need SysTick, which cannot preempt our CAN ISR. */
  if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
  {
    return len;
  }
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
    NVIC_SystemReset();
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
