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
#include "cybergear_calibration_app.h"
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
#define MOTOR_INIT_CANID 0x500
#define CYBERGEAR_DEBUG_INTERVAL_MS 200U
#define EL05_DEBUG_INTERVAL_MS 200U
#define MOTOR_INIT_FEEDBACK_TIMEOUT_MS 3000U
#define CYBERGEAR_HOMING_REVERSE_ANGLE_RAD 1.0471975512f /* 60 degrees */
#define CYBERGEAR_HOMING_SLOW_SPEED_RAD_S 0.4f
#define CYBERGEAR_HOMING_CLEARANCE_RAD 0.034906585f /* 2 degrees beyond each edge */
#define CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS 100U
#define CYBERGEAR_HOMING_PHASE_TIMEOUT_MS 15000U
#define MOTOR_RETURN_IGNORE_MS 2000U
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
#if CYBERGEAR_CALIBRATION_BUILD
static CgCalApp cybergear_calibration_app;
#endif

volatile float target_angle[4] = {0,0,2.0,0};
#if !CYBERGEAR_CALIBRATION_BUILD
static volatile bool el05_initializing = false;
static volatile bool motor_init_requested = false;
static volatile bool motors_running = false;
static volatile bool motor_return_active = false;
static volatile uint32_t motor_return_started_ms = 0U;
#endif
static volatile bool motor_init_feedback_received = false;
static volatile uint32_t motor_last_feedback_ms = 0U;
static volatile uint32_t motor_can_rx_count = 0U;
static volatile uint32_t motor_can_last_rx_id = 0U;
static volatile uint32_t motor_can_last_rx_dlc = 0U;
static volatile uint32_t motor_can_last_rx_id_type = 0U;
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
#if !CYBERGEAR_CALIBRATION_BUILD
static void motor_init_print_can_status(void)
{
  FDCAN_ProtocolStatusTypeDef status = {0};
  FDCAN_ErrorCountersTypeDef counters = {0};
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t rx_count = motor_can_rx_count;
  const uint32_t rx_id = motor_can_last_rx_id;
  const uint32_t rx_dlc = motor_can_last_rx_dlc;
  const uint32_t rx_id_type = motor_can_last_rx_id_type;
  __set_PRIMASK(interrupt_mask);
  printf("CAN3 rx=%lu last=0x%08lX dlc=%lu ext=%u\r\n",
         (unsigned long)rx_count, (unsigned long)rx_id,
         (unsigned long)rx_dlc, (unsigned int)(rx_id_type == FDCAN_EXTENDED_ID));
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan3, &status) == HAL_OK &&
      HAL_FDCAN_GetErrorCounters(&hfdcan3, &counters) == HAL_OK)
  {
    printf("CAN3 busoff=%lu lec=%lu tec=%lu rec=%lu\r\n",
           (unsigned long)status.BusOff, (unsigned long)status.LastErrorCode,
           (unsigned long)counters.TxErrorCnt, (unsigned long)counters.RxErrorCnt);
  }
  printf("CAN3 hal=0x%08lX txfree=%lu CG=0x%02X host=0x%02X\r\n",
         (unsigned long)HAL_FDCAN_GetError(&hfdcan3),
         (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3), CYBER_GEAR_ID, HOST_ID);
}

static void motor_check_feedback(void)
{
  /* Snapshot the tick with the ISR timestamp to avoid a receive-time race. */
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t last_feedback_ms = motor_last_feedback_ms;
  __set_PRIMASK(interrupt_mask);
  if ((uint32_t)(now_ms - last_feedback_ms) >= MOTOR_INIT_FEEDBACK_TIMEOUT_MS)
  {
    motor_init_print_can_status();
    printf("No motor feedback for 3 seconds; resetting MCU\r\n");
    NVIC_SystemReset();
  }
}

static void motor_init_wait_for_feedback(void)
{
  /* Also allow a response before entering an error handler or cyclic control. */
  while (!motor_init_feedback_received)
  {
    motor_check_feedback();
    HAL_Delay(10);
  }
}

static void motor_return_update(void)
{
  bool completed = false;
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  if (!motor_return_active)
  {
    __set_PRIMASK(interrupt_mask);
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - motor_return_started_ms) >= MOTOR_RETURN_IGNORE_MS)
  {
    /* Drain commands already queued during the return before accepting new ones. */
    HAL_FDCAN_RxFifo1Callback(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    motor_return_active = false;
    completed = true;
  }
  __set_PRIMASK(interrupt_mask);
  if (completed)
  {
    printf("Motor return: 2 seconds elapsed; CAN commands accepted\r\n");
  }
}

static void el05_debug_print(void)
{
  static uint32_t last_print_ms = 0U;
  const uint32_t now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - last_print_ms) < EL05_DEBUG_INTERVAL_MS)
  {
    return;
  }
  last_print_ms = now_ms;

  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const float position_rad = robstride_handler[EL05_INDEX].feedback.position_rad;
  const float target_rad = -target_angle[3] - 2.963f;
  __set_PRIMASK(interrupt_mask);

  printf("EL05 pos=%.3f target=%.3f rad\r\n",
         (double)position_rad, (double)target_rad);
}

static void cybergear_debug_print(void)
{
#if CYBERGEAR_LOG_UART
  /* Drain at most one record per main iteration; keep every line < UART timeout. */
  CyberGearLog log;
  if (!cybergear_pop_log(&cybergear_base, &log)) return;
  printf("CG t=%lu state=%u fault=%u drop=%lu\r\n",
      (unsigned long)log.timestamp_ms, (unsigned)log.state,
      (unsigned)log.fault, (unsigned long)log.drops);
  printf("CG q=%.4f qd=%.4f vd=%.3f ad=%.3f\r\n",
      (double)log.q, (double)log.qd, (double)log.vd, (double)log.ad);
  printf("CG it=%.3f id=%.3f req=%.3f cmd=%.3f\r\n",
      (double)log.i_track, (double)log.i_dist, (double)log.i_req, (double)log.i_cmd);
  printf("CG b=%.3f age=%lu dt=%lu tx=%lu/%lu\r\n",
      (double)log.b0, (unsigned long)log.rx_age_ms, (unsigned long)log.control_dt_ms,
      (unsigned long)log.tx_queued, (unsigned long)log.tx_failed);
  printf("CG lim=%u/%u/%u stop=%u/%u/%u\r\n",
      (unsigned)log.amp_limited, (unsigned)log.slew_limited,
      (unsigned)log.compensation_limited, (unsigned)log.stop_queued,
      (unsigned)log.reset_confirmed, (unsigned)log.stationary);
#endif
}
static bool cybergear_homing_feedback(CyberGearFeedback *feedback)
{
  motor_check_feedback();
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
      motor_init_print_can_status();
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

bool cybergear_homing(void)
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
    motor_check_feedback();
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    feedback = cybergear_base.feedback;
    const uint32_t now_ms = HAL_GetTick();
    __set_PRIMASK(interrupt_mask);
    if (feedback.online &&
        (uint32_t)(now_ms - feedback.last_received_ms) <=
        (uint32_t)(now_ms - feedback_wait_started_ms))
    {
      break;
    }
    if ((uint32_t)(now_ms - feedback_wait_started_ms) >= MOTOR_INIT_FEEDBACK_TIMEOUT_MS)
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
        motor_init_print_can_status();
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

  if (!cybergear_set_velocity(&cybergear_base, 1.0f))
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

    /* Reverse once if the switch has not changed after 60 degrees. */
    if (!reversed &&
        feedback.position_rad - start_position_rad >= CYBERGEAR_HOMING_REVERSE_ANGLE_RAD &&
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == initial_state)
    {
      direction = -1.0f;
      reversed = true;
    }
    if (!cybergear_set_velocity(&cybergear_base, direction))
    {
      cybergear_stop(&cybergear_base);
      return false;
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
  /* Keep stopped until all blocking motor initialization is complete. */
  return true;
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

  /* Reject incomplete machine settings before the legacy homing can energize CG. */
  if (!cybergear_config_valid(&cybergear_base.config))
  {
    printf("CG machine parameters unset: see cybergear_config.h\r\n");
    return false;
  }

  /* Establish feedback before enabling motion. A queued frame is not proof
     of delivery, so retry STOP while the motor is powering up. */
  const uint32_t wait_started_ms = HAL_GetTick();
  uint32_t last_probe_ms = wait_started_ms - 100U;
  while (1)
  {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const CyberGearFeedback feedback = cybergear_base.feedback;
    __set_PRIMASK(interrupt_mask);
    const uint32_t now_ms = HAL_GetTick();
    if (feedback.online &&
        (uint32_t)(now_ms - feedback.last_received_ms) < CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS)
    {
      if (feedback.fault_flags != 0U)
      {
        cybergear_stop(&cybergear_base);
        printf("CG startup fault=0x%02X\r\n", (unsigned int)feedback.fault_flags);
        return false;
      }
      break;
    }
    motor_check_feedback();
    if ((uint32_t)(now_ms - wait_started_ms) >= MOTOR_INIT_FEEDBACK_TIMEOUT_MS)
    {
      printf("CG startup: no feedback from ID 0x%02X\r\n", CYBER_GEAR_ID);
      motor_init_print_can_status();
      return false;
    }
    if ((uint32_t)(now_ms - last_probe_ms) >= 100U)
    {
      /* A full queue is retried on the next probe without enabling the motor. */
      cybergear_stop(&cybergear_base);
      last_probe_ms = now_ms;
    }
    HAL_Delay(10);
  }


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

#endif /* Normal motion/homing helpers are excluded from measurement firmware. */

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
    //printf("0x%03lX\r\n",rxheader.Identifier);

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxheader, rxdata) != HAL_OK)
    {
      break;
    }

    /* Count all frames before protocol/ID checks to diagnose rejected replies. */
    motor_can_rx_count++;
    motor_can_last_rx_id = rxheader.Identifier;
    motor_can_last_rx_dlc = rxheader.DataLength;
    motor_can_last_rx_id_type = rxheader.IdType;

    /* CyberGear type 17/readback and type 21/fault must pass BEFORE the
       existing RobStride type-2-only path. Keep its validation unchanged. */
    if (cybergear_process_rx(&cybergear_base, &rxheader, rxdata))
    {
      if (robstride_get_communication_type(rxheader.Identifier) == FeedbackId)
      {
        motor_last_feedback_ms = HAL_GetTick();
        motor_init_feedback_received = true;
      }
      continue;
    }

    if (rxheader.IdType != FDCAN_EXTENDED_ID ||
        rxheader.RxFrameType != FDCAN_DATA_FRAME ||
        rxheader.DataLength != FDCAN_DLC_BYTES_8 ||
        robstride_get_communication_type(rxheader.Identifier) != FeedbackId ||
        robstride_get_destination_id(rxheader.Identifier) != HOST_ID)
    {
      continue;
    }

    /* Count known motors even before their software handlers are initialized. */
    const uint8_t motor_id = (uint8_t)(robstride_get_area_2(rxheader.Identifier) & 0xffU);
    if (motor_id == CYBER_GEAR_ID || motor_id == RIGHT_RS03_ID ||
        motor_id == LEFT_RS03_ID || motor_id == EL05_ID)
    {
      motor_last_feedback_ms = HAL_GetTick();
      motor_init_feedback_received = true;
    }

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

#if CYBERGEAR_CALIBRATION_BUILD
  /* Measurement accepts only local UART trial requests. Consume upper-board
     frames without changing targets, homing requests, or return state. */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
  {
    FDCAN_RxHeaderTypeDef ignored_header;
    uint8_t ignored_data[64];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &ignored_header, ignored_data) != HAL_OK) break;
  }
#else

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
      case MOTOR_INIT_CANID:
      {
        static bool has_previous_command = false;
        static int32_t previous_command = 0;
        /* The first int32 is big-endian, like the other inter-board values. */
        if (rxheader.IdType == FDCAN_STANDARD_ID &&
            rxheader.RxFrameType == FDCAN_DATA_FRAME &&
            rxheader.DataLength >= FDCAN_DLC_BYTES_4)
        {
          int32_t command = 0;
          u8_to_int(rxdata, &command, 4);
          //printf("%d\r\n",command);
          /* Use the first valid command as the baseline for change detection. */
          if (!motor_return_active && has_previous_command && command != previous_command)
          {
            if (motors_running)
            {
              const uint32_t interrupt_mask = __get_PRIMASK();
              __disable_irq();
              motor_return_active = true;
              target_angle[0] = 0.140f;
              target_angle[1] = -0.900f;
              target_angle[2] = 1.98f;
              //target_angle[3] = 0.0f;
              motor_return_started_ms = HAL_GetTick();
              __set_PRIMASK(interrupt_mask);
            }
            else
            {
              motor_init_requested = true;
            }
          }
          /* Track ignored commands too, so they are not replayed after return. */
          previous_command = command;
          has_previous_command = true;
        }
        break;
      }
      case CANID: 
        if (motor_return_active || rxheader.IdType != FDCAN_STANDARD_ID ||
            rxheader.RxFrameType != FDCAN_DATA_FRAME || rxheader.DataLength != FDCAN_DLC_BYTES_16)
        {
          break;
        }
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
#endif
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
#if CYBERGEAR_CALIBRATION_BUILD
    static uint8_t measurement_phase = 0U;
    if (measurement_phase == 0U)
      cg_cal_app_tick(&cybergear_calibration_app, HAL_GetTick());
    measurement_phase = (uint8_t)((measurement_phase + 1U) % CG_CAL_APP_PERIOD_MS);
#else
    static uint8_t control_phase = 0U;

    /* CG can use phase 0/5. Other axes and the board reply retain all phases. */
    if (cybergear_control_phase_due(control_phase))
    {
      cybergear_control_position_adrc(&cybergear_base, target_angle[0]);
    }
    switch (control_phase)
    {
      case 1U:
        robstride_set_position(&robstride_handler[RIGHT_RS03_INDEX], target_angle[2] - 1.884f);
        break;
      case 2U:
        robstride_set_position(&robstride_handler[LEFT_RS03_INDEX], -target_angle[1] - 1.0f);
        break;
      case 3U:
        if (!el05_initializing)
        {
          robstride_set_position(&robstride_handler[EL05_INDEX], - target_angle[3] - 2.963f);
        }
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

    float send_angles[4] = {0};
    uint8_t txdata[16] = {0};

    // 1. 各モーターのフィードバックから現在角度(position_rad)を取得
    send_angles[2] = robstride_handler[RIGHT_RS03_INDEX].feedback.position_rad + 1.884;
    send_angles[1] = -(robstride_handler[LEFT_RS03_INDEX].feedback.position_rad + 1.0);
    send_angles[3] = -(robstride_handler[EL05_INDEX].feedback.position_rad + 2.963);
    send_angles[0] = cybergear_base.feedback.position_rad;

    // 2. float (4つ) を uint8_t配列 (16バイト) に変換
    float_to_u8(send_angles, txdata, 4);

    // 3. 送信設定の変更 (16バイトのCAN FDフレームとして送信)
    inter_board_txheader.Identifier = 0x210; // 必要に応じてIDを変更してください
    inter_board_txheader.DataLength = FDCAN_DLC_BYTES_16; 

    // 4. FDCAN1から送信
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &inter_board_txheader, txdata);
#endif
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
  if (inter_board_CAN_RxTxSettings_init(&inter_board_txheader) != HAL_OK ||
      motor_CAN_RxTxSettings_init(&motor_txheader) != HAL_OK)
  {
    Error_Handler();
  }

#if CYBERGEAR_CALIBRATION_BUILD
  /* Isolated measurement entry: no search homing, encoder zero, RS enable,
     upper-board motion, normal ADRC, or watchdog MCU reset is reachable. */
  CgCalConfig calibration_config;
  cg_cal_app_config_defaults(&calibration_config);
  if (!cybergear_init(&cybergear_base, &hfdcan3, CYBER_GEAR_ID, HOST_ID) ||
      !cg_cal_app_init(&cybergear_calibration_app, &cybergear_base, &calibration_config))
  {
    const char disabled[] = "# CGCAL disabled: configure calibration current/temperature/speed and ARM\r\n";
    HAL_UART_Transmit(&huart2, (const uint8_t *)disabled, sizeof(disabled)-1U, 100U);
    for (unsigned int attempt=0; attempt<20U; ++attempt) { cybergear_stop(&cybergear_base); HAL_Delay(20U); }
    while (1) HAL_Delay(10U);
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    /* The timer is unavailable, so finish bounded STOP attempts in main. */
    cybergear_base.internal_send=true;
    for (unsigned int attempt=0; attempt<20U; ++attempt) { cybergear_stop(&cybergear_base); HAL_Delay(20U); }
    cybergear_base.internal_send=false;
    while (1) HAL_Delay(10U);
  }
  const char help[] = "# CGCAL: wait READY; p=positive pulse, n=negative pulse, x=abort. Origin fixed until reboot.\r\n";
  HAL_UART_Transmit(&huart2, (const uint8_t *)help, sizeof(help)-1U, 100U);
  bool announced_ready=false;
  while (1)
  {
    uint8_t key;
    if (HAL_UART_Receive(&huart2, &key, 1U, 0U) == HAL_OK)
    {
      if (key == 'x' || key == 'X') cg_cal_app_abort(&cybergear_calibration_app);
      else if (key == 'p' || key == 'P') cg_cal_app_request_trial(&cybergear_calibration_app, 1);
      else if (key == 'n' || key == 'N') cg_cal_app_request_trial(&cybergear_calibration_app, -1);
    }
    const uint32_t snapshot_mask=__get_PRIMASK();
    __disable_irq();
    const CgCalAppState calibration_state=cybergear_calibration_app.state;
    const bool needs_dump=cybergear_calibration_app.dump_pending;
    const float origin=cybergear_calibration_app.initial_position_rad;
    __set_PRIMASK(snapshot_mask);
    if (!announced_ready && calibration_state == CG_CAL_APP_READY)
    {
      char ready[96];
      int length=snprintf(ready, sizeof(ready), "# CGCAL READY origin=%.7f rad; p/n starts one trial\r\n", (double)origin);
      if (length > 0 && (size_t)length < sizeof(ready))
        HAL_UART_Transmit(&huart2, (const uint8_t *)ready, (uint16_t)length, 100U);
      announced_ready=true;
    }
    if (needs_dump) cg_cal_app_dump(&cybergear_calibration_app, &huart2);
    HAL_Delay(1U);
  }
#else
  /* CAN reception is active; defer homing, enable and cyclic commands. */
  while (!motor_init_requested)
  {
    HAL_Delay(10);
  }

  /* Start monitoring with initialization, then keep monitoring in normal use. */
  const uint32_t motor_init_started_ms = HAL_GetTick();
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  motor_init_feedback_received = false;
  motor_last_feedback_ms = HAL_GetTick();
  __set_PRIMASK(interrupt_mask);

  if (!cybergear_base_init())
  {
    motor_init_wait_for_feedback();
    Error_Handler();
  }

  if (!cybergear_homing())
  {
    printf("CyberGear Homing Failed\r\n");
    motor_init_wait_for_feedback();
    Error_Handler();
  }
  /* Enable motors after the blocking homing routine, just before cyclic commands. */
  if (!robstride_init())
  {
    motor_init_wait_for_feedback();
    Error_Handler();
  }
  motor_init_wait_for_feedback();
  /* Start ADRC after blocking initialization, immediately before cyclic commands. */
  if (!cybergear_start_position_adrc(&cybergear_base))
  {
    printf("CyberGear ADRC start failed\r\n");
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  motors_running = true;
  printf("Motor initialization complete: %lu ms\r\n",
         (unsigned long)(HAL_GetTick() - motor_init_started_ms));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const uint32_t el05_startup_ms = HAL_GetTick();
  const uint32_t el05_initial_rx_count = robstride_handler[EL05_INDEX].feedback.received_count;
  bool el05_retry_pending = true;
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    motor_check_feedback();
    motor_return_update();
    cybergear_service(&cybergear_base);
    /* Retry once after fresh feedback, only during startup. */
    if (el05_retry_pending)
    {
      const RobstrideFeedback feedback = robstride_handler[EL05_INDEX].feedback;
      const uint32_t now_ms = HAL_GetTick();
      if ((uint32_t)(now_ms - el05_startup_ms) >= 3000U)
      {
        el05_retry_pending = false;
        printf("EL05 startup retry expired: no fresh feedback\r\n");
      }
      else if (feedback.online && feedback.received_count != el05_initial_rx_count &&
               (uint32_t)(now_ms - feedback.last_leceived_ms) < 100U)
      {
        /* Never re-arm after a fault, a later stop, or a motor power cycle. */
        el05_retry_pending = false;
        if (feedback.mode == 0U && feedback.fault_flags == 0U)
        {
          el05_initializing = true;
          const bool queued = robstride_start_position_pp_mode(
              &robstride_handler[EL05_INDEX], 10, 1, 10);
          el05_initializing = false;
          printf("EL05 startup retry after feedback: queued=%u\r\n", (unsigned int)queued);
        }
      }
    }
    
    el05_debug_print();
    cybergear_debug_print();
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
#endif
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
