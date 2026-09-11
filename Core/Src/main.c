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
#include "robstride_startup.h"
#include "board_protocol.h"
#include "cybergear.h"
#include "app_mode.h"
#include "cybergear_test_motion.h"
#include "cybergear_console.h"
#include "cybergear_tuning.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HOST_ID CYBERGEAR_HOST_ID

#define RIGHT_RS03_ID 3
#define LEFT_RS03_ID 4

#define EL05_ID 5

#define CYBER_GEAR_ID CYBERGEAR_MOTOR_ID


#define RIGHT_RS03_INDEX 0
#define LEFT_RS03_INDEX 1

#define EL05_INDEX 2

#define CANID 0x200
#define MOTOR_INIT_CANID 0x500
#define EL05_DEBUG_INTERVAL_MS 200U
#define MOTOR_RETURN_IGNORE_MS 2000U
#define MOTOR_STOP_TIMEOUT_MS 1000U
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
static volatile bool motor_init_requested = false;
static volatile bool motors_running = false;
static volatile bool motor_return_active = false;
static volatile uint32_t motor_return_started_ms = 0U;
static volatile bool motor_init_feedback_received = false;
static volatile uint32_t motor_last_feedback_ms = 0U;
static volatile uint32_t motor_can_rx_count = 0U;
static volatile uint32_t motor_can_last_rx_id = 0U;
static volatile uint32_t motor_can_last_rx_dlc = 0U;
static volatile uint32_t motor_can_last_rx_id_type = 0U;
#if APP_CYBERGEAR_STANDALONE_TEST
static CyberGearTestMotion cybergear_test_motion;
static CyberGearConsole cybergear_console;
static CyberGearTuning cybergear_test_tuning, cybergear_test_candidate;
static bool cybergear_test_initialized, cybergear_test_can_started;
static volatile bool cybergear_test_timer_active;
static uint8_t cybergear_test_control_phase;
#else
static RobstrideStartup robstride_startup;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */
static void motor_init_failed(const char *reason);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  if (APP_CYBERGEAR_STANDALONE_TEST || !motors_running) return;
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t now_ms = HAL_GetTick();
  bool healthy = cybergear_base.state == CG_STATE_RUNNING &&
      cybergear_base.feedback.online && cybergear_base.feedback.fault_flags == 0U &&
      now_ms - cybergear_base.feedback.last_received_ms < cybergear_base.config.controller.feedback_timeout_ms;
  for (uint32_t index = 0U; index < 3U; ++index)
  {
    const RobstrideFeedback feedback = robstride_handler[index].feedback;
    healthy = healthy && feedback.online && feedback.fault_flags == 0U &&
        feedback.mode == 2U && isfinite(feedback.position_rad) &&
        now_ms - feedback.last_leceived_ms < ROBSTRIDE_STARTUP_FEEDBACK_MS;
  }
  __set_PRIMASK(interrupt_mask);
  if (!healthy) motor_init_failed("motor feedback or control fault");
}

static bool motor_stop_all(void)
{
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  motors_running = false;
  motor_return_active = false;
  __set_PRIMASK(interrupt_mask);
  HAL_TIM_Base_Stop_IT(&htim6);
  const uint32_t started_ms = HAL_GetTick();
  uint32_t last_probe_ms = started_ms - 10U;
  uint32_t next_stop_axis = 0U;
  uint32_t queued_mask = 0U;
  uint32_t stopped_mask = 0U;
  uint32_t baseline[3] = {0};
  uint32_t cybergear_baseline = cybergear_base.feedback.rx_sequence;
  if (cybergear_base.managed) cybergear_stop(&cybergear_base);
  do
  {
    const uint32_t now_ms = HAL_GetTick();
    if (cybergear_base.managed)
    {
      const uint32_t mask = __get_PRIMASK();
      __disable_irq();
      cybergear_control_position_adrc(&cybergear_base, 0.0f);
      __set_PRIMASK(mask);
    }
    if (now_ms - last_probe_ms >= 10U)
    {
      const uint32_t mask = __get_PRIMASK();
      __disable_irq();
      if (next_stop_axis == 3U && !cybergear_base.managed)
      {
        const uint32_t sequence = cybergear_base.feedback.rx_sequence;
        if (cybergear_stop(&cybergear_base) && (queued_mask & 8U) == 0U)
        {
          cybergear_baseline = sequence;
          queued_mask |= 8U;
        }
      }
      else if (next_stop_axis < 3U)
      {
        const uint32_t index = next_stop_axis;
        const uint32_t sequence = robstride_handler[index].feedback.received_count;
        if (robstride_stop(&robstride_handler[index]) && (queued_mask & (1U << index)) == 0U)
        {
          baseline[index] = sequence;
          queued_mask |= 1U << index;
        }
      }
      next_stop_axis = (next_stop_axis + 1U) % 4U;
      __set_PRIMASK(mask);
      last_probe_ms = now_ms;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t sample_ms = HAL_GetTick();
    stopped_mask = 0U;
    for (uint32_t index = 0U; index < 3U; ++index)
    {
      const RobstrideFeedback feedback = robstride_handler[index].feedback;
      if (feedback.online && feedback.received_count != baseline[index] &&
          feedback.mode == 0U && isfinite(feedback.velocity_rps) &&
          fabsf(feedback.velocity_rps) <= 0.05f &&
          sample_ms - feedback.last_leceived_ms < ROBSTRIDE_STARTUP_FEEDBACK_MS)
        stopped_mask |= 1U << index;
    }
    if (cybergear_base.managed)
    {
      if (cybergear_base.stop_queued) queued_mask |= 8U;
      if (cybergear_base.reset_confirmed && cybergear_base.stationary &&
          cybergear_base.feedback.mode == 0U &&
          sample_ms - cybergear_base.feedback.last_received_ms < cybergear_base.config.controller.feedback_timeout_ms)
        stopped_mask |= 8U;
    }
    else
    {
      const CyberGearFeedback feedback = cybergear_base.feedback;
      if (feedback.online && feedback.rx_sequence != cybergear_baseline && feedback.mode == 0U &&
          isfinite(feedback.velocity_rad_s) && fabsf(feedback.velocity_rad_s) <= 0.05f &&
          sample_ms - feedback.last_received_ms < CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS)
        stopped_mask |= 8U;
    }
    __set_PRIMASK(mask);
    if ((queued_mask & stopped_mask) == 15U) return true;
    HAL_Delay(cybergear_base.config.controller.period_ms);
  } while (HAL_GetTick() - started_ms < MOTOR_STOP_TIMEOUT_MS);
  printf("Motor STOP unconfirmed: queued=0x%lX stopped=0x%lX\r\n",
         (unsigned long)queued_mask, (unsigned long)stopped_mask);
  return false;
}

static void motor_init_failed(const char *reason)
{
  motor_stop_all();
  printf("Motor initialization/control failed: %s\r\n", reason);
  motor_init_print_can_status();
  Error_Handler();
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
#if APP_CYBERGEAR_STANDALONE_TEST
  if (cybergear_console.stop_requested || cybergear_base.motor_fault_sequence != 0U)
  {
    cybergear_stop(&cybergear_base);
    return false;
  }
#endif
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
    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
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
  HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
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
        /* Keep the first crossing angle, then confirm the switch state. */
        *edge_rad = feedback.position_rad;
        edge_started_ms = HAL_GetTick();
        edge_pending = true;
      }
      if ((uint32_t)(HAL_GetTick() - edge_started_ms) >= CYBERGEAR_HOMING_DEBOUNCE_MS)
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
    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
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
    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
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
#if APP_CYBERGEAR_STANDALONE_TEST
  if (cybergear_console.stop_requested) return false;
#endif
  // 1. 速度制御モードに変更して有効化
  if (!cybergear_set_run_mode(&cybergear_base, CYBERGEAR_RUN_MODE_SPEED)) 
  {
    return false;
  }
  HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
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
#if APP_CYBERGEAR_STANDALONE_TEST
    if (cybergear_console.stop_requested)
    {
      cybergear_stop(&cybergear_base);
      return false;
    }
#endif
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
    if ((uint32_t)(now_ms - feedback_wait_started_ms) >= CYBERGEAR_HOMING_STARTUP_TIMEOUT_MS)
    {
      cybergear_stop(&cybergear_base);
      return false;
    }
    /* Retry a lost enable response while the speed reference is still zero. */
    if ((uint32_t)(now_ms - last_enable_ms) >= CYBERGEAR_HOMING_PROBE_INTERVAL_MS)
    {
      if (!cybergear_enable(&cybergear_base))
      {
        cybergear_stop(&cybergear_base);
        motor_init_print_can_status();
        return false;
      }
      last_enable_ms = now_ms;
    }
    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
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

  if (!cybergear_set_velocity(&cybergear_base, CYBERGEAR_HOMING_FAST_SPEED_RAD_S))
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

    /* Reverse once if the switch has not changed within the search angle. */
    if (!reversed &&
        feedback.position_rad - start_position_rad >= CYBERGEAR_HOMING_REVERSE_ANGLE_RAD &&
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == initial_state)
    {
      direction = -1.0f;
      reversed = true;
    }
    if (!cybergear_set_velocity(&cybergear_base, direction * CYBERGEAR_HOMING_FAST_SPEED_RAD_S))
    {
      cybergear_stop(&cybergear_base);
      return false;
    }
    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
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
  HAL_Delay(CYBERGEAR_HOMING_POLL_MS);

  if (!cybergear_set_zero(&cybergear_base))
  {
    return false;
  }
  HAL_Delay(CYBERGEAR_HOMING_POLL_MS);

  printf("CG home edges=%.4f,%.4f center=%.4f rad\r\n",
         (double)reverse_edge_rad, (double)forward_edge_rad, (double)center_rad);
  /* Keep stopped until all blocking motor initialization is complete. */
  return true;
}

#if !APP_CYBERGEAR_STANDALONE_TEST
static void motor_prepare_handlers(void)
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

}

#endif

bool cybergear_base_init(void)
{
#if APP_CYBERGEAR_STANDALONE_TEST
  if (!cybergear_test_initialized || cybergear_base.fault != CG_FAULT_NONE ||
      cybergear_base.motor_fault_sequence != 0U) return false;
#else
  const uint32_t init_mask = __get_PRIMASK();
  __disable_irq();
  const bool initialized = cybergear_init(
    &cybergear_base,
    &hfdcan3,
    CYBER_GEAR_ID,
    HOST_ID
  );
  __set_PRIMASK(init_mask);
  if (!initialized)
  {
    return false;
  }
#endif

  /* Reject incomplete machine settings before the legacy homing can energize CG. */
  if (!cybergear_config_valid(&cybergear_base.config))
  {
    printf("CG machine parameters invalid: see cybergear_config.h\r\n");
    return false;
  }

  /* Establish feedback before enabling motion. A queued frame is not proof
     of delivery, so retry STOP while the motor is powering up. */
  const uint32_t wait_started_ms = HAL_GetTick();
  uint32_t last_probe_ms = wait_started_ms - CYBERGEAR_HOMING_PROBE_INTERVAL_MS;
  while (1)
  {
#if APP_CYBERGEAR_STANDALONE_TEST
    if (cybergear_console.stop_requested || cybergear_base.fault != CG_FAULT_NONE ||
        cybergear_base.motor_fault_sequence != 0U) return false;
#endif
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
    if ((uint32_t)(now_ms - wait_started_ms) >= CYBERGEAR_HOMING_STARTUP_TIMEOUT_MS)
    {
      printf("CG startup: no feedback from ID 0x%02X\r\n", CYBER_GEAR_ID);
      motor_init_print_can_status();
      return false;
    }
    if ((uint32_t)(now_ms - last_probe_ms) >= CYBERGEAR_HOMING_PROBE_INTERVAL_MS)
    {
      /* A full queue is retried on the next probe without enabling the motor. */
      cybergear_stop(&cybergear_base);
      last_probe_ms = now_ms;
    }

    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
  }


  return true;
}

#if !APP_CYBERGEAR_STANDALONE_TEST
static bool motor_start_control(void)
{
  if (!robstride_startup_init(&robstride_startup, robstride_handler, 3U) ||
      !cybergear_begin_position_control(&cybergear_base, CYBERGEAR_COMPARE_OPERATION_MODE ?
          CYBERGEAR_RUN_MODE_OPERATION : CYBERGEAR_RUN_MODE_CURRENT))
    return false;
  while (1)
  {
    robstride_startup_update(&robstride_startup);
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool cybergear_ok = cybergear_control_position_adrc(
        &cybergear_base, cybergear_base.requested_target_rad);
    const bool ready = cybergear_ok && cybergear_base.state == CG_STATE_RUNNING &&
        robstride_startup_ready(&robstride_startup);
    HAL_StatusTypeDef timer_status = HAL_ERROR;
    if (ready)
    {
      cybergear_base.first_cyclic = true;
      motors_running = true;
      timer_status = HAL_TIM_Base_Start_IT(&htim6);
      if (timer_status != HAL_OK) motors_running = false;
    }
    __set_PRIMASK(mask);
    if (ready) return timer_status == HAL_OK;
    if (!cybergear_ok || robstride_startup_failed(&robstride_startup)) return false;
    cybergear_service(&cybergear_base);
    HAL_Delay(cybergear_base.config.controller.period_ms);
  }
}

#endif

#if APP_CYBERGEAR_STANDALONE_TEST
static void cybergear_test_console_flush(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const bool stop_requested = cybergear_console.stop_requested;
  const bool discard_line = cybergear_console.dropping || cybergear_console.receiving_line;
  cybergear_console_init(&cybergear_console);
  cybergear_console.stop_requested = stop_requested;
  cybergear_console.dropping = discard_line;
  cybergear_console.receiving_line = discard_line;
  __set_PRIMASK(mask);
}

void USART2_IRQHandler(void)
{
  const uint32_t status = READ_REG(huart2.Instance->ISR);
  if ((status & (UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE)) != 0U)
  {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    if ((status & UART_FLAG_RXNE) != 0U) (void)READ_REG(huart2.Instance->RDR);
    cybergear_console_receive_error(&cybergear_console);
    return;
  }
  if ((status & UART_FLAG_RXNE) != 0U)
    cybergear_console_receive(&cybergear_console, (uint8_t)READ_REG(huart2.Instance->RDR));
}

static void cybergear_test_console_init(void)
{
  __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
  cybergear_test_console_flush();
  HAL_NVIC_SetPriority(USART2_IRQn, 2U, 0U);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
}

static bool cybergear_test_trajectory_done(void)
{
  return !cybergear_base.recovery_zero_active &&
      !cybergear_base.trajectory.active && !cybergear_base.prepared_ready &&
      fabsf(cybergear_base.trajectory.target_rad - target_angle[0]) <=
      cybergear_base.effective_limits.target_tolerance_rad &&
      fabsf(cybergear_base.requested_target_rad - target_angle[0]) <=
      cybergear_base.effective_limits.target_tolerance_rad;
}

static void cybergear_test_status(bool force)
{
  static uint32_t last_print_ms = 0U;
  const uint32_t now_ms = HAL_GetTick();
  if (!force && (uint32_t)(now_ms - last_print_ms) < CG_TEST_LOG_INTERVAL_MS) return;
  last_print_ms = now_ms;
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const CyberGearFeedback feedback = cybergear_base.feedback;
  const CyberGearState state = cybergear_base.state;
  const CyberGearFault fault = cybergear_base.fault;
  const float target_rad = target_angle[0];
  const float current_a = cybergear_base.controller.last_queued_current_a;
  const float estimated_velocity_rad_s = cybergear_base.controller.velocity_rad_s;
  const float tracking_current_a = cybergear_base.output.tracking_current_a;
  const float disturbance_current_a = cybergear_base.output.disturbance_current_a;
  const bool disturbance_limited = cybergear_base.output.disturbance_limited;
  const float requested_current_a = cybergear_base.output.requested_current_a;
  const bool amplitude_limited = cybergear_base.output.amplitude_limited;
  const bool slew_limited = cybergear_base.output.slew_limited;
  const float tracking_error_rad = cybergear_base.trajectory.point.q_rad - feedback.position_rad;
  const uint32_t saturation_ms = cybergear_base.saturation_active ?
      (uint32_t)(cybergear_base.last_control_ms - cybergear_base.saturation_since_ms) : 0U;
  const uint32_t recovery_count = cybergear_base.recovery_count;
  const bool recovery_zero_active = cybergear_base.recovery_zero_active;
  const bool trajectory_done = cybergear_test_trajectory_done();
  const uint32_t sample_ms = HAL_GetTick();
  const uint32_t settled_ms = cybergear_test_motion.settling ?
      (uint32_t)(sample_ms - cybergear_test_motion.settled_since_ms) : 0U;
  __set_PRIMASK(interrupt_mask);
  HAL_GPIO_TogglePin(Board_LED_GPIO_Port, Board_LED_Pin);
  printf("CG TEST " CG_TEST_FIRMWARE_TAG " state=%u fault=%u online=%u age=%lu\r\n",
         (unsigned int)state, (unsigned int)fault, (unsigned int)feedback.online,
         (unsigned long)(sample_ms - feedback.last_received_ms));
  printf("CG TEST q=%.4f v=%.4f target=%.4f limit=%u\r\n",
         (double)feedback.position_rad, (double)feedback.velocity_rad_s,
         (double)target_rad,
         (unsigned int)HAL_GPIO_ReadPin(limit_GPIO_Port, limit_Pin));
  printf("CG TEST i_cmd=%.3f halted=%u legs=%lu\r\n", (double)current_a,
         (unsigned int)cybergear_test_motion.halted,
         (unsigned long)cybergear_test_motion.completed_legs);
  printf("CG TEST err=%.4f vest=%.4f done=%u dwell=%lu\r\n",
         (double)(target_rad - feedback.position_rad), (double)estimated_velocity_rad_s,
         (unsigned int)trajectory_done, (unsigned long)settled_ms);
  printf("CG TEST itrk=%.3f idist=%.3f dlim=%.3f dclip=%u\r\n",
         (double)tracking_current_a, (double)disturbance_current_a,
         (double)cybergear_base.config.controller.disturbance_limit_a,
         (unsigned int)disturbance_limited);
  printf("CG TEST ireq=%.3f amp=%u slew=%u sat_ms=%lu track=%.4f recover=%lu zero=%u\r\n",
         (double)requested_current_a, (unsigned int)amplitude_limited,
         (unsigned int)slew_limited, (unsigned long)saturation_ms,
         (double)tracking_error_rad, (unsigned long)recovery_count, (unsigned int)recovery_zero_active);
}

static bool cybergear_test_can_ready(void)
{
  if (cybergear_test_can_started) return true;
  if (motor_CAN_RxTxSettings_init(&motor_txheader) != HAL_OK) return false;
  cybergear_test_can_started = true;
  return true;
}

static bool cybergear_test_reinitialize(const CyberGearTuning *settings)
{
  if (!cybergear_tuning_valid(settings)) return false;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  cybergear_test_timer_active = false;
  const HAL_StatusTypeDef timer_status = HAL_TIM_Base_Stop_IT(&htim6);
  cybergear_test_motion.halted = true;
  __set_PRIMASK(mask);
  cybergear_test_console_flush();
  if (!cybergear_test_can_ready()) return false;
  mask = __get_PRIMASK();
  __disable_irq();
  if (!cybergear_test_initialized)
    cybergear_test_initialized = cybergear_init(&cybergear_base, &hfdcan3, CYBER_GEAR_ID, HOST_ID);
  const bool requested = cybergear_test_initialized &&
      cybergear_request_reinitialize_stop(&cybergear_base);
  __set_PRIMASK(mask);
  if (!requested) return false;
  printf("CG CONSOLE: stopping; waiting for fresh STOP and stationary feedback\r\n");
  const uint32_t started_ms = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - started_ms) <=
      cybergear_base.config.stop_timeout_ms + cybergear_base.config.command_retry_ms)
  {
    mask = __get_PRIMASK();
    __disable_irq();
    cybergear_control_position_adrc(&cybergear_base, target_angle[0]);
    const bool failed = cybergear_base.state == CG_STATE_FAULT &&
        (!cybergear_base.reset_confirmed || !cybergear_base.stationary);
    const bool queue_empty = HAL_FDCAN_IsTxBufferMessagePending(&hfdcan3,
        FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) == 0U;
    const bool stopped = queue_empty && cybergear_reset_fault(&cybergear_base);
    const bool cancelled = cybergear_console.stop_requested;
    bool initialized = false;
    if (stopped && timer_status == HAL_OK && !cancelled)
    {
      initialized = cybergear_init(&cybergear_base, &hfdcan3, CYBER_GEAR_ID, HOST_ID) &&
          cybergear_configure(&cybergear_base, &settings->motor);
      if (initialized)
      {
        cybergear_test_tuning = *settings;
        initialized = cybergear_test_motion_init_settings(&cybergear_test_motion, HAL_GetTick(),
            settings->amplitude_deg, settings->small_amplitude_deg, settings->dwell_ms, settings->leg_timeout_ms);
        cybergear_test_motion.halted = true;
        target_angle[0] = 0.0f;
        cybergear_test_control_phase = 0U;
      }
    }
    __set_PRIMASK(mask);
    if (stopped)
    {
      cybergear_test_console_flush();
      if (initialized)
        printf("CG CONSOLE: reinitialized; drive disabled\r\n");
      return initialized;
    }
    if (failed) break;
    HAL_Delay(CG_TEST_STOP_POLL_MS);
  }
  cybergear_test_console_flush();
  printf("CG CONSOLE: STOP unconfirmed; settings unchanged; check motor then reinit + Enter\r\n");
  return false;
}

static bool cybergear_test_start(void)
{
  if (!cybergear_test_reinitialize(&cybergear_test_tuning)) return false;
  if (cybergear_console.stop_requested || !cybergear_base_init()) return false;
  if (!cybergear_configure(&cybergear_base, &cybergear_test_tuning.motor)) return false;
  printf("CG TEST: homing started; %c=stop; other input discarded while busy\r\n", CG_TEST_STOP_COMMAND);
  if (!cybergear_homing() || cybergear_console.stop_requested) return false;
  target_angle[0] = 0.0f;
  if (!cybergear_begin_position_control(&cybergear_base, CYBERGEAR_RUN_MODE_CURRENT)) return false;
  while (1)
  {
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const CyberGearState startup_state = cybergear_base.state;
    const CyberGearFeedback startup_feedback = cybergear_base.feedback;
    const uint32_t startup_now_ms = HAL_GetTick();
    const uint32_t quiet_ms = cybergear_base.quiet_active ?
        (uint32_t)(startup_now_ms - cybergear_base.quiet_since_ms) : 0U;
    if (cybergear_console.stop_requested) cybergear_stop(&cybergear_base);
    const bool healthy = cybergear_control_position_adrc(&cybergear_base, target_angle[0]);
    const bool running = cybergear_base.state == CG_STATE_RUNNING;
    __set_PRIMASK(mask);
    if (!healthy)
    {
      printf("CG startup failed: phase=%u fault=%u elapsed=%lu ms\r\n",
             (unsigned int)startup_state, (unsigned int)cybergear_base.fault,
             (unsigned long)(startup_now_ms - cybergear_base.startup_ms));
      printf("CG startup feedback: mode=%u online=%u age=%lu v=%.5f q=%.5f\r\n",
             (unsigned int)startup_feedback.mode, (unsigned int)startup_feedback.online,
             (unsigned long)(startup_now_ms - startup_feedback.last_received_ms),
             (double)startup_feedback.velocity_rad_s, (double)startup_feedback.position_rad);
      printf("CG startup stationary: limit=%.5f quiet_ms=%lu required_ms=%lu\r\n",
             (double)cybergear_base.config.stationary_speed_rad_s,
             (unsigned long)quiet_ms, (unsigned long)cybergear_base.config.stationary_dwell_ms);
      printf("CG startup readback: pending=%u valid=%u value=%u requested=%u\r\n",
             (unsigned int)cybergear_base.mode_read_pending,
             (unsigned int)cybergear_base.mode_read_valid,
             (unsigned int)cybergear_base.mode_read_value,
             (unsigned int)cybergear_base.requested_mode);
      return false;
    }
    if (running) break;
    cybergear_service(&cybergear_base);
    HAL_Delay(cybergear_base.config.controller.period_ms);
  }
  if (!cybergear_test_motion_init_settings(&cybergear_test_motion, HAL_GetTick(),
      cybergear_test_tuning.amplitude_deg, cybergear_test_tuning.small_amplitude_deg,
      cybergear_test_tuning.dwell_ms, cybergear_test_tuning.leg_timeout_ms)) return false;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (cybergear_console.stop_requested)
  {
    cybergear_stop(&cybergear_base);
    __set_PRIMASK(mask);
    return false;
  }
  target_angle[0] = cybergear_test_motion.target_rad;
  cybergear_test_control_phase = 0U;
  __HAL_TIM_SET_COUNTER(&htim6, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  cybergear_base.first_cyclic = true;
  cybergear_base.last_control_ms = HAL_GetTick();
  cybergear_test_timer_active = true;
  const HAL_StatusTypeDef timer_status = HAL_TIM_Base_Start_IT(&htim6);
  if (timer_status != HAL_OK) cybergear_test_timer_active = false;
  __set_PRIMASK(mask);
  cybergear_test_console_flush();
  printf("CG TEST: +/-%.2f, +/-%.2f deg; dwell=%lu ms\r\n",
         (double)cybergear_test_tuning.amplitude_deg, (double)cybergear_test_tuning.small_amplitude_deg,
         (unsigned long)cybergear_test_tuning.dwell_ms);
  printf("CG TEST: current<=%.3f A speed<=%.3f rad/s wc=%.2f wo=%.2f\r\n",
         (double)cybergear_test_tuning.motor.controller.current_limit_a,
         (double)cybergear_test_tuning.motor.trajectory.velocity_max_rad_s,
         (double)cybergear_test_tuning.motor.controller.bandwidth_rad_s,
         (double)cybergear_test_tuning.motor.controller.observer_rad_s);
  return timer_status == HAL_OK;
}

static void cybergear_standalone_test_run(void)
{
  const bool settings_valid = cybergear_tuning_defaults(&cybergear_test_tuning);
  cybergear_test_console_init();
  if (!settings_valid) printf("CG CONSOLE: invalid boot settings; start is blocked\r\n");
  printf("CG TEST: " CG_TEST_FIRMWARE_TAG "; UART console ready; CAN1/RobStride disabled\r\n");
  printf("CG CONSOLE: %c + Enter=start; %c=stop; set KEY VALUE / reinit / show / help + Enter\r\n",
         CG_TEST_START_COMMAND, CG_TEST_STOP_COMMAND);
  bool waiting = true;
  bool running = false;
  bool status_paused = false;
  size_t show_index = cybergear_tuning_count();
  static CyberGearConsoleCommand command;
  while (1)
  {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const CyberGearConsoleAction action = cybergear_console_poll(&cybergear_console, &command);
    __set_PRIMASK(mask);
    bool status_requested = status_paused && action != CG_CONSOLE_NONE;
    bool halt_report = false;
    if (action == CG_CONSOLE_STOP)
    {
      mask = __get_PRIMASK();
      __disable_irq();
      if (cybergear_test_initialized) cybergear_stop(&cybergear_base);
      cybergear_test_motion.halted = true;
      __set_PRIMASK(mask);
      waiting = false;
      running = false;
      halt_report = true;
      printf("CG CONSOLE: stopped; reinit or set required before another start\r\n");
    }
    else if (action == CG_CONSOLE_SET || action == CG_CONSOLE_REINIT)
    {
      cybergear_test_candidate = cybergear_test_tuning;
      if (action == CG_CONSOLE_SET &&
          !cybergear_tuning_set(&cybergear_test_candidate, command.key, command.value))
        printf("CG CONSOLE: invalid setting; unchanged (show/help lists keys and limits)\r\n");
      else
      {
        running = false;
        waiting = cybergear_test_reinitialize(&cybergear_test_candidate);
        status_paused = !waiting;
        status_requested = false;
        halt_report = !waiting;
        if (!waiting) printf("CG CONSOLE: reinit failed; no restart\r\n");
        else printf("CG CONSOLE: ready; send %c + Enter to home/start\r\n", CG_TEST_START_COMMAND);
      }
    }
    else if (action == CG_CONSOLE_START)
    {
      if (!waiting || running) printf("CG CONSOLE: start rejected; reinit required or already running\r\n");
      else
      {
        waiting = false;
        running = cybergear_test_start();
        status_paused = !running;
        status_requested = false;
        halt_report = !running;
        if (!running)
        {
          mask = __get_PRIMASK();
          __disable_irq();
          if (cybergear_test_initialized) cybergear_request_reinitialize_stop(&cybergear_base);
          cybergear_test_motion.halted = true;
          __set_PRIMASK(mask);
          cybergear_test_console_flush();
          printf("CG CONSOLE: startup failed/stopped; reinit required\r\n");
        }
      }
    }
    else if (action == CG_CONSOLE_SHOW || action == CG_CONSOLE_HELP) show_index = 0U;
    else if (action == CG_CONSOLE_INVALID) printf("CG CONSOLE: invalid/overflowed input discarded; use help + Enter\r\n");

    if (running)
    {
      mask = __get_PRIMASK();
      __disable_irq();
      const uint32_t now_ms = HAL_GetTick();
      const bool fresh = cybergear_base.feedback.online &&
          now_ms - cybergear_base.feedback.last_received_ms <= cybergear_base.config.controller.feedback_timeout_ms;
      const CyberGearTestResult result = cybergear_test_motion_update(&cybergear_test_motion,
          now_ms, cybergear_base.state == CG_STATE_RUNNING, fresh, cybergear_test_trajectory_done(),
          cybergear_base.feedback.position_rad, cybergear_base.controller.velocity_rad_s);
      if (result == CG_TEST_NEW_TARGET) target_angle[0] = cybergear_test_motion.target_rad;
      else if (result >= CG_TEST_STOP_FAULT)
      {
        cybergear_stop(&cybergear_base);
        running = false;
        halt_report = true;
      }
      __set_PRIMASK(mask);
      if (result == CG_TEST_NEW_TARGET)
        printf("CG TEST: new target=%.4f rad\r\n", (double)cybergear_test_motion.target_rad);
      else if (result >= CG_TEST_STOP_FAULT)
        printf("CG TEST: halted reason=%u; reinit required\r\n", (unsigned int)result);
    }
    if (halt_report)
    {
      status_paused = true;
      status_requested = true;
      show_index = cybergear_tuning_count();
    }
    if (cybergear_test_initialized)
    {
      if (!cybergear_test_timer_active && cybergear_base.managed)
      {
        mask = __get_PRIMASK();
        __disable_irq();
        cybergear_control_position_adrc(&cybergear_base, target_angle[0]);
        __set_PRIMASK(mask);
      }
      cybergear_service(&cybergear_base);
      if (!status_paused && cybergear_base.fault != CG_FAULT_NONE)
      {
        running = false;
        waiting = false;
        cybergear_test_motion.halted = true;
        status_paused = true;
        status_requested = true;
        show_index = cybergear_tuning_count();
      }
      if (status_requested) cybergear_test_status(true);
      else if (!status_paused) cybergear_test_status(false);
    }
    if (show_index < cybergear_tuning_count())
    {
      printf("CG SET %s=%.6g %s\r\n", cybergear_tuning_key(show_index),
             cybergear_tuning_value(&cybergear_test_tuning, show_index), cybergear_tuning_help(show_index));
      show_index++;
    }
    HAL_Delay(CG_TEST_LOOP_DELAY_MS);
  }
}
#endif

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

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
  {
    FDCAN_RxHeaderTypeDef rxheader;
    uint8_t rxdata[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxheader, rxdata) != HAL_OK)
    {
      break;
    }

    /* Drain the FIFO without allowing external commands to change the test. */
    if (APP_CYBERGEAR_STANDALONE_TEST) continue;

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
          const int32_t command = board_decode_init_command(rxdata);
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
            else if (previous_command == 0 && command == 1)
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
        board_decode_angles(rxdata, received_floats);
        bool valid = true;
        for (uint32_t index = 0U; index < 4U; ++index)
          valid = valid && isfinite(received_floats[index]);
        if (!valid) break;
        const uint32_t mask = __get_PRIMASK();
        __disable_irq();
        for (uint32_t index = 0U; index < 4U; ++index)
          target_angle[index] = received_floats[index];
        __set_PRIMASK(mask);

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
#if APP_CYBERGEAR_STANDALONE_TEST
  if (!cybergear_test_timer_active) return;
#endif
  if (htim == &htim6 && (APP_CYBERGEAR_STANDALONE_TEST || motors_running)) {
#if APP_CYBERGEAR_STANDALONE_TEST
    uint8_t control_phase = (cybergear_test_control_phase + 1U) % 10U;
    cybergear_test_control_phase = control_phase;
#else
    static uint8_t control_phase = 0U;
#endif

    /* CG can use phase 0/5. Other axes and the board reply retain all phases. */
    if (cybergear_control_phase_due(control_phase))
    {
      cybergear_control_position_adrc(&cybergear_base, target_angle[0]);
    }
    if (!APP_CYBERGEAR_STANDALONE_TEST) switch (control_phase)
    {
      case 1U:
        robstride_set_position(&robstride_handler[RIGHT_RS03_INDEX], target_angle[2] - 1.884f);
        break;
      case 2U:
        robstride_set_position(&robstride_handler[LEFT_RS03_INDEX], -target_angle[1] - 1.0f);
        break;
      case 3U:
        robstride_set_position(&robstride_handler[EL05_INDEX], - target_angle[3] - 2.963f);
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

    /* The standalone test needs no other board or CAN1 acknowledgement. */
    if (APP_CYBERGEAR_STANDALONE_TEST) return;

    float send_angles[4] = {0};
    uint8_t txdata[16] = {0};

    // 1. 各モーターのフィードバックから現在角度(position_rad)を取得
    send_angles[2] = robstride_handler[RIGHT_RS03_INDEX].feedback.position_rad + 1.884f;
    send_angles[1] = -(robstride_handler[LEFT_RS03_INDEX].feedback.position_rad + 1.0f);
    send_angles[3] = -(robstride_handler[EL05_INDEX].feedback.position_rad + 2.963f);
    send_angles[0] = cybergear_base.feedback.position_rad;

    // 2. float (4つ) を uint8_t配列 (16バイト) に変換
    board_encode_angles(send_angles, txdata);

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
  if (!APP_CYBERGEAR_STANDALONE_TEST) MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
#if APP_CYBERGEAR_STANDALONE_TEST
  cybergear_standalone_test_run();
#else
  motor_CAN_txheader_init(&motor_txheader);
  motor_prepare_handlers();
  if (!cybergear_init(&cybergear_base, &hfdcan3, CYBER_GEAR_ID, HOST_ID))
  {
    Error_Handler();
    return 1;
  }
  if (inter_board_CAN_RxTxSettings_init(&inter_board_txheader) != HAL_OK ||
      motor_CAN_RxTxSettings_init(&motor_txheader) != HAL_OK)
  {
    motor_init_failed("CAN setup");
    return 1;
  }

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
    motor_init_failed("CyberGear startup/configuration");
    return 1;
  }

  if (!cybergear_homing())
  {
    motor_init_failed("CyberGear homing");
    return 1;
  }
  if (!motor_start_control())
  {
    motor_init_failed("motor startup confirmation or control timer");
    return 1;
  }
  printf("Motor initialization complete: %lu ms\r\n",
         (unsigned long)(HAL_GetTick() - motor_init_started_ms));
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    motor_check_feedback();
    motor_return_update();
    cybergear_service(&cybergear_base);
    
    el05_debug_print();
    cybergear_debug_print();
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
  huart2.Init.BaudRate = CYBERGEAR_UART_BAUD_RATE;
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
#if !APP_CYBERGEAR_STANDALONE_TEST
  GPIO_InitStruct.Pin = limit_sw_0_Pin|limit_sw_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

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
#if !APP_CYBERGEAR_STANDALONE_TEST
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
#endif

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




int _write(int file,char *ptr,int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, CYBERGEAR_UART_TX_TIMEOUT_MS);
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
