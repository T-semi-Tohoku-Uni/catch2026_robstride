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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  CG_HOME_SET_MODE,
  CG_HOME_SET_VELOCITY,
  CG_HOME_ENABLE,
  CG_HOME_STOP,
  CG_HOME_SET_ZERO
} CyberGearHomingCommand;
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
static volatile bool motor_initializing = false;
static volatile bool motors_running = false;
static volatile bool motor_stop_pending = false;
static volatile bool motor_return_active = false;
static volatile uint32_t motor_return_started_ms = 0U;
static volatile bool motor_init_feedback_received = false;
static volatile uint32_t motor_last_feedback_ms = 0U;
static volatile uint32_t motor_can_rx_count = 0U;
static volatile uint32_t motor_can_last_rx_id = 0U;
static volatile uint32_t motor_can_last_rx_dlc = 0U;
static volatile uint32_t motor_can_last_rx_id_type = 0U;
static RobstrideStartup robstride_startup;
static bool robstride_mode_pending[3] = {false};
static uint32_t robstride_mode_since_ms[3] = {0U};
static bool diagnostic_uart_ready = false;
static uint32_t motor_stop_queued_mask = 0U;
static uint32_t motor_stopped_mask = 0U;
static uint32_t motor_stop_baseline[3] = {0U};
static uint32_t motor_stop_cybergear_baseline = 0U;
static uint32_t motor_stop_next_axis = 0U;
static uint32_t motor_stop_last_probe_ms = 0U;
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
static bool motor_stop_all(void);
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
  if (!motors_running) return;
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t now_ms = HAL_GetTick();
  const bool stop_requested = cybergear_base.fault == CG_FAULT_NONE &&
      (cybergear_base.state == CG_STATE_STOPPING || cybergear_base.state == CG_STATE_STOPPED);
  bool healthy = cybergear_base.state == CG_STATE_RUNNING &&
      cybergear_base.feedback.online && cybergear_base.feedback.fault_flags == 0U &&
      now_ms - cybergear_base.feedback.last_received_ms <= cybergear_base.config.controller.feedback_timeout_ms;
  for (uint32_t index = 0U; index < 3U; ++index)
  {
    const RobstrideFeedback feedback = robstride_handler[index].feedback;
    if (feedback.mode == 2U)
    {
      robstride_mode_pending[index] = false;
    }
    else if (!robstride_mode_pending[index])
    {
      robstride_mode_pending[index] = true;
      robstride_mode_since_ms[index] = now_ms;
    }
    const bool mode_valid = !robstride_mode_pending[index] ||
        now_ms - robstride_mode_since_ms[index] <= ROBSTRIDE_STARTUP_FEEDBACK_MS;
    healthy = healthy && feedback.online && feedback.fault_flags == 0U &&
        mode_valid && isfinite(feedback.position_rad) &&
        now_ms - feedback.last_leceived_ms <= ROBSTRIDE_STARTUP_FEEDBACK_MS;
  }
  __set_PRIMASK(interrupt_mask);
  if (stop_requested)
  {
    if (motor_stop_all())
      printf("Motors stopped; waiting for initialization request\r\n");
    else
      printf("Motor STOP confirmation pending; retrying\r\n");
  }
  else if (!healthy) motor_init_failed("motor feedback or control fault");
}

static void motor_stop_update(void)
{
  if (!motor_stop_pending) return;
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (cybergear_base.managed)
  {
    cybergear_control_position_adrc(&cybergear_base, 0.0f);
  }
  if (now_ms - motor_stop_last_probe_ms >= 10U)
  {
    if (motor_stop_next_axis == 3U && !cybergear_base.managed && (motor_stopped_mask & 8U) == 0U)
    {
      const uint32_t sequence = cybergear_base.feedback.rx_sequence;
      if (cybergear_stop(&cybergear_base) && (motor_stop_queued_mask & 8U) == 0U)
      {
        motor_stop_cybergear_baseline = sequence;
        motor_stop_queued_mask |= 8U;
      }
    }
    else if (motor_stop_next_axis < 3U && (motor_stopped_mask & (1U << motor_stop_next_axis)) == 0U)
    {
      const uint32_t index = motor_stop_next_axis;
      const uint32_t sequence = robstride_handler[index].feedback.received_count;
      if (robstride_stop(&robstride_handler[index]) && (motor_stop_queued_mask & (1U << index)) == 0U)
      {
        motor_stop_baseline[index] = sequence;
        motor_stop_queued_mask |= 1U << index;
      }
    }
    motor_stop_next_axis = (motor_stop_next_axis + 1U) % 4U;
    motor_stop_last_probe_ms = now_ms;
  }
  const uint32_t sample_ms = HAL_GetTick();
  for (uint32_t index = 0U; index < 3U; ++index)
  {
    const RobstrideFeedback feedback = robstride_handler[index].feedback;
    if (feedback.online && feedback.received_count != motor_stop_baseline[index] &&
        feedback.mode == 0U && isfinite(feedback.velocity_rps) &&
        fabsf(feedback.velocity_rps) <= 0.05f &&
        sample_ms - feedback.last_leceived_ms <= ROBSTRIDE_STARTUP_FEEDBACK_MS)
      motor_stopped_mask |= 1U << index;
  }
  if (cybergear_base.managed || cybergear_base.state == CG_STATE_STOPPED)
  {
    if (cybergear_base.stop_queued) motor_stop_queued_mask |= 8U;
    if (cybergear_base.reset_confirmed && cybergear_base.stationary &&
        cybergear_base.feedback.mode == 0U &&
        sample_ms - cybergear_base.feedback.last_received_ms <= cybergear_base.config.controller.feedback_timeout_ms)
      motor_stopped_mask |= 8U;
  }
  else
  {
    const CyberGearFeedback feedback = cybergear_base.feedback;
    if (feedback.online && feedback.rx_sequence != motor_stop_cybergear_baseline && feedback.mode == 0U &&
        isfinite(feedback.velocity_rad_s) && fabsf(feedback.velocity_rad_s) <= 0.05f &&
        sample_ms - feedback.last_received_ms <= CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS)
      motor_stopped_mask |= 8U;
  }
  if ((motor_stop_queued_mask & motor_stopped_mask) == 15U) motor_stop_pending = false;
  __set_PRIMASK(mask);
}

static bool motor_stop_all(void)
{
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  motors_running = false;
  motor_return_active = false;
  motor_init_requested = false;
  motor_stop_pending = true;
  motor_stop_queued_mask = 0U;
  motor_stopped_mask = 0U;
  for (uint32_t index = 0U; index < 3U; ++index)
    motor_stop_baseline[index] = robstride_handler[index].feedback.received_count;
  motor_stop_cybergear_baseline = cybergear_base.feedback.rx_sequence;
  motor_stop_next_axis = 0U;
  motor_stop_last_probe_ms = HAL_GetTick() - 10U;
  if (cybergear_base.managed) cybergear_stop(&cybergear_base);
  __set_PRIMASK(interrupt_mask);
  HAL_TIM_Base_Stop_IT(&htim6);
  const uint32_t started_ms = HAL_GetTick();
  do
  {
    motor_stop_update();
    if (!motor_stop_pending) return true;
    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
  } while (HAL_GetTick() - started_ms < MOTOR_STOP_TIMEOUT_MS);
  printf("Motor STOP unconfirmed: queued=0x%lX stopped=0x%lX\r\n",
         (unsigned long)motor_stop_queued_mask, (unsigned long)motor_stopped_mask);
  return false;
}

static void motor_init_failed(const char *reason)
{
  motor_stop_all();
  printf("Motor initialization/control failed: %s\r\n", reason);
  motor_init_print_can_status();
  motor_init_requested = false;
  printf("Waiting for a new motor initialization request\r\n");
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
  if (!motors_running || !diagnostic_uart_ready) return;
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
  if (!diagnostic_uart_ready) return;
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

static bool cybergear_homing_send(CyberGearHomingCommand command, float value)
{
  const uint32_t started_ms = HAL_GetTick();
  do
  {
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const bool faulted = cybergear_base.feedback.fault_flags != 0U ||
        cybergear_base.motor_fault_sequence != 0U;
    const bool feedback_valid = command != CG_HOME_SET_VELOCITY || value == 0.0f ||
        (cybergear_base.feedback.online && isfinite(cybergear_base.feedback.position_rad) &&
         isfinite(cybergear_base.feedback.velocity_rad_s) &&
         HAL_GetTick() - cybergear_base.feedback.last_received_ms <= CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS);
    bool queued = false;
    if ((!faulted || command == CG_HOME_STOP) && feedback_valid)
    {
      switch (command)
      {
        case CG_HOME_SET_MODE:
          queued = cybergear_set_run_mode(&cybergear_base, CYBERGEAR_RUN_MODE_SPEED);
          break;
        case CG_HOME_SET_VELOCITY:
          queued = cybergear_set_velocity(&cybergear_base, value);
          break;
        case CG_HOME_ENABLE:
          queued = cybergear_enable(&cybergear_base);
          break;
        case CG_HOME_STOP:
          queued = cybergear_stop(&cybergear_base);
          break;
        case CG_HOME_SET_ZERO:
          queued = cybergear_set_zero(&cybergear_base);
          break;
      }
    }
    __set_PRIMASK(interrupt_mask);
    if (queued) return true;
    if ((faulted && command != CG_HOME_STOP) || !feedback_valid) return false;
    HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
  } while (HAL_GetTick() - started_ms < CYBERGEAR_HOMING_PROBE_INTERVAL_MS);
  return false;
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
      (uint32_t)(now_ms - feedback->last_received_ms) <= CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS;
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
    if (!cybergear_homing_send(CG_HOME_SET_VELOCITY, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
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
  if (!cybergear_homing_send(CG_HOME_SET_VELOCITY, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
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
    if (!cybergear_homing_send(CG_HOME_SET_VELOCITY, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
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
    if (!cybergear_homing_send(CG_HOME_SET_VELOCITY, speed_rad_s))
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
  // 1. 速度制御モードに変更して有効化
  if (!cybergear_homing_send(CG_HOME_SET_MODE, 0.0f))
  {
    return false;
  }
  HAL_Delay(CYBERGEAR_HOMING_POLL_MS);
  if (!cybergear_homing_send(CG_HOME_SET_VELOCITY, 0.0f))
  {
    cybergear_stop(&cybergear_base);
    return false;
  }
  const uint32_t feedback_wait_started_ms = HAL_GetTick();
  uint32_t last_enable_ms = feedback_wait_started_ms;
  if (!cybergear_homing_send(CG_HOME_ENABLE, 0.0f))
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
    if ((uint32_t)(now_ms - feedback_wait_started_ms) >= CYBERGEAR_HOMING_STARTUP_TIMEOUT_MS)
    {
      cybergear_stop(&cybergear_base);
      return false;
    }
    /* Retry a lost enable response while the speed reference is still zero. */
    if ((uint32_t)(now_ms - last_enable_ms) >= CYBERGEAR_HOMING_PROBE_INTERVAL_MS)
    {
      if (!cybergear_homing_send(CG_HOME_ENABLE, 0.0f))
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

  if (!cybergear_homing_send(CG_HOME_SET_VELOCITY, CYBERGEAR_HOMING_FAST_SPEED_RAD_S))
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
    if (!cybergear_homing_send(CG_HOME_SET_VELOCITY, direction * CYBERGEAR_HOMING_FAST_SPEED_RAD_S))
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

  if (!cybergear_homing_send(CG_HOME_STOP, 0.0f))
  {
    return false;
  }
  /* Allow STOP to be sent before setting zero; no position/settling check. */
  HAL_Delay(CYBERGEAR_HOMING_POLL_MS);

  if (!cybergear_homing_send(CG_HOME_SET_ZERO, 0.0f))
  {
    return false;
  }
  HAL_Delay(CYBERGEAR_HOMING_POLL_MS);

  printf("CG home edges=%.4f,%.4f center=%.4f rad\r\n",
         (double)reverse_edge_rad, (double)forward_edge_rad, (double)center_rad);
  /* Keep stopped until all blocking motor initialization is complete. */
  return true;
}

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

bool cybergear_base_init(void)
{
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
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const CyberGearFeedback feedback = cybergear_base.feedback;
    __set_PRIMASK(interrupt_mask);
    const uint32_t now_ms = HAL_GetTick();
    if (feedback.online &&
        (uint32_t)(now_ms - feedback.last_received_ms) <= CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS)
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

static bool motor_start_control(void)
{
  const float current_limits_a[3] = {
    [RIGHT_RS03_INDEX] = ROBSTRIDE_RIGHT_RS03_CURRENT_LIMIT_A,
    [LEFT_RS03_INDEX] = ROBSTRIDE_LEFT_RS03_CURRENT_LIMIT_A,
    [EL05_INDEX] = ROBSTRIDE_EL05_CURRENT_LIMIT_A
  };
  if (!robstride_startup_init(&robstride_startup, robstride_handler, 3U, current_limits_a) ||
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

static void motor_initialize_requested(void)
{
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  if (!motor_init_requested || motors_running || motor_initializing || motor_stop_pending)
  {
    __set_PRIMASK(interrupt_mask);
    return;
  }
  motor_init_requested = false;
  motor_initializing = true;
  motor_init_feedback_received = false;
  motor_last_feedback_ms = HAL_GetTick();
  for (uint32_t index = 0U; index < 3U; ++index)
    robstride_mode_pending[index] = false;
  __set_PRIMASK(interrupt_mask);

  const uint32_t started_ms = HAL_GetTick();
  if (!cybergear_base_init())
    motor_init_failed("CyberGear startup/configuration");
  else if (!cybergear_homing())
    motor_init_failed("CyberGear homing");
  else if (!motor_start_control())
    motor_init_failed("motor startup confirmation or control timer");
  else
    printf("Motor initialization complete: %lu ms\r\n",
           (unsigned long)(HAL_GetTick() - started_ms));

  __disable_irq();
  motor_init_requested = false;
  motor_initializing = false;
  __set_PRIMASK(interrupt_mask);
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
          if (!motor_return_active && !motor_initializing && !motor_stop_pending &&
              has_previous_command && command != previous_command)
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
  if (htim == &htim6 && motors_running) {
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
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
  motor_CAN_txheader_init(&motor_txheader);
  motor_prepare_handlers();
  if (!cybergear_init(&cybergear_base, &hfdcan3, CYBER_GEAR_ID, HOST_ID))
  {
    Error_Handler();
  }
  if (inter_board_CAN_RxTxSettings_init(&inter_board_txheader) != HAL_OK ||
      motor_CAN_RxTxSettings_init(&motor_txheader) != HAL_OK)
  {
    motor_init_failed("CAN setup");
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    motor_stop_update();
    motor_initialize_requested();
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
    return;
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    return;
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    return;
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    return;
  }
  /* USER CODE BEGIN USART2_Init 2 */
  diagnostic_uart_ready = true;
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




int _write(int file,char *ptr,int len)
{
  if (diagnostic_uart_ready)
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
  // while (1)
  // {
  // }
  NVIC_SystemReset();
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
