#include "cybergear_homing.h"
#include "motor_config.h"

#include <math.h>
#include <stdio.h>

static bool cybergear_homing_feedback(const CyberGearHomingContext *context, CyberGearFeedback *feedback)
{
  context->check_feedback();
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  *feedback = context->motor->feedback;
  const uint32_t now_ms = HAL_GetTick();
  __set_PRIMASK(interrupt_mask);
  const bool valid = feedback->online && feedback->fault_flags == 0U &&
      isfinite(feedback->position_rad) && isfinite(feedback->velocity_rad_s) &&
      (uint32_t)(now_ms - feedback->last_received_ms) < CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS;
  if (!valid)
  {
    cybergear_stop(context->motor);
    printf("CG home feedback: online=%u fault=%u age=%lu\r\n",
           (unsigned int)feedback->online, (unsigned int)feedback->fault_flags,
           (unsigned long)(now_ms - feedback->last_received_ms));
  }
  return valid;
}

/* Continue past the edge so the next measurement approaches from the other side. */
static bool cybergear_homing_clear_edge(const CyberGearHomingContext *context, float direction)
{
  CyberGearFeedback feedback;
  if (!cybergear_homing_feedback(context, &feedback))
  {
    return false;
  }
  const float start_rad = feedback.position_rad;
  const uint32_t started_ms = HAL_GetTick();
  while ((feedback.position_rad - start_rad) * direction < CYBERGEAR_HOMING_CLEARANCE_RAD)
  {
    if ((uint32_t)(HAL_GetTick() - started_ms) >= CYBERGEAR_HOMING_PHASE_TIMEOUT_MS)
    {
      cybergear_stop(context->motor);
      printf("CG home clearance timeout\r\n");
      return false;
    }
    if (!cybergear_set_velocity(context->motor, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
    {
      cybergear_stop(context->motor);
      printf("CG home clearance TX failed\r\n");
      return false;
    }
    HAL_Delay(10);
    if (!cybergear_homing_feedback(context, &feedback))
    {
      return false;
    }
  }
  return true;
}

static bool cybergear_homing_measure_edge(const CyberGearHomingContext *context, float direction, float *edge_rad)
{
  /* Clearance can cross a narrow detection region completely. Use the actual
     state here rather than assuming the state seen at the previous edge. */
  const GPIO_PinState from_state = HAL_GPIO_ReadPin(limit_GPIO_Port, limit_Pin);
  if (!cybergear_set_velocity(context->motor, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
  {
    cybergear_stop(context->motor);
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
    if (!cybergear_homing_feedback(context, &feedback))
    {
      return false;
    }
    /* Ignore an edge crossed by residual motion in the previous direction. */
    if (HAL_GPIO_ReadPin(limit_GPIO_Port, limit_Pin) != from_state &&
        feedback.velocity_rad_s * direction > 0.0f)
    {
      if (!edge_pending)
      {
        /* Keep the first crossing angle, then confirm the switch for 20 ms. */
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
    if (!cybergear_set_velocity(context->motor, direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S))
    {
      cybergear_stop(context->motor);
      printf("CG home edge TX failed\r\n");
      return false;
    }
    HAL_Delay(10);
  }
  cybergear_stop(context->motor);
  printf("CG home edge timeout\r\n");
  return false;
}

static bool cybergear_homing_move_to_center(const CyberGearHomingContext *context, float center_rad)
{
  if (!isfinite(center_rad))
  {
    cybergear_stop(context->motor);
    printf("CG home center invalid target\r\n");
    return false;
  }
  const uint32_t started_ms = HAL_GetTick();
  CyberGearFeedback feedback = {0};
  if (!cybergear_homing_feedback(context, &feedback))
  {
    return false;
  }
  const float direction = (center_rad >= feedback.position_rad) ? 1.0f : -1.0f;
  const float speed_rad_s = direction * CYBERGEAR_HOMING_SLOW_SPEED_RAD_S;
  while ((uint32_t)(HAL_GetTick() - started_ms) < CYBERGEAR_HOMING_PHASE_TIMEOUT_MS)
  {
    if (!cybergear_homing_feedback(context, &feedback))
    {
      return false;
    }
    const float error_rad = center_rad - feedback.position_rad;
    /* Finish on reaching/crossing the average, without hunting or settling. */
    if (error_rad * direction <= 0.0f)
    {
      return true;
    }
    if (!cybergear_set_velocity(context->motor, speed_rad_s))
    {
      cybergear_stop(context->motor);
      printf("CG home center TX failed\r\n");
      context->print_can_status();
      return false;
    }
    HAL_Delay(10);
  }
  cybergear_stop(context->motor);
  printf("CG home center timeout: pos=%.5f target=%.5f rad\r\n",
         (double)feedback.position_rad, (double)center_rad);
  printf("CG home center vel=%.5f cmd=%.5f rad/s\r\n",
         (double)feedback.velocity_rad_s, (double)speed_rad_s);
  return false;
}

bool cybergear_homing_run(const CyberGearHomingContext *context)
{
  // 1. 速度制御モードに変更して有効化
  if (!cybergear_set_run_mode(context->motor, CYBERGEAR_RUN_MODE_SPEED))
  {
    return false;
  }
  HAL_Delay(10);
  if (!cybergear_set_velocity(context->motor, 0.0f))
  {
    cybergear_stop(context->motor);
    return false;
  }
  const uint32_t feedback_wait_started_ms = HAL_GetTick();
  uint32_t last_enable_ms = feedback_wait_started_ms;
  if (!cybergear_enable(context->motor))
  {
    return false;
  }

  /* Obtain a fresh starting angle before beginning the homing motion. */
  CyberGearFeedback feedback;
  while (1)
  {
    context->check_feedback();
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    feedback = context->motor->feedback;
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
      cybergear_stop(context->motor);
      return false;
    }
    /* Retry a lost enable response while the speed reference is still zero. */
    if ((uint32_t)(now_ms - last_enable_ms) >= MOTOR_PROBE_INTERVAL_MS)
    {
      if (!cybergear_enable(context->motor))
      {
        cybergear_stop(context->motor);
        context->print_can_status();
        return false;
      }
      last_enable_ms = now_ms;
    }
    HAL_Delay(10);
  }
  if (feedback.fault_flags != 0U)
  {
    cybergear_stop(context->motor);
    return false;
  }

  const float start_position_rad = feedback.position_rad;
  bool reversed = false;
  float direction = 1.0f;
  const uint32_t search_started_ms = HAL_GetTick();
  const GPIO_PinState initial_state = HAL_GPIO_ReadPin(limit_GPIO_Port, limit_Pin);

  if (!cybergear_set_velocity(context->motor, 1.0f))
  {
    cybergear_stop(context->motor);
    return false;
  }

  while (HAL_GPIO_ReadPin(limit_GPIO_Port, limit_Pin) == initial_state)
  {
    if (!cybergear_homing_feedback(context, &feedback) ||
        (uint32_t)(HAL_GetTick() - search_started_ms) >= CYBERGEAR_HOMING_PHASE_TIMEOUT_MS)
    {
      cybergear_stop(context->motor);
      return false;
    }

    /* Reverse once if the switch has not changed after 60 degrees. */
    if (!reversed &&
        feedback.position_rad - start_position_rad >= CYBERGEAR_HOMING_REVERSE_ANGLE_RAD &&
        HAL_GPIO_ReadPin(limit_GPIO_Port, limit_Pin) == initial_state)
    {
      direction = -1.0f;
      reversed = true;
    }
    if (!cybergear_set_velocity(context->motor, direction))
    {
      cybergear_stop(context->motor);
      return false;
    }
    HAL_Delay(10);
  }

  /* Discard the fast crossing; measure switch crossings at low speed both ways. */
  float reverse_edge_rad;
  float forward_edge_rad;
  /* clear_edge sends the slow command; avoid two back-to-back CAN writes. */
  if (!cybergear_homing_clear_edge(context, direction))
  {
    cybergear_stop(context->motor);
    printf("CG home failed: first clearance\r\n");
    return false;
  }
  if (!cybergear_homing_measure_edge(context, -direction, &reverse_edge_rad))
  {
    cybergear_stop(context->motor);
    printf("CG home failed: reverse edge\r\n");
    return false;
  }
  if (!cybergear_homing_clear_edge(context, -direction))
  {
    cybergear_stop(context->motor);
    printf("CG home failed: reverse clearance\r\n");
    return false;
  }
  if (!cybergear_homing_measure_edge(context, direction, &forward_edge_rad))
  {
    cybergear_stop(context->motor);
    printf("CG home failed: forward edge\r\n");
    return false;
  }
  const float center_rad = 0.5f * (reverse_edge_rad + forward_edge_rad);
  if (!cybergear_homing_move_to_center(context, center_rad))
  {
    cybergear_stop(context->motor);
    printf("CG home failed: center move\r\n");
    return false;
  }

  if (!cybergear_stop(context->motor))
  {
    return false;
  }
  /* Allow STOP to be sent before setting zero; no position/settling check. */
  HAL_Delay(10);

  if (!cybergear_set_zero(context->motor))
  {
    return false;
  }
  HAL_Delay(10);

  printf("CG home edges=%.4f,%.4f center=%.4f rad\r\n",
         (double)reverse_edge_rad, (double)forward_edge_rad, (double)center_rad);
  /* Keep stopped until all blocking motor initialization is complete. */
  return true;
}

