#include "motor_app.h"
#include "motor_config.h"
#include "board_protocol.h"
#include "can_init.h"
#include "robstride_app.h"
#include "cybergear_homing.h"

#include <stdio.h>
#include <math.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim6;

static FDCAN_TxHeaderTypeDef inter_board_txheader;
static FDCAN_TxHeaderTypeDef motor_txheader;
static RobstrideMotor robstride_handler[ROBSTRIDE_MOTOR_COUNT] = {0};
static CyberGearMotor cybergear_base;

static volatile float target_angle[BOARD_AXIS_COUNT] = {0.0f, 0.0f, 2.0f, 0.0f};
static volatile bool motor_init_requested = false;
static volatile bool motors_running = false;
static volatile bool motor_return_active = false;
static volatile uint32_t motor_return_started_ms = 0U;
static volatile uint32_t motor_last_feedback_ms = 0U;
static volatile uint32_t motor_can_rx_count = 0U;
static volatile uint32_t motor_can_last_rx_id = 0U;
static volatile uint32_t motor_can_last_rx_dlc = 0U;
static volatile uint32_t motor_can_last_rx_id_type = 0U;

static void motor_prepare_handlers(void)
{
  static const uint8_t motor_ids[ROBSTRIDE_MOTOR_COUNT] = {
    [RIGHT_RS03_INDEX] = RIGHT_RS03_ID,
    [LEFT_RS03_INDEX] = LEFT_RS03_ID,
    [EL05_INDEX] = EL05_ID
  };
  for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
  {
    robstride_handler[i].host_id = HOST_ID;
    robstride_handler[i].motor_id = motor_ids[i];
    robstride_handler[i].txheader = motor_txheader;
  }
  cybergear_init(&cybergear_base, &hfdcan3, CYBER_GEAR_ID, HOST_ID);
}

static bool motor_stop_all(void)
{
  /* Prevent cyclic commands from undoing STOP while IRQs remain available for RX. */
  motors_running = false;
  motor_return_active = false;
  cybergear_base.adrc.active = false;
  HAL_TIM_Base_Stop_IT(&htim6);

  const uint32_t started_ms = HAL_GetTick();
  uint32_t initial_rx_count[ROBSTRIDE_MOTOR_COUNT] = {0};
  uint32_t queued_mask = 0U;
  uint32_t stopped_mask = 0U;
  const uint32_t all_mask = (1U << (ROBSTRIDE_MOTOR_COUNT + 1U)) - 1U;
  do
  {
    /* Four motors exceed the three TX FIFO slots: pace every attempt, including
       failed ones, and never let one missing motor skip STOP for another. */
    uint32_t send_mask = __get_PRIMASK();
    __disable_irq();
    if ((queued_mask & 1U) == 0U) cybergear_base.feedback.online = false;
    if (cybergear_stop(&cybergear_base)) queued_mask |= 1U;
    __set_PRIMASK(send_mask);
    HAL_Delay(MOTOR_POLL_INTERVAL_MS);
    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
      send_mask = __get_PRIMASK();
      __disable_irq();
      if ((queued_mask & (1U << (i + 1U))) == 0U)
      {
        initial_rx_count[i] = robstride_handler[i].feedback.received_count;
      }
      if (robstride_stop(&robstride_handler[i])) queued_mask |= 1U << (i + 1U);
      __set_PRIMASK(send_mask);
      HAL_Delay(MOTOR_POLL_INTERVAL_MS);
    }

    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = HAL_GetTick();
    const CyberGearFeedback cg = cybergear_base.feedback;
    stopped_mask = (cg.online && cg.mode == 0U &&
        now_ms - cg.last_received_ms < ROBSTRIDE_FEEDBACK_TIMEOUT_MS) ? 1U : 0U;
    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
      const RobstrideFeedback feedback = robstride_handler[i].feedback;
      if (feedback.online && feedback.mode == 0U &&
          feedback.received_count != initial_rx_count[i] &&
          now_ms - feedback.last_leceived_ms < ROBSTRIDE_FEEDBACK_TIMEOUT_MS)
      {
        stopped_mask |= 1U << (i + 1U);
      }
    }
    __set_PRIMASK(mask);
    if ((queued_mask & stopped_mask) == all_mask) return true;
  } while (HAL_GetTick() - started_ms < MOTOR_STOP_TIMEOUT_MS);

  printf("Motor STOP unconfirmed: queued=0x%lX stopped=0x%lX\r\n",
         (unsigned long)queued_mask, (unsigned long)stopped_mask);
  return false;
}

static void motor_init_failed(const char *reason)
{
  motor_stop_all();
  printf("Motor initialization failed: %s\r\n", reason);
  Error_Handler();
}

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
  /* Startup/homing have their own deadlines and must reach STOP on failure. */
  if (!motors_running) return;
  /* Snapshot the tick with the ISR timestamp to avoid a receive-time race. */
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t last_feedback_ms = motor_last_feedback_ms;
  __set_PRIMASK(interrupt_mask);
  if ((uint32_t)(now_ms - last_feedback_ms) >= MOTOR_INIT_FEEDBACK_TIMEOUT_MS)
  {
    motor_stop_all();
    motor_init_print_can_status();
    printf("No motor feedback for 3 seconds; resetting MCU\r\n");
    NVIC_SystemReset();
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
    motor_app_receive_command(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    motor_return_active = false;
    completed = true;
  }
  __set_PRIMASK(interrupt_mask);
  if (completed)
  {
    printf("Motor return: 2 seconds elapsed; CAN commands accepted\r\n");
  }
}

static void cybergear_debug_print(void)
{
  static uint32_t last_print_ms = 0U;
  if ((uint32_t)(HAL_GetTick() - last_print_ms) < CYBERGEAR_DEBUG_INTERVAL_MS)
  {
    return;
  }

  /* Snapshot shared state; keep UART output outside the critical section. */
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t now_ms = HAL_GetTick();
  const CyberGearFeedback feedback = cybergear_base.feedback;
  const CyberGearAdrcState adrc = cybergear_base.adrc;
  const float target_rad = target_angle[0];
  __set_PRIMASK(interrupt_mask);
  last_print_ms = now_ms;

  /* Short lines fit the existing UART write timeout at 115200 baud. */
  printf("CG t=%lu pos=%.3f target=%.3f ref=%.3f rad\r\n",
         (unsigned long)now_ms, (double)feedback.position_rad,
         (double)target_rad, (double)adrc.reference_rad);
  printf("CG iq_cmd=%.3f A vel=%.3f rad/s torque=%.3f Nm\r\n",
         (double)adrc.current_a, (double)feedback.velocity_rad_s,
         (double)feedback.torque_nm);
  printf("CG err=%.3f est_vel=%.3f dist=%.3f\r\n",
         (double)(adrc.reference_rad - feedback.position_rad),
         (double)adrc.velocity_rad_s, (double)adrc.disturbance_rad_s2);
  printf("CG active=%u init=%u online=%u mode=%u fault=0x%02X age=%lu ms\r\n",
         (unsigned int)adrc.active, (unsigned int)adrc.initialized,
         (unsigned int)feedback.online, (unsigned int)feedback.mode,
         (unsigned int)feedback.fault_flags,
         (unsigned long)(now_ms - feedback.last_received_ms));
}

static bool robstride_init(uint32_t started_ms)
{
  enum InitStage {
    SEND_STOP, WAIT_STOPPED, SET_MODE, SET_VELOCITY, SET_ACCELERATION,
    SET_CURRENT, SET_HOLD, SEND_ENABLE, WAIT_RUNNING, READY
  };
  struct InitState {
    enum InitStage stage;
    bool waiting_for_write;
    uint32_t baseline_count;
    uint32_t last_probe_ms;
    uint32_t enabled_ms;
    float hold_position;
  } state[ROBSTRIDE_MOTOR_COUNT] = {0};
  uint32_t index = 0U;

  while (HAL_GetTick() - started_ms < ROBSTRIDE_INIT_TIMEOUT_MS)
  {
    RobstrideFeedback feedback[ROBSTRIDE_MOTOR_COUNT];
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = HAL_GetTick();
    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
      feedback[i] = robstride_handler[i].feedback;
    }
    __set_PRIMASK(interrupt_mask);

    bool all_ready = true;
    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
      const bool fresh = feedback[i].online &&
          now_ms - feedback[i].last_leceived_ms < ROBSTRIDE_FEEDBACK_TIMEOUT_MS;
      const bool healthy = fresh && feedback[i].fault_flags == 0U &&
          isfinite(feedback[i].position_rad);
      if (state[i].stage == READY && (!healthy || feedback[i].mode != 2U))
      {
        state[i].stage = SEND_STOP;
        state[i].waiting_for_write = false;
      }
      /* Wait out reported faults with STOP probes; do not clear/re-enable them. */
      if (state[i].stage >= SET_MODE && state[i].stage < READY &&
          (!healthy || (state[i].stage <= SEND_ENABLE && feedback[i].mode != 0U)))
      {
        state[i].stage = SEND_STOP;
        state[i].waiting_for_write = false;
      }
      if (state[i].stage == WAIT_RUNNING && healthy && feedback[i].mode == 2U &&
          feedback[i].received_count != state[i].baseline_count)
      {
        state[i].stage = READY;
      }
      if (state[i].stage != READY) all_ready = false;
    }
    if (all_ready) return true;

    /* One command per poll keeps FIFO space for replies and retries. Axes make
       progress independently, so a slow power-up does not block healthy axes. */
    struct InitState *s = &state[index];
    RobstrideMotor *motor = &robstride_handler[index];
    const RobstrideFeedback f = feedback[index];
    bool write_queued = false;
    if (s->waiting_for_write)
    {
      if (f.received_count != s->baseline_count)
      {
        /* Type-18 writes return type-2 feedback. Do not advance merely on HAL_OK. */
        s->waiting_for_write = false;
        if (s->stage == SET_MODE) motor->run_mode = POSITION_PP;
        s->stage = (enum InitStage)(s->stage + 1);
      }
      else if (now_ms - s->last_probe_ms >= ROBSTRIDE_INIT_PROBE_INTERVAL_MS)
      {
        s->waiting_for_write = false;
      }
      index = (index + 1U) % ROBSTRIDE_MOTOR_COUNT;
      HAL_Delay(MOTOR_POLL_INTERVAL_MS);
      continue;
    }
    switch (s->stage)
    {
      case SEND_STOP:
        s->baseline_count = f.received_count;
        if (robstride_stop(motor))
        {
          s->last_probe_ms = now_ms;
          s->stage = WAIT_STOPPED;
        }
        break;
      case WAIT_STOPPED:
        if (f.online && f.received_count != s->baseline_count && f.mode == 0U &&
            f.fault_flags == 0U && isfinite(f.position_rad) &&
            now_ms - f.last_leceived_ms < ROBSTRIDE_FEEDBACK_TIMEOUT_MS)
        {
          s->hold_position = f.position_rad;
          s->stage = SET_MODE;
        }
        else if (now_ms - s->last_probe_ms >= ROBSTRIDE_INIT_PROBE_INTERVAL_MS)
        {
          robstride_stop(motor);
          s->last_probe_ms = now_ms;
        }
        break;
      case SET_MODE:
        write_queued = robstride_set_run_mode(motor, POSITION_PP);
        break;
      case SET_VELOCITY:
        write_queued = robstride_set_pp_velocity_max(motor, MOTOR_PP_VELOCITY_RAD_S);
        break;
      case SET_ACCELERATION:
        write_queued = robstride_set_pp_acceleration(motor, MOTOR_PP_ACCELERATION_RAD_S2);
        break;
      case SET_CURRENT:
        write_queued = robstride_set_current_limit(motor, MOTOR_PP_CURRENT_LIMIT_A);
        break;
      case SET_HOLD:
        write_queued = robstride_set_position(motor, s->hold_position);
        break;
      case SEND_ENABLE:
        s->baseline_count = f.received_count;
        if (robstride_enable(motor))
        {
          s->enabled_ms = now_ms;
          s->last_probe_ms = now_ms;
          s->stage = WAIT_RUNNING;
        }
        break;
      case WAIT_RUNNING:
        if (now_ms - s->enabled_ms >= ROBSTRIDE_INIT_RETRY_INTERVAL_MS)
        {
          s->stage = SEND_STOP;
          break;
        }
        /* Parameter writes request feedback without another Enable. */
        /* fall through */
      case READY:
        if (now_ms - s->last_probe_ms >= ROBSTRIDE_INIT_PROBE_INTERVAL_MS)
        {
          robstride_set_current_limit(motor, MOTOR_PP_CURRENT_LIMIT_A);
          s->last_probe_ms = now_ms;
        }
        break;
    }
    if (write_queued)
    {
      s->baseline_count = f.received_count;
      s->last_probe_ms = now_ms;
      s->waiting_for_write = true;
    }
    index = (index + 1U) % ROBSTRIDE_MOTOR_COUNT;
    HAL_Delay(MOTOR_POLL_INTERVAL_MS);
  }

  for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
  {
    printf("RS startup timeout: id=%u stage=%u\r\n",
           (unsigned int)robstride_handler[i].motor_id, (unsigned int)state[i].stage);
  }
  return false;
}

static bool cybergear_base_init(void)
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

  /* Establish feedback before enabling motion. A queued frame is not proof
     of delivery, so retry STOP while the motor is powering up. */
  const uint32_t wait_started_ms = HAL_GetTick();
  uint32_t last_probe_ms = wait_started_ms - MOTOR_PROBE_INTERVAL_MS;
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
    if ((uint32_t)(now_ms - last_probe_ms) >= MOTOR_PROBE_INTERVAL_MS)
    {
      /* A full queue is retried on the next probe without enabling the motor. */
      cybergear_stop(&cybergear_base);
      last_probe_ms = now_ms;
    }
    HAL_Delay(MOTOR_POLL_INTERVAL_MS);
  }


  if (!cybergear_set_run_mode(
    &cybergear_base,
    CYBERGEAR_RUN_MODE_OPERATION
  ))
  {
    return false;
  }
  HAL_Delay(MOTOR_POLL_INTERVAL_MS);

  return cybergear_enable(&cybergear_base);
}

void motor_app_receive_feedback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
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

    /* Count all frames before protocol/ID checks to diagnose rejected replies. */
    motor_can_rx_count++;
    motor_can_last_rx_id = rxheader.Identifier;
    motor_can_last_rx_dlc = rxheader.DataLength;
    motor_can_last_rx_id_type = rxheader.IdType;

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
    }

    if (cybergear_parse_feedback(
      &cybergear_base,
      rxheader.Identifier,
      rxdata
    ))
    {
      continue;
    }

    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
      if (robstride_handler[i].motor_id == motor_id)
      {
        robstride_parse_feedback(rxheader.Identifier, rxdata, &robstride_handler[i].feedback);
        break;
      }
    }
  }
}

void motor_app_receive_command(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
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
      case BOARD_INIT_CAN_ID:
      {
        static bool has_previous_command = false;
        static int32_t previous_command = 0;
        /* The first int32 is big-endian, like the other inter-board values. */
        if (rxheader.IdType == FDCAN_STANDARD_ID &&
            rxheader.RxFrameType == FDCAN_DATA_FRAME &&
            rxheader.DataLength >= FDCAN_DLC_BYTES_4)
        {
          const int32_t command = board_decode_init_command(rxdata);
          /* Use the first valid command as the baseline for change detection. */
          if (!motor_return_active && has_previous_command && command != previous_command)
          {
            if (motors_running)
            {
              const uint32_t interrupt_mask = __get_PRIMASK();
              __disable_irq();
              motor_return_active = true;
              target_angle[BOARD_AXIS_BASE] = 0.140f;
              target_angle[BOARD_AXIS_LEFT] = -0.900f;
              target_angle[BOARD_AXIS_RIGHT] = 1.98f;
              motor_return_started_ms = HAL_GetTick();
              __set_PRIMASK(interrupt_mask);
            }
            else if (previous_command == 0 && command == 1)
            {
              /* Initialization requires a received 0 -> 1 transition. */
              motor_init_requested = true;
            }
          }
          /* Track ignored commands too, so they are not replayed after return. */
          previous_command = command;
          has_previous_command = true;
        }
        break;
      }
      case BOARD_TARGET_CAN_ID:
        if (motor_return_active || rxheader.IdType != FDCAN_STANDARD_ID ||
            rxheader.RxFrameType != FDCAN_DATA_FRAME || rxheader.DataLength != FDCAN_DLC_BYTES_16)
        {
          break;
        }
        float received_floats[BOARD_AXIS_COUNT];
        board_decode_angles(rxdata, received_floats);
        for (uint32_t i = 0; i < BOARD_AXIS_COUNT; ++i)
        {
          target_angle[i] = received_floats[i];
        }

        break;
      default:
        break;
    }
  }
}

void motor_app_control_tick(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6 && motors_running)
  {
    static uint8_t control_phase = 0U;

    /* Stagger four commands across 1 ms ticks; each motor runs at 10 ms. */
    switch (control_phase)
    {
      case 0U:
        cybergear_control_position_adrc(&cybergear_base, target_angle[BOARD_AXIS_BASE]);
        break;
      case 1U:
        robstride_set_position(&robstride_handler[RIGHT_RS03_INDEX],
            board_target_to_motor(BOARD_AXIS_RIGHT, target_angle[BOARD_AXIS_RIGHT]));
        break;
      case 2U:
        robstride_set_position(&robstride_handler[LEFT_RS03_INDEX],
            board_target_to_motor(BOARD_AXIS_LEFT, target_angle[BOARD_AXIS_LEFT]));
        break;
      case 3U:
        robstride_set_position(&robstride_handler[EL05_INDEX],
            board_target_to_motor(BOARD_AXIS_EL05, target_angle[BOARD_AXIS_EL05]));
        break;
      default:
        break;
    }

    control_phase++;
    if (control_phase < MOTOR_CONTROL_PHASE_COUNT)
    {
      return;
    }
    control_phase = 0U;

    float send_angles[BOARD_AXIS_COUNT] = {0};
    uint8_t txdata[BOARD_ANGLE_PAYLOAD_BYTES] = {0};

    send_angles[BOARD_AXIS_RIGHT] = board_position_from_motor(BOARD_AXIS_RIGHT,
        robstride_handler[RIGHT_RS03_INDEX].feedback.position_rad);
    send_angles[BOARD_AXIS_LEFT] = board_position_from_motor(BOARD_AXIS_LEFT,
        robstride_handler[LEFT_RS03_INDEX].feedback.position_rad);
    send_angles[BOARD_AXIS_EL05] = board_position_from_motor(BOARD_AXIS_EL05,
        robstride_handler[EL05_INDEX].feedback.position_rad);
    send_angles[BOARD_AXIS_BASE] = cybergear_base.feedback.position_rad;

    board_encode_angles(send_angles, txdata);
    inter_board_txheader.Identifier = BOARD_FEEDBACK_CAN_ID;
    inter_board_txheader.DataLength = FDCAN_DLC_BYTES_16;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &inter_board_txheader, txdata);
  }
}

void motor_app_start(void)
{
  /* Populate every STOP destination even if a later startup stage fails. */
  motor_CAN_txheader_init(&motor_txheader);
  motor_prepare_handlers();
  if (inter_board_CAN_RxTxSettings_init(&inter_board_txheader) != HAL_OK)
  {
    Error_Handler();
    return;
  }
  if (motor_CAN_RxTxSettings_init(&motor_txheader) != HAL_OK)
  {
    motor_init_failed("CAN setup");
    return;
  }

  /* CAN reception is active; defer homing, enable and cyclic commands. */
  while (!motor_init_requested)
  {
    HAL_Delay(MOTOR_POLL_INTERVAL_MS);
  }

  const uint32_t motor_init_started_ms = HAL_GetTick();
  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  motor_last_feedback_ms = HAL_GetTick();
  __set_PRIMASK(interrupt_mask);

  if (!cybergear_base_init())
  {
    motor_init_failed("CyberGear startup");
    return;
  }

  const CyberGearHomingContext homing = {
    &cybergear_base, motor_check_feedback, motor_init_print_can_status
  };
  if (!cybergear_homing_run(&homing))
  {
    motor_init_failed("CyberGear homing");
    return;
  }
  const uint32_t robstride_started_ms = HAL_GetTick();
  while (1)
  {
    if (!robstride_init(robstride_started_ms))
    {
      motor_init_failed("RobStride confirmation timeout");
      return;
    }
    /* Start ADRC after blocking initialization, immediately before cyclic commands. */
    if (!cybergear_start_position_adrc(&cybergear_base))
    {
      motor_init_failed("CyberGear ADRC startup");
      return;
    }

    /* ADRC startup includes UART and HAL delays. Recheck the latest RS state
       before starting TIM6, without a receive callback between check and start. */
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = HAL_GetTick();
    bool ready = now_ms - robstride_started_ms < ROBSTRIDE_INIT_TIMEOUT_MS;
    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
      const RobstrideFeedback feedback = robstride_handler[i].feedback;
      if (!feedback.online || feedback.fault_flags != 0U || feedback.mode != 2U ||
          !isfinite(feedback.position_rad) ||
          now_ms - feedback.last_leceived_ms >= ROBSTRIDE_FEEDBACK_TIMEOUT_MS)
      {
        ready = false;
      }
    }
    HAL_StatusTypeDef timer_status = HAL_ERROR;
    if (ready)
    {
      timer_status = HAL_TIM_Base_Start_IT(&htim6);
      motors_running = timer_status == HAL_OK;
    }
    __set_PRIMASK(mask);
    if (ready)
    {
      if (timer_status != HAL_OK)
      {
        motor_init_failed("control timer startup");
        return;
      }
      break;
    }
    /* A transient loss during handoff can recover within the original budget. */
    cybergear_stop(&cybergear_base);
  }
  printf("Motor initialization complete: %lu ms\r\n",
         (unsigned long)(HAL_GetTick() - motor_init_started_ms));
}

void motor_app_process(void)
{
  motor_check_feedback();
  motor_return_update();
  cybergear_debug_print();
  HAL_Delay(MOTOR_POLL_INTERVAL_MS);
}
