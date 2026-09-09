#include "motor_app.h"
#include "motor_config.h"
#include "board_protocol.h"
#include "can_init.h"
#include "robstride_app.h"
#include "cybergear_homing.h"

#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim6;

static FDCAN_TxHeaderTypeDef inter_board_txheader;
static FDCAN_TxHeaderTypeDef motor_txheader;
static RobstrideMotor robstride_handler[ROBSTRIDE_MOTOR_COUNT] = {0};
static CyberGearMotor cybergear_base;

static volatile float target_angle[BOARD_AXIS_COUNT] = {0.0f, 0.0f, 2.0f, 0.0f};
static volatile bool el05_initializing = false;
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
static uint32_t el05_startup_ms;
static uint32_t el05_initial_rx_count;
static bool el05_retry_pending;

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
    HAL_Delay(MOTOR_POLL_INTERVAL_MS);
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

static bool robstride_init(void)
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
    robstride_handler[i].run_mode = POSITION_PP;
    robstride_handler[i].txheader = motor_txheader;
  }

  for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
  {
    if (!robstride_start_position_pp_mode(&robstride_handler[i],
        MOTOR_PP_VELOCITY_RAD_S, MOTOR_PP_ACCELERATION_RAD_S2, MOTOR_PP_CURRENT_LIMIT_A))
    {
      return false;
    }
  }

  return true;
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
      motor_init_feedback_received = true;
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
  if (htim == &htim6)
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
        if (!el05_initializing)
        {
          robstride_set_position(&robstride_handler[EL05_INDEX],
              board_target_to_motor(BOARD_AXIS_EL05, target_angle[BOARD_AXIS_EL05]));
        }
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
  if (inter_board_CAN_RxTxSettings_init(&inter_board_txheader) != HAL_OK ||
      motor_CAN_RxTxSettings_init(&motor_txheader) != HAL_OK)
  {
    Error_Handler();
  }

  /* CAN reception is active; defer homing, enable and cyclic commands. */
  while (!motor_init_requested)
  {
    HAL_Delay(MOTOR_POLL_INTERVAL_MS);
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

  const CyberGearHomingContext homing = {
    &cybergear_base, motor_check_feedback, motor_init_print_can_status
  };
  if (!cybergear_homing_run(&homing))
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
  el05_startup_ms = HAL_GetTick();
  el05_initial_rx_count = robstride_handler[EL05_INDEX].feedback.received_count;
  el05_retry_pending = true;
}

void motor_app_process(void)
{
  motor_check_feedback();
  motor_return_update();
  /* Retry once after fresh feedback, only during startup. */
  if (el05_retry_pending)
  {
    const RobstrideFeedback feedback = robstride_handler[EL05_INDEX].feedback;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - el05_startup_ms) >= MOTOR_INIT_FEEDBACK_TIMEOUT_MS)
    {
      el05_retry_pending = false;
      printf("EL05 startup retry expired: no fresh feedback\r\n");
    }
    else if (feedback.online && feedback.received_count != el05_initial_rx_count &&
             (uint32_t)(now_ms - feedback.last_leceived_ms) < CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS)
    {
      /* Never re-arm after a fault, a later stop, or a motor power cycle. */
      el05_retry_pending = false;
      if (feedback.mode == 0U && feedback.fault_flags == 0U)
      {
        el05_initializing = true;
        const bool queued = robstride_start_position_pp_mode(
            &robstride_handler[EL05_INDEX],
            MOTOR_PP_VELOCITY_RAD_S, MOTOR_PP_ACCELERATION_RAD_S2, MOTOR_PP_CURRENT_LIMIT_A);
        el05_initializing = false;
        printf("EL05 startup retry after feedback: queued=%u\r\n", (unsigned int)queued);
      }
    }
  }

  cybergear_debug_print();
  HAL_Delay(MOTOR_POLL_INTERVAL_MS);
}
