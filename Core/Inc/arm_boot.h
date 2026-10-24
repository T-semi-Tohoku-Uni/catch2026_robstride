#ifndef ARM_BOOT_H
#define ARM_BOOT_H

/* Explicit one-attempt startup; readiness queries never start homing. */
static uint8_t arm_initialization_requested;
static uint32_t arm_initialization_started_ms;
static uint8_t arm_boot_step,arm_boot_waiting,arm_boot_failed,arm_boot_complete;
static uint8_t arm_home_reached,arm_native_confirmed,arm_fault_stop_queued;
static uint32_t arm_boot_started,arm_boot_delay_until,arm_home_started,arm_feedback_started;
static uint32_t arm_feedback_baseline[3];
static GPIO_PinState arm_home_initial;

static uint32_t arm_failure_flags;
static uint8_t arm_failure_reason,arm_failure_axis=255U,arm_failure_stage;
static uint8_t arm_axis_command[3],arm_axis_response[3],arm_fault_snapshot[36];
static uint8_t arm_step_tx_confirmed;
static uint32_t arm_step_response_baseline;
static uint8_t arm_step_axis(void) {
  if((arm_boot_step>=10U && arm_boot_step<16U) || arm_boot_step==26U) return 2U;
  if((arm_boot_step>=16U && arm_boot_step<22U) || arm_boot_step==27U) return 1U;
  return 0U;
}
static uint8_t arm_step_command(void) {
  switch(arm_boot_step) {
    case 0:case 3:case 8:case 22:return 1U;
    case 1:return 2U;
    case 2:case 5:case 25:return 3U;
    case 4:return 4U;
    case 6:return 5U;
    case 9:return 6U;
    case 23:return 11U;
    case 24:return 12U;
    case 26:case 27:return 13U;
    default: {
      unsigned action=(arm_boot_step-10U)%6U;
      switch(action) {case 0:return 1U;case 1:return 7U;case 2:return 3U;case 4:return 8U;case 5:return 9U;default:return 10U;}
    }
  }
}
static uint8_t arm_step_expected_mode(void) {
  uint8_t command=arm_step_command();
  return command==1U || command==2U || command==4U || command==6U || command==7U || command==11U || command==12U ? 0U:2U;
}
static uint8_t arm_axis_mode(unsigned axis) {
  return axis==0U ? cybergear_base.feedback.mode : robstride_handler[axis==1U ? LEFT_RS03_INDEX:RIGHT_RS03_INDEX].feedback.mode;
}
static uint8_t arm_axis_fault(unsigned axis) {
  return axis==0U ? cybergear_base.feedback.fault_flags : robstride_handler[axis==1U ? LEFT_RS03_INDEX:RIGHT_RS03_INDEX].feedback.fault_flags;
}
static uint32_t arm_axis_age(unsigned axis,uint32_t now) {
  if(axis_rx_count[axis]==0U) return UINT32_MAX;
  uint32_t received=axis==0U ? cybergear_base.feedback.last_received_ms : robstride_handler[axis==1U ? LEFT_RS03_INDEX:RIGHT_RS03_INDEX].feedback.last_leceived_ms;
  return now-received;
}
static void arm_axis_record(uint8_t *out,unsigned axis,uint32_t now) {
  uint8_t seen=axis_rx_count[axis]!=0U;
  uint32_t age=arm_axis_age(axis,now);
  uint8_t online=axis==0U ? cybergear_base.feedback.online:robstride_handler[axis==1U ? LEFT_RS03_INDEX:RIGHT_RS03_INDEX].feedback.online;
  memset(out,0,12);
  out[0]=seen ? arm_axis_mode(axis):255U;out[1]=seen ? arm_axis_fault(axis):255U;
  out[2]=(seen?1U:0U) | (seen && online && age<CANRPC_PROJECT_MOTOR_FEEDBACK_MAX_AGE_MS ? 2U:0U) |
    (seen && arm_axis_fault(axis)==0U ? 4U:0U) | (arm_axis_response[axis]==4U ? 8U:0U);
  out[3]=axis==0U ? CYBER_GEAR_ID : (axis==1U ? LEFT_RS03_ID:RIGHT_RS03_ID);
  canrpc_put_u32(out+4,age);out[8]=arm_axis_command[axis];out[9]=arm_axis_response[axis];
}

static int arm_feedback_fresh(void) {
  uint32_t now=HAL_GetTick();
  return cybergear_base.feedback.online &&
    robstride_handler[0].feedback.online && robstride_handler[1].feedback.online &&
    (uint32_t)(now-cybergear_base.feedback.last_received_ms)<CANRPC_PROJECT_MOTOR_FEEDBACK_MAX_AGE_MS &&
    (uint32_t)(now-robstride_handler[0].feedback.last_leceived_ms)<CANRPC_PROJECT_MOTOR_FEEDBACK_MAX_AGE_MS &&
    (uint32_t)(now-robstride_handler[1].feedback.last_leceived_ms)<CANRPC_PROJECT_MOTOR_FEEDBACK_MAX_AGE_MS;
}
static int arm_feedback_operating(void) {
  return cybergear_base.feedback.mode==2U && cybergear_base.feedback.fault_flags==0U &&
    robstride_handler[0].feedback.mode==2U && robstride_handler[0].feedback.fault_flags==0U &&
    robstride_handler[1].feedback.mode==2U && robstride_handler[1].feedback.fault_flags==0U;
}
static uint32_t arm_ready_flags(void) {
  return (arm_boot_complete ? 1U:0U) | (arm_feedback_fresh()?2U:0U) |
    (arm_feedback_operating()?4U:0U) | (arm_native_confirmed?8U:0U) | (arm_home_reached?16U:0U);
}
static int arm_rpc(uint8_t service,const uint8_t *data,uint8_t length,uint8_t *reply,uint8_t *reply_length) {
  (void)data;
  if(service==CANRPC_PROJECT_ARM_TARGET_SERVICE) return arm_target_rpc(service,data,length,reply,reply_length);
  if ((service != CANRPC_PROJECT_ARM_READY_SERVICE &&
       service != CANRPC_PROJECT_ARM_INITIALIZE_SERVICE) || length != 0U) return 0;
  if (service == CANRPC_PROJECT_ARM_INITIALIZE_SERVICE && !arm_initialization_requested &&
      !rpc_reset_active && rpc_state != CANRPC_RESET_PENDING) {
    arm_initialization_requested = 1U;
    arm_initialization_started_ms = HAL_GetTick();
    arm_boot_started = arm_boot_delay_until = arm_initialization_started_ms;
  }
  uint32_t flags=arm_boot_failed ? arm_failure_flags:arm_ready_flags();
  memset(reply,0,64);
  canrpc_put_u32(reply,arm_boot_failed || rpc_state==CANRPC_FAULT ? 2U : (flags==31U ? 0U:1U));
  canrpc_put_u32(reply+4,flags);
  reply[8]=!arm_initialization_requested ? 31U : (arm_boot_failed ? arm_failure_stage:(arm_boot_complete?29U:arm_boot_step));
  reply[9]=arm_failure_reason;reply[10]=arm_failure_axis;reply[11]=2U;
  if(arm_boot_failed) memcpy(reply+12,arm_fault_snapshot,36);
  else for(unsigned i=0;i<3U;++i) arm_axis_record(reply+12+12U*i,i,HAL_GetTick());
  memcpy(reply+48,rpc_boot,16U);
  *reply_length=64U;return 1;
}
static void arm_boot_fault(uint8_t reason,uint8_t axis) {
  if(arm_boot_failed) return;
  arm_failure_flags=arm_ready_flags();
  arm_failure_reason=reason;arm_failure_axis=axis;arm_failure_stage=arm_boot_complete ? 30U:arm_boot_step;
  if(axis<3U && reason<=5U) arm_axis_response[axis]=5U;
  for(unsigned i=0;i<3U;++i) {
    arm_axis_record(arm_fault_snapshot+12U*i,i,HAL_GetTick());
    arm_fault_snapshot[12U*i+2U]|=0x80U;
  }
  arm_boot_failed=1U;motors_ready=false;
  rpc_state=CANRPC_FAULT;rpc_announce_due=1U;
  /* Snapshot precedes cleanup; a late STOP must not erase the failure evidence. */
  arm_boot_started=HAL_GetTick();arm_fault_stop_queued=0U;
}

static int arm_boot_tx_complete(uint8_t marker) {
  int found=0;
  while((hfdcan3.Instance->TXEFS & FDCAN_TXEFS_EFFL)!=0U) {
    FDCAN_TxEventFifoTypeDef event;
    if(HAL_FDCAN_GetTxEvent(&hfdcan3,&event)!=HAL_OK) break;
    if(event.MessageMarker==marker &&
       (event.EventType==FDCAN_TX_EVENT || event.EventType==FDCAN_TX_IN_SPITE_OF_ABORT)) found=1;
  }
  return found;
}
static void arm_boot_initialize(void) {
  cybergear_init(&cybergear_base,&hfdcan3,CYBER_GEAR_ID,HOST_ID);
  for(unsigned i=0;i<2U;++i) {
    robstride_handler[i].host_id=HOST_ID;
    robstride_handler[i].motor_id=i==RIGHT_RS03_INDEX ? RIGHT_RS03_ID:LEFT_RS03_ID;
    robstride_handler[i].run_mode=POSITION_PP;
    robstride_handler[i].txheader=motor_txheader;
  }
}
static int arm_boot_send_step(void) {
  uint8_t marker=(uint8_t)(0x20U+arm_boot_step);
  cybergear_base.tx_header.TxEventFifoControl=FDCAN_STORE_TX_EVENTS;
  cybergear_base.tx_header.MessageMarker=marker;
  for(unsigned i=0;i<2U;++i) {
    robstride_handler[i].txheader.TxEventFifoControl=FDCAN_STORE_TX_EVENTS;
    robstride_handler[i].txheader.MessageMarker=marker;
  }
  if(arm_boot_step==25U) arm_feedback_baseline[0]=axis_rx_count[0];
  if(arm_boot_step==26U) arm_feedback_baseline[2]=axis_rx_count[2];
  if(arm_boot_step==27U) arm_feedback_baseline[1]=axis_rx_count[1];
  switch(arm_boot_step) {
    case 0:return cybergear_stop(&cybergear_base);
    case 1:return cybergear_set_run_mode(&cybergear_base,CYBERGEAR_RUN_MODE_OPERATION);
    case 2:return cybergear_enable(&cybergear_base);
    case 3:return cybergear_stop(&cybergear_base);
    case 4:return cybergear_set_run_mode(&cybergear_base,CYBERGEAR_RUN_MODE_SPEED);
    case 5:return cybergear_enable(&cybergear_base);
    case 6:
      arm_home_initial=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
      return cybergear_set_velocity(&cybergear_base,1.0f);
    case 8:return cybergear_stop(&cybergear_base);
    case 9:return cybergear_set_zero(&cybergear_base);
    case 22:return cybergear_stop(&cybergear_base);
    case 23:return cybergear_set_run_mode(&cybergear_base,CYBERGEAR_RUN_MODE_CURRENT);
    case 24:return cybergear_set_current(&cybergear_base,0.0f);
    case 25:
      memset(&cybergear_base.adrc,0,sizeof(cybergear_base.adrc));
      return cybergear_enable(&cybergear_base);
    case 26:return robstride_set_current_limit(&robstride_handler[RIGHT_RS03_INDEX],10.0f);
    case 27:return robstride_set_current_limit(&robstride_handler[LEFT_RS03_INDEX],10.0f);
    default: {
      unsigned index=arm_boot_step<16U ? 0U:1U;
      unsigned action=(arm_boot_step-10U)%6U;
      RobstrideMotor *motor=&robstride_handler[index];
      switch(action) {
        case 0:return robstride_stop(motor);
        case 1:return robstride_set_run_mode(motor,POSITION_PP);
        case 2:return robstride_enable(motor);
        case 4:return robstride_set_pp_velocity_max(motor,10.0f);
        case 5:return robstride_set_pp_acceleration(motor,1.0f);
        default:return robstride_set_current_limit(motor,10.0f);
      }
    }
  }
}
static void arm_boot_poll(void) {
  if (!arm_initialization_requested || rpc_reset_active || rpc_state == CANRPC_RESET_PENDING) return;
  uint32_t now=HAL_GetTick();
  if(arm_boot_complete) return;
  if(arm_boot_failed) {
    if(!arm_fault_stop_queued && (uint32_t)(now-arm_boot_started)<CANRPC_PROJECT_MOTOR_STEP_TIMEOUT_MS)
      arm_fault_stop_queued=cybergear_request_stop_event(&cybergear_base,0x7eU);
    (void)arm_boot_tx_complete(0x7eU);
    return;
  }
  if ((uint32_t)(now - arm_initialization_started_ms) >= CANRPC_PROJECT_MOTOR_INITIALIZATION_TIMEOUT_MS) {
    arm_boot_fault(9U,255U);return;
  }
  if(arm_boot_step==7U) {
    if((uint32_t)(now-arm_home_started)>=CANRPC_PROJECT_MOTOR_HOMING_TIMEOUT_MS) {arm_boot_fault(6U,0U);return;}
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==arm_home_initial) return;
    arm_home_reached=1U;arm_boot_step=8U;arm_boot_started=now;return;
  }
  if(arm_boot_step==28U) {
    if((uint32_t)(now-arm_feedback_started)>=CANRPC_PROJECT_MOTOR_FEEDBACK_TIMEOUT_MS) {arm_boot_fault(7U,255U);return;}
    if(!arm_feedback_fresh() || !arm_feedback_operating()) return;
    for(unsigned i=0;i<3U;++i) if(axis_rx_count[i]==arm_feedback_baseline[i]) return;
    cybergear_base.tx_header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    for(unsigned i=0;i<2U;++i) robstride_handler[i].txheader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    cybergear_base.adrc.last_update_ms=now;cybergear_base.adrc.active=true;
    arm_boot_complete=1U;arm_control_enable();motors_ready=true;rpc_ready();return;
  }
  if((int32_t)(now-arm_boot_delay_until)<0) return;
  uint8_t axis=arm_step_axis();
  if((uint32_t)(now-arm_boot_started)>=CANRPC_PROJECT_MOTOR_STEP_TIMEOUT_MS) {
    uint8_t reason=!arm_boot_waiting ? 1U : (!arm_step_tx_confirmed ? 2U :
      (axis_rx_count[axis]!=arm_step_response_baseline && arm_axis_mode(axis)!=arm_step_expected_mode() ? 5U:3U));
    arm_boot_fault(reason,axis);return;
  }
  if(!arm_boot_waiting) {
    arm_axis_command[axis]=arm_step_command();arm_axis_response[axis]=1U;
    arm_step_response_baseline=axis_rx_count[axis];
    if(arm_boot_send_step()) {
      arm_boot_waiting=1U;arm_step_tx_confirmed=0U;arm_axis_response[axis]=2U;
    }
    return;
  }
  if(!arm_step_tx_confirmed && arm_boot_tx_complete((uint8_t)(0x20U+arm_boot_step))) {
    arm_step_tx_confirmed=1U;arm_axis_response[axis]=3U;
  }
  if(axis_rx_count[axis]!=arm_step_response_baseline && arm_axis_fault(axis)!=0U) {
    arm_boot_fault(4U,axis);return;
  }
  if(!arm_step_tx_confirmed || axis_rx_count[axis]==arm_step_response_baseline ||
     arm_axis_age(axis,now)>=CANRPC_PROJECT_MOTOR_FEEDBACK_MAX_AGE_MS ||
     arm_axis_mode(axis)!=arm_step_expected_mode()) return;
  arm_axis_response[axis]=4U;
  arm_boot_waiting=0U;
  if(arm_boot_step==6U) arm_home_started=now;
  if(arm_boot_step==27U) {
    arm_native_confirmed=1U;arm_feedback_started=now;
  }
  arm_boot_delay_until=now+(arm_boot_step==8U ? CANRPC_PROJECT_MOTOR_HOME_SETTLE_MS:CANRPC_PROJECT_MOTOR_INTER_COMMAND_MS);
  ++arm_boot_step;
  arm_boot_started=arm_boot_delay_until;
}
#endif
