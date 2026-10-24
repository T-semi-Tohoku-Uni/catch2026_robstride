#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

/* One native request per axis; TX events and status are different observations. */
typedef struct {
  uint8_t pending,tx_confirmed,failed,current_valid;
  float queued_current;
  uint32_t started,baseline,target_token,identifier;
} ArmNativeRequest;
static ArmNativeRequest arm_native_request[3];
static uint32_t arm_event_id[256];
static uint8_t arm_event_valid[256];
static uint8_t arm_control_enabled,arm_control_latched,arm_control_failure,arm_control_failure_axis;
static uint8_t arm_target_active,arm_target_slot,arm_target_confirmed;
static uint32_t arm_target_token,arm_target_started,arm_stop_started;
static float arm_target_values[3];

static bool arm_native_take_event(uint8_t marker,uint32_t identifier) {
  uint32_t mask=__get_PRIMASK();__disable_irq();
  while((hfdcan3.Instance->TXEFS & FDCAN_TXEFS_EFFL)!=0U) {
    FDCAN_TxEventFifoTypeDef event;
    if(HAL_FDCAN_GetTxEvent(&hfdcan3,&event)!=HAL_OK) break;
    if(event.EventType==FDCAN_TX_EVENT || event.EventType==FDCAN_TX_IN_SPITE_OF_ABORT) {
      arm_event_id[event.MessageMarker]=event.Identifier;arm_event_valid[event.MessageMarker]=1U;
    }
  }
  bool found=arm_event_valid[marker] && (!identifier || arm_event_id[marker]==identifier);
  if(found) arm_event_valid[marker]=0U;
  __set_PRIMASK(mask);return found;
}
static int arm_native_axis(uint8_t motor_id) {
  return motor_id==CYBER_GEAR_ID ? 0 : (motor_id==LEFT_RS03_ID ? 1 : (motor_id==RIGHT_RS03_ID ? 2:-1));
}
static bool arm_native_busy(uint8_t motor_id) {
  int axis=arm_native_axis(motor_id);
  return (rpc_reset_active && reset_stop_confirmed) || arm_control_latched || arm_control_failure || (axis>=0 && arm_native_request[axis].pending);
}
static bool arm_native_send(uint8_t motor_id,FDCAN_TxHeaderTypeDef *header,uint8_t *data) {
  int axis=arm_native_axis(motor_id);
  if(axis==0 && ((header->Identifier>>24)&31U)==CYBERGEAR_COMM_STOP) {
    if(!rpc_reset_active && !arm_control_latched && !arm_control_failure) {
      arm_control_failure=8U;arm_control_failure_axis=0U;
    }
    return false;
  }
  if(!arm_boot_complete || axis<0 || arm_native_busy(motor_id)) return false;
  uint32_t mask=__get_PRIMASK();__disable_irq();
  ArmNativeRequest *request=&arm_native_request[axis];
  uint8_t marker=(uint8_t)(0x80U+(unsigned)axis);
  (void)arm_native_take_event(marker,0U);
  FDCAN_TxHeaderTypeDef local=*header;local.TxEventFifoControl=FDCAN_STORE_TX_EVENTS;local.MessageMarker=marker;
  request->baseline=axis_rx_count[axis];request->started=HAL_GetTick();request->identifier=header->Identifier;
  request->target_token=arm_target_active ? arm_target_token:0U;
  request->current_valid=axis==0 && ((header->Identifier>>24)&31U)==CYBERGEAR_COMM_WRITE_PARAMETER &&
    ((uint16_t)data[0]|((uint16_t)data[1]<<8))==CYBERGEAR_PARAM_IQ_REF;
  if(request->current_valid) request->queued_current=canrpc_get_float(data+4);
  if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3,&local,data)!=HAL_OK) {
    arm_control_failure=1U;arm_control_failure_axis=(uint8_t)axis;request->failed=1U;
    __set_PRIMASK(mask);return false;
  }
  request->pending=1U;request->tx_confirmed=0U;
  arm_axis_command[axis]=axis==0 ? 15U:14U;arm_axis_response[axis]=2U;
  __set_PRIMASK(mask);return true;
}
static void arm_native_poll(void) {
  if(!arm_control_enabled) return;
  if(rpc_reset_active) {
    arm_reset_monitor();
    if(reset_stop_confirmed) {
      for(unsigned i=0;i<3U;++i) arm_native_request[i].pending=0U;
      return;
    }
  }
  uint32_t now=HAL_GetTick();
  for(unsigned i=0;i<3U;++i) {
    ArmNativeRequest *request=&arm_native_request[i];
    if(!request->pending || arm_control_latched || arm_control_failure) continue;
    if((uint32_t)(now-request->started)>=CANRPC_PROJECT_NATIVE_ACK_TIMEOUT_MS) {
      arm_control_failure=!request->tx_confirmed ? 2U :
        (axis_rx_count[i]!=request->baseline && arm_axis_mode(i)!=2U ? 5U:3U);
      arm_control_failure_axis=(uint8_t)i;request->failed=1U;continue;
    }
    if(!request->tx_confirmed && arm_native_take_event((uint8_t)(0x80U+i),request->identifier)) {
      request->tx_confirmed=1U;arm_axis_response[i]=3U;
      if(i==0U && request->current_valid) cybergear_base.adrc.current_a=request->queued_current;
    }
    if(axis_rx_count[i]!=request->baseline && arm_axis_fault(i)!=0U) {
      arm_control_failure=4U;arm_control_failure_axis=(uint8_t)i;request->failed=1U;continue;
    }
    if(request->tx_confirmed && axis_rx_count[i]!=request->baseline && arm_axis_mode(i)==2U &&
       arm_axis_age(i,now)<CANRPC_PROJECT_MOTOR_FEEDBACK_MAX_AGE_MS) {
      request->pending=0U;arm_axis_response[i]=4U;
      if(arm_target_active && request->target_token==arm_target_token) arm_target_confirmed|=(uint8_t)(1U<<i);
    }
  }
}
static void arm_control_enable(void) {
  uint32_t mask=__get_PRIMASK();__disable_irq();
  cybergear_base.request_send=arm_native_send;cybergear_base.request_busy=arm_native_busy;
  cybergear_base.request_event=arm_native_take_event;
  for(unsigned i=0;i<2U;++i) robstride_handler[i].request_send=arm_native_send;
  arm_control_enabled=1U;__set_PRIMASK(mask);
}
static int arm_target_rpc(uint8_t service,const uint8_t *data,uint8_t length,uint8_t *reply,uint8_t *reply_length) {
  if(service!=CANRPC_PROJECT_ARM_TARGET_SERVICE || length!=12U) return 0;
  uint32_t result=0U;
  for(unsigned i=0;i<3U;++i) if(!isfinite(canrpc_get_float(data+4U*i))) result=1U;
  if(!arm_boot_complete || arm_control_latched || rpc_state!=CANRPC_READY) result=3U;
  if(!result) return 0;
  canrpc_put_u32(reply,result);memcpy(reply+4,data,12);canrpc_put_u32(reply+16,0U);*reply_length=20U;return 1;
}
static int arm_control_start(uint8_t service,const uint8_t *data,uint8_t length,uint8_t slot,uint32_t token) {
  if(service!=CANRPC_PROJECT_ARM_TARGET_SERVICE || length!=12U || !arm_boot_complete || arm_target_active || arm_control_latched || rpc_state!=CANRPC_READY) return 0;
  for(unsigned i=0;i<3U;++i) if(!isfinite(canrpc_get_float(data+4U*i))) return 0;
  arm_target_active=1U;arm_target_slot=slot;arm_target_token=token;arm_target_confirmed=0U;arm_target_started=HAL_GetTick();
  for(unsigned i=0;i<3U;++i) arm_target_values[i]=target_angle[i]=canrpc_get_float(data+4U*i);
  return 1;
}
static void arm_control_fail(uint8_t reason,uint8_t axis) {
  if(arm_control_latched) return;
  arm_control_latched=1U;arm_stop_started=HAL_GetTick();
  arm_boot_fault(reason,axis);
  /* Use the bounded STOP handshake; failed native ACKs do not guarantee position holding. */
  motors_ready=true;cybergear_base.preserve_adrc_on_tx_failure=true;
  (void)arm_reset_prepare();
}
static void arm_control_poll(void) {
  if (!arm_control_enabled) return;
  uint32_t mask=__get_PRIMASK();__disable_irq();
  arm_native_poll();
  if(rpc_reset_active) {
    if(arm_target_active) {
      canrpc_rpc_cache_entry_t *entry=&rpc_server.entries[arm_target_slot];
      if(entry->token==arm_target_token && entry->state==4U) entry->state=3U;
      arm_target_active=0U;
    }
    __set_PRIMASK(mask);return;
  }
  if(arm_control_failure && !arm_control_latched) arm_control_fail(arm_control_failure,arm_control_failure_axis);
  if(arm_target_active) {
    if((uint32_t)(HAL_GetTick()-arm_target_started)>=rpc_config.response_timeout_ms && !arm_control_latched)
      arm_control_fail(3U,255U);
    if(arm_control_latched || arm_target_confirmed==7U) {
      uint8_t reply[20];canrpc_put_u32(reply,arm_control_latched ? 3U:0U);
      for(unsigned i=0;i<3U;++i) canrpc_put_float(reply+4U+4U*i,arm_target_values[i]);
      canrpc_put_u32(reply+16,arm_target_confirmed);
      if(!rpc_application_complete(arm_target_slot,arm_target_token,reply,20U) && !arm_control_latched)
        arm_control_fail(3U,255U);
      arm_target_active=0U;
    }
  }
  if(arm_control_latched && !reset_stop_confirmed &&
     (uint32_t)(HAL_GetTick()-arm_stop_started)<CANRPC_PROJECT_RESET_EXECUTE_TIMEOUT_MS) (void)arm_reset_prepare();
  __set_PRIMASK(mask);
}
#endif
