#ifndef CANRPC_STM32_H
#define CANRPC_STM32_H

#include "main.h"
#include "canrpc.h"

/* Board transport: FD with BRS disabled, native motor CAN stays unchanged. */
static uint8_t rpc_node, rpc_boot[16], rpc_boot_seq, rpc_state;
static uint32_t rpc_last_announce;
static volatile uint8_t rpc_announce_due;
static FDCAN_HandleTypeDef *rpc_can;
static uint8_t rpc_peer_boot[16], rpc_old_peer_boot[8][16], rpc_peer_known, rpc_peer_state;
static uint8_t rpc_old_peer_count, rpc_old_peer_next;
static uint32_t rpc_peer_uptime, rpc_peer_seen;

static inline int rpc_peer_ready(void) {
  return rpc_peer_known && rpc_peer_state == CANRPC_READY &&
         (uint32_t)(HAL_GetTick() - rpc_peer_seen) < 750U;
}

static int rpc_entropy(void) {
  uint32_t start = HAL_GetTick();
  __HAL_RCC_HSI48_ENABLE();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET)
    if ((uint32_t)(HAL_GetTick() - start) > 100U) return 0;
  __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);
  (void)RCC->AHB2ENR;
  SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_RNGRST);
  CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_RNGRST);
  RNG->CR = RNG_CR_RNGEN;
  uint32_t previous = 0U, nonzero = 0U;
  for (unsigned i = 0; i < 5U; ++i) {
    start = HAL_GetTick();
    while ((RNG->SR & RNG_SR_DRDY) == 0U) {
      if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0U ||
          (uint32_t)(HAL_GetTick() - start) > 100U) return 0;
    }
    if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0U) return 0;
    uint32_t word = RNG->DR;
    if (i != 0U) {
      if (word == previous) return 0;
      canrpc_put_u32(rpc_boot + (i - 1U) * 4U, word);
      nonzero |= word;
    }
    previous = word;
  }
  RNG->CR = 0U;
  return nonzero != 0U;
}

static int rpc_init(FDCAN_HandleTypeDef *can, uint8_t node) {
  rpc_can = can; rpc_node = node; rpc_state = 0U;
  rpc_announce_due = 1U;
  return rpc_entropy();
}

static int rpc_send(uint32_t id, uint32_t dlc, const uint8_t *data) {
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_EXTENDED_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = dlc;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(rpc_can, &header, (uint8_t *)data);
  __set_PRIMASK(mask);
  return status == HAL_OK;
}

static void rpc_poll(void) {
  uint32_t now = HAL_GetTick();
  if (rpc_announce_due || (uint32_t)(now - rpc_last_announce) >= CANRPC_ANNOUNCE_PERIOD_MS) {
    uint8_t data[24];
    canrpc_boot_payload(data, rpc_boot, now, rpc_state);
    if (rpc_send(canrpc_id(6, CANRPC_CONTROL, CANRPC_BROADCAST, CANRPC_ANNOUNCE,
                           rpc_node, rpc_boot_seq), FDCAN_DLC_BYTES_24, data)) {
      rpc_boot_seq = (rpc_boot_seq + 1U) & 31U;
      rpc_last_announce = now; rpc_announce_due = 0U;
    }
  }
}

static void rpc_ready(void) { rpc_state = CANRPC_READY; rpc_announce_due = 1U; }

static int rpc_valid(const FDCAN_RxHeaderTypeDef *header) {
  return header->IdType == FDCAN_EXTENDED_ID && header->FDFormat == FDCAN_FD_CAN &&
         header->RxFrameType == FDCAN_DATA_FRAME && header->Identifier <= 0x1fffffffU &&
         canrpc_valid_source(header->Identifier);
}

static void rpc_management(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data) {
  uint32_t id = header->Identifier;
  if (!rpc_valid(header)) return;
  if (canrpc_class(id) == CANRPC_CONTROL && canrpc_service(id) == CANRPC_ANNOUNCE &&
      canrpc_destination(id) == CANRPC_BROADCAST && canrpc_source(id) == CANRPC_NODE_ROS &&
      header->DataLength == FDCAN_DLC_BYTES_24 && data[20] == 1U && data[21] == 0U &&
      data[22] <= 4U && (data[23] & 0xfeU) == 0U) {
    uint8_t nonzero = 0U;
    for (unsigned i = 0; i < 16U; ++i) nonzero |= data[i];
    if (!nonzero) return;
    uint32_t uptime = canrpc_get_u32(data + 16);
    if (rpc_peer_known && memcmp(rpc_peer_boot, data, 16) == 0) {
      if ((int32_t)(uptime - rpc_peer_uptime) <= 0) return;
    } else {
      for (unsigned i = 0; i < rpc_old_peer_count; ++i)
        if (memcmp(rpc_old_peer_boot[i], data, 16) == 0) return;
      if (rpc_peer_known) {
        memcpy(rpc_old_peer_boot[rpc_old_peer_next], rpc_peer_boot, 16);
        rpc_old_peer_next = (rpc_old_peer_next + 1U) % 8U;
        if (rpc_old_peer_count < 8U) ++rpc_old_peer_count;
      }
      memcpy(rpc_peer_boot, data, 16); rpc_peer_known = 1U;
    }
    rpc_peer_uptime = uptime; rpc_peer_state = data[22]; rpc_peer_seen = HAL_GetTick();
    return;
  }
  if (canrpc_destination(id) != rpc_node) return;
  uint8_t source = canrpc_source(id), tag = canrpc_tag(id), opcode = canrpc_service(id);
  uint8_t unused = 0U;
  if (canrpc_class(id) == CANRPC_REQUEST) {
    rpc_send(canrpc_id(4, CANRPC_CONTROL, source, CANRPC_REJECT_SERVICE, rpc_node, tag),
             FDCAN_DLC_BYTES_0, &unused);
  } else if (canrpc_class(id) == CANRPC_CONTROL) {
    if (opcode == CANRPC_ANNOUNCE_QUERY && tag == 0U && header->DataLength == FDCAN_DLC_BYTES_0)
      rpc_announce_due = 1U;
    else if (opcode == 0x12U && header->DataLength == FDCAN_DLC_BYTES_48) {
      uint8_t reply[48] = {0};
      memcpy(reply, data, 16); memcpy(reply + 16, rpc_boot, 16);
      memcpy(reply + 32, data + 32, 8); canrpc_put_u32(reply + 40, HAL_GetTick());
      reply[44] = 0x12U; reply[45] = data[44];
      rpc_send(canrpc_id(4, CANRPC_CONTROL, source, 0x13U, rpc_node, tag), FDCAN_DLC_BYTES_48, reply);
    }
  }
}
#endif
