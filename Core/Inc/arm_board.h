#ifndef ARM_BOARD_H
#define ARM_BOARD_H

#include "main.h"

/* Reject regenerated settings that change native ACK or control timing. */
static int arm_board_config_valid(const FDCAN_HandleTypeDef *native,
                                  const TIM_HandleTypeDef *timer)
{
  if (HAL_RCC_GetSysClockFreq() != 80000000U ||
      HAL_RCC_GetHCLKFreq() != 80000000U ||
      HAL_RCC_GetPCLK1Freq() != 80000000U ||
      native->Instance != FDCAN3 || timer->Instance != TIM6 ||
      native->Init.ClockDivider != FDCAN_CLOCK_DIV1 ||
      native->Init.FrameFormat != FDCAN_FRAME_FD_BRS ||
      native->Init.Mode != FDCAN_MODE_NORMAL ||
      native->Init.AutoRetransmission != DISABLE ||
      native->Init.NominalPrescaler != 4U ||
      native->Init.NominalSyncJumpWidth != 1U ||
      native->Init.NominalTimeSeg1 != 15U ||
      native->Init.NominalTimeSeg2 != 4U ||
      native->Init.ExtFiltersNbr != 1U ||
      native->Init.TxFifoQueueMode != FDCAN_TX_FIFO_OPERATION ||
      timer->Init.Prescaler != 79U || timer->Init.Period != 999U ||
      timer->Init.CounterMode != TIM_COUNTERMODE_UP ||
      timer->Init.AutoReloadPreload != TIM_AUTORELOAD_PRELOAD_DISABLE) return 0;

  const IRQn_Type required[] = {FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn,
    FDCAN3_IT0_IRQn, FDCAN3_IT1_IRQn, TIM6_DAC_IRQn, EXTI0_IRQn, EXTI1_IRQn};
  for (unsigned i = 0; i < sizeof(required) / sizeof(required[0]); ++i) {
    if (!NVIC_GetEnableIRQ(required[i]) || NVIC_GetPriority(required[i]) != 0U) return 0;
  }
  return 1;
}

#endif
