#ifndef APP_HOST_MAIN_H
#define APP_HOST_MAIN_H

#include <stdint.h>

typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t unused; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } FDCAN_FilterTypeDef;
typedef struct { uint32_t Identifier, DataLength; } FDCAN_TxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, DataLength, IdType, RxFrameType;
} FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t BusOff, LastErrorCode; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;

#define FDCAN1 ((void *)1)
#define FDCAN3 ((void *)3)
#define FDCAN_EXTENDED_ID 0x40000000U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_DLC_BYTES_16 10U
#define FDCAN_RX_FIFO0 0x40U
#define FDCAN_RX_FIFO1 0x41U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 16U

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void NVIC_SystemReset(void);
void Error_Handler(void);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *);

#endif
