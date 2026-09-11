#ifndef HOST_MAIN_H
#define HOST_MAIN_H

#include <stdint.h>

typedef struct { uint32_t unused; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier; } FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t unused; } FDCAN_FilterTypeDef;
typedef struct { uint32_t unused; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;

#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_EXTENDED_ID 1U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_8 8U
#define limit_sw_0_GPIO_Port ((GPIO_TypeDef *)0)
#define limit_sw_1_GPIO_Port ((GPIO_TypeDef *)0)
#define limit_sw_0_Pin 1U
#define limit_sw_1_Pin 2U

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *data);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *can, uint32_t fifo);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data);
void Error_Handler(void);

#define limit_GPIO_Port ((void *)0)
#define limit_Pin 1U
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);

#endif
