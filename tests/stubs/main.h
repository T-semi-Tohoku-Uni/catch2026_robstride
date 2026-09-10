#ifndef HOST_MAIN_H
#define HOST_MAIN_H

#include <stdint.h>

typedef struct { uint32_t unused; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier; } FDCAN_TxHeaderTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *data);
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;

#define limit_GPIO_Port ((void *)0)
#define limit_Pin 1U
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);

#endif
