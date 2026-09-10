#ifndef ROBSTRIDE_STARTUP_TEST_HAL_H
#define ROBSTRIDE_STARTUP_TEST_HAL_H

#include <stdint.h>

#define __MAIN_H
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier; } FDCAN_TxHeaderTypeDef;

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    FDCAN_TxHeaderTypeDef *header, uint8_t *data);

#endif
