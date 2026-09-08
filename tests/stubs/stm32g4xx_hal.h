#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { int unused; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,
    BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker; } FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier,IdType,RxFrameType,DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t BusOff; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt,RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef struct { uint32_t CYCCNT; } TestDWT;
extern TestDWT test_dwt;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define FDCAN_EXTENDED_ID 1
#define FDCAN_DATA_FRAME 0
#define FDCAN_DLC_BYTES_8 0x80000
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_NO_TX_EVENTS 0
#define HAL_OK 0
uint32_t HAL_GetTick(void);
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*,FDCAN_TxHeaderTypeDef*,uint8_t*);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef*);
int HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*,FDCAN_ProtocolStatusTypeDef*);
int HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef*,FDCAN_ErrorCountersTypeDef*);
static inline uint32_t __get_PRIMASK(void) {return 0;}
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t x) {(void)x;}
static inline void __DMB(void) {}
#endif
