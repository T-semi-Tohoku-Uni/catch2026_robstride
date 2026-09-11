#ifndef CYBERGEAR_TEST_STM32G4XX_HAL_H
#define CYBERGEAR_TEST_STM32G4XX_HAL_H

/* Host-only HAL surface. These declarations never enter the firmware build. */
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef struct { void *Instance; uint32_t ErrorCode; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength;
    uint32_t ErrorStateIndicator, BitRateSwitch, FDFormat;
    uint32_t TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength;
    uint32_t ErrorStateIndicator, BitRateSwitch, FDFormat;
    uint32_t RxTimestamp, FilterIndex, IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t LastErrorCode, DataLastErrorCode, Activity;
    uint32_t ErrorPassive, Warning, BusOff;
    uint32_t RxESIflag, RxBRSflag, RxFDFflag, ProtocolException;
    uint32_t TDCvalue, RestrictedOperationMode;
} FDCAN_ProtocolStatusTypeDef;
typedef struct {
    uint32_t TxErrorCnt, RxErrorCnt, RxErrorPassive, ErrorLogging;
} FDCAN_ErrorCountersTypeDef;

#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 0x40000000U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_REMOTE_FRAME 0x20000000U
#define FDCAN_DLC_BYTES_7 0x00070000U
#define FDCAN_DLC_BYTES_8 0x00080000U
#define FDCAN_DLC_BYTES_12 0x00090000U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define HAL_FDCAN_ERROR_NONE 0U
#define HAL_FDCAN_ERROR_BUS_OFF 0x00000080U

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *header, uint8_t *data);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    const FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *status);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(
    const FDCAN_HandleTypeDef *hfdcan, FDCAN_ErrorCountersTypeDef *counters);
uint32_t HAL_FDCAN_GetError(const FDCAN_HandleTypeDef *hfdcan);

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
#ifdef CYBERGEAR_HOST_IRQ_HOOK
void cybergear_test_restore_irq(uint32_t mask);
static inline void __set_PRIMASK(uint32_t mask) { cybergear_test_restore_irq(mask); }
#else
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
static inline void __DMB(void) {}

#endif
