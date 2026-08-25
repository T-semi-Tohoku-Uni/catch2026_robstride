#ifndef __CAN__INT_H
#define __CAN__INT_H

#include "main.h"

void motor_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings);
void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader);

void inter_board_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings);
void inter_board_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef inter_board_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader);

#endif
