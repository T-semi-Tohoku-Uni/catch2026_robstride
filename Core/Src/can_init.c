#include "can_init.h"
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan1;

void motor_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_EXTENDED_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_MASK;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  Hfdcan_Filter_Settings->FilterID1 = 0x00;
  Hfdcan_Filter_Settings->FilterID2 = 0x0000000;
}

void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
	Htxheader->IdType = FDCAN_EXTENDED_ID;
	Htxheader->TxFrameType = FDCAN_DATA_FRAME;
	Htxheader->DataLength = FDCAN_DLC_BYTES_8;
	Htxheader->FDFormat = FDCAN_CLASSIC_CAN;
	Htxheader->BitRateSwitch = FDCAN_BRS_OFF;
	Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	Htxheader->MessageMarker = 0;
}

HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  FDCAN_FilterTypeDef FDCAN_Filter_settings_robstride;

  motor_CAN_filter_init(&FDCAN_Filter_settings_robstride);
  motor_CAN_txheader_init(Htxheader);

  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings_robstride))
	{
		printf("config filter is error\r\n");
		return HAL_ERROR;
	}

  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
		printf("config global filter is error\r\n");
    return HAL_ERROR;
	}
	
  if (HAL_OK != HAL_FDCAN_Start(&hfdcan1))
	{
		printf("fdcan start is error\r\n");
		return HAL_ERROR;
	}

  if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0))
	{
		printf("fdcan active notification is error\r\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}
