#include "can_task.h"

int datacheck = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	datacheck = 1;
	CAN_read(hcan, 0, RxData0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	datacheck = 2;
	CAN_read(hcan, 1, RxData1);
}

void CAN_write(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t length, uint8_t TxData[])
{
	TxHeader.IDE = CAN_ID_STD;	// using CAN standard ID
	TxHeader.StdId = std_id;
	TxHeader.RTR = CAN_RTR_DATA;	// indicates we're sending data frame
	TxHeader.DLC = length;	// length of data in bytes

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_error_handler();
	}
}

void CAN_read(CAN_HandleTypeDef *hcan, uint32_t RxFIFO, uint8_t RxData[])
{
	if (datacheck == 1 || datacheck == 2)
	{
		if (HAL_CAN_GetRxMessage(hcan, RxFIFO, &RxHeader, RxData) != HAL_OK)
		{
			CAN_error_handler();
		}
		datacheck = 0;
		rxfifo = RxFIFO;
	}
}

void CAN_error_handler(void)
{
	__disable_irq();
	while(1)
	{
	}
}
