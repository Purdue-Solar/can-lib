/**
 * @file stm32f0xx.c
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief
 * @version 0.1
 * @date 2022-09-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#if defined(BOARD_STM32F) && BOARD_STM32F == 0

#include "can_lib.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"

void CAN_Init(CAN_Interface* interface, CAN_Config config)
{
	CAN_FilterTypeDef filter;

	// Config filter ranges
	filter.FilterIdLow = config.FilterIdLow;
	filter.FilterIdHigh = config.FilterIdHigh;
	filter.FilterMode = CAN_FILTERMODE_IDLIST;

	// Config filter banks
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;

	interface->Handle->Init.AutoRetransmission = config.AutoRetransmit ? ENABLE : DISABLE;

	HAL_CAN_ConfigFilter(interface->Handle, &filter);
	HAL_CAN_Start(interface->Handle);
}

CAN_TransmitStatus CAN_Transmit(CAN_Interface* interface, CAN_Frame* frame)
{
	CAN_TxHeaderTypeDef txHeader;

	while (HAL_CAN_IsTxMessagePending(interface->Handle, interface->Mailbox)) {}
	HAL_Delay(1);

	txHeader.StdId = frame->Id;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = frame->Length;
	txHeader.RTR = frame->RTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(interface->Handle, &txHeader, frame->Data.Bytes, &interface->Mailbox);
	
	switch (status)
	{
		case HAL_OK:
			return TransmitStatus_Success;
		case HAL_ERROR:
			return TransmitStatus_Error;
		default:
			return TransmitStatus_Unknown;
	}
}

CAN_Callback* _canRxCallback = NULL;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	CAN_Frame frame;

	if (_canRxCallback != NULL)
	{
		// Receive all available frames
		while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
		{
			HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, frame.Data.Bytes);

			if (status == HAL_OK)
			{
				frame.Id = rxHeader.StdId;
				frame.Length = rxHeader.DLC;
				frame.RTR = rxHeader.RTR == CAN_RTR_REMOTE;

				(*_canRxCallback)(&frame);
			}
		}
	}
}

void CAN_SetRxCallback(CAN_Interface* interface, CAN_Callback *callback)
{
	HAL_CAN_ActivateNotification(interface->Handle, CAN_IT_RX_FIFO0_MSG_PENDING);
	_canRxCallback = callback;
}

void CAN_ClearRxCallback(CAN_Interface* interface)
{
	HAL_CAN_DeactivateNotification(interface->Handle, CAN_IT_RX_FIFO0_MSG_PENDING);
	_canRxCallback = NULL;
}

#endif // defined(BOARD_STM32F) && BOARD_STM32F == 3
