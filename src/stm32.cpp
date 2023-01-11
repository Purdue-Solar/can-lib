/**
 * @file stm32f.c
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief STM32 CAN implementation file
 * @version 0.1
 * @date 2022-09-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#if defined(BOARD_STM32)

#include "can_lib.h"

namespace PSR
{

CANBus::CANBus(CANBus::Interface& interface, const CANBus::Config& config)
	: _interface(interface), _config(config), _rxCallback(nullptr)
{
}

void CANBus::Init()
{
	CAN_FilterTypeDef filter;

	// Configure filter ranges
	filter.FilterMaskIdLow  = (uint16_t)this->_config.FilterMask;
	filter.FilterMaskIdHigh = (uint16_t)(this->_config.FilterMask >> 16);
	filter.FilterMode       = CAN_FILTERMODE_IDLIST;

	// Configure filter banks
	filter.FilterBank           = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterActivation     = ENABLE;
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;

	// TODO: Fully understand filter setup
	// HAL_CAN_ConfigFilter(this->_interface, &filter);

	this->_interface.Init.AutoRetransmission = this->_config.AutoRetransmit ? ENABLE : DISABLE;
	HAL_CAN_Start(&this->_interface);
}

CANBus::TransmitStatus CANBus::Transmit(const Frame& frame)
{
	CAN_TxHeaderTypeDef txHeader;

	while (HAL_CAN_IsTxMessagePending(&this->_interface, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2))
	{
	}
	HAL_Delay(1);

	txHeader.ExtId = frame.IsExtended ? frame.Id & CANBus::EXT_ID_MASK : 0;
	txHeader.StdId = frame.IsExtended ? 0 : frame.Id & CANBus::STD_ID_MASK;
	txHeader.IDE   = frame.IsExtended ? CAN_ID_EXT : CAN_ID_STD;
	txHeader.DLC   = frame.Length;
	txHeader.RTR   = frame.IsRTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

	uint32_t mailbox;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&this->_interface, &txHeader, (uint8_t*)frame.Data.Bytes, &mailbox);

	switch (status)
	{
	case HAL_OK:
		return CANBus::TransmitStatus::Success;
	case HAL_ERROR:
		return CANBus::TransmitStatus::Error;
	default:
		return CANBus::TransmitStatus::Unknown;
	}
}

// Callback CAN object storage

struct _callbackStoreStruct
{
	CANBus::Interface* interface;
	CANBus::Callback callback;
};

constexpr int _storageSize = 4;
static _callbackStoreStruct _canRxCallbackCANObjects[_storageSize] = { 0 };

bool CANBus::SetCallback(CANBus::Callback callback)
{
	if (callback != nullptr && this->_config.Mode != CANBus::ReceiveMode::Callback)
	{
		return false;
	}

	this->_rxCallback = callback;

	if (callback != nullptr)
	{
		// Fill first spot in storage with interface and callback.
		int i;
		for (i = 0; i < _storageSize; ++i)
		{
			if (_canRxCallbackCANObjects[i].interface == nullptr)
			{
				_canRxCallbackCANObjects[i] = { &this->_interface, callback };
				break;
			}
		}

		if (i == _storageSize)
		{
			return false;
		}

		// Activate interrupt handler
		HAL_CAN_ActivateNotification(&this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	else
	{
		// Remove all instances of current CAN object from storage
		bool anyNotThis = false;
		for (int i = 0; i < _storageSize; ++i)
		{
			if (_canRxCallbackCANObjects[i].interface == &this->_interface)
			{
				_canRxCallbackCANObjects[i] = { nullptr, nullptr };
			}
			else if (_canRxCallbackCANObjects[i].interface != nullptr)
			{
				anyNotThis = true;
			}
		}

		// Deactivate interrupt handler if there are no other handlers in the store.
		if (!anyNotThis)
			HAL_CAN_DeactivateNotification(&this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	return true;
}

HAL_StatusTypeDef TranslateNextFrame(CAN_HandleTypeDef* hcan, CANBus::Frame& frame)
{
	CAN_RxHeaderTypeDef rxHeader;
	HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, frame.Data.Bytes);

	if (status == HAL_OK)
	{
		bool isExtended  = rxHeader.IDE == CAN_ID_EXT;
		frame.Id         = isExtended ? rxHeader.ExtId : rxHeader.StdId;
		frame.Length     = rxHeader.DLC;
		frame.IsRTR      = rxHeader.RTR == CAN_RTR_REMOTE;
		frame.IsExtended = isExtended;
	}

	return status;
}

bool CANBus::Receive(CANBus::Frame& frame)
{
	return TranslateNextFrame(&this->_interface, frame) == HAL_OK;
}

} // namespace PSR

extern "C"
{
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
	{
		PSR::CANBus::Frame frame;
		PSR::CANBus::Callback rxCallback = nullptr;

		for (int i = 0; i < PSR::_storageSize; ++i)
		{
			if (PSR::_canRxCallbackCANObjects[i].interface == hcan)
			{
				rxCallback = PSR::_canRxCallbackCANObjects[i].callback;
				break;
			}
		}

		if (rxCallback != nullptr)
		{
			// Receive all available frames
			while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
			{
				if (TranslateNextFrame(hcan, frame) == HAL_OK)
				{
					rxCallback(frame);
				}
			}
		}
	}
}

#endif // defined(BOARD_STM32)
