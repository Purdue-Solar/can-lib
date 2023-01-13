/**
 * @file stm32.c
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief STM32 CAN implementation file
 * @version 0.8
 *
 * @copyright Copyright (c) 2023
 *
 */

#if defined(BOARD_STM32)

#include "can_lib.h"

namespace PSR
{

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

// CAN callback object storage
//
//	Allows the storage of multiple callbacks, each one related to a
//	single instance of a CANBus object

struct _callbackStoreStruct
{
	CANBus* busObject;
	CANBus::Interface* interface;
	CANBus::Callback callback;
};

static constexpr int _callbackStorageSize = 8;

static struct _callbackStoreStruct _canRxCallbackStore[_callbackStorageSize] = { 0 };

static bool IsCallbackStorageEmpty()
{
	for (int i = 0; i < _callbackStorageSize; ++i)
	{
		if (_canRxCallbackStore[i].interface != nullptr)
		{
			return false;
		}
	}

	return true;
}

bool CANBus::SetCallback(CANBus::Callback callback)
{
	// Do not update if the object is not in callback mode
	if (callback != nullptr && this->_config.Mode != CANBus::ReceiveMode::Callback)
	{
		return false;
	}

	this->_rxCallback = callback;

	if (callback != nullptr)
	{
		// Fill first spot in storage with interface and callback
		int i;
		for (i = 0; i < _callbackStorageSize; ++i)
		{
			if (_canRxCallbackStore[i].interface == nullptr)
			{
				_canRxCallbackStore[i] = { this, &this->_interface, callback };
				break;
			}
		}

		// Callback store is full
		if (i == _callbackStorageSize)
		{
			return false;
		}

		// Activate interrupt handler
		}
	else
	{
		// Remove all instances of current CAN object from storage
		for (int i = 0; i < _callbackStorageSize; ++i)
		{
			struct _callbackStoreStruct obj = _canRxCallbackStore[i];
			if (obj.busObject == this)
			{
				_canRxCallbackStore[i] = { nullptr, nullptr, nullptr };
			}
		}
	}

	if (IsCallbackStorageEmpty())
		HAL_CAN_DeactivateNotification(&this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);
	else
		HAL_CAN_ActivateNotification(&this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);

	return true;
}

/**
 * @brief Try to receive a frame from the interface and update a reference to a frame
 *
 * @param hcan A pointer to the CAN interface
 * @param frame The frame to be updated with the received frame
 * @return bool Whether there was a frame available and it was successfully translated.
 */
static bool TranslateNextFrame(CAN_HandleTypeDef* hcan, CANBus::Frame& frame)
{
	if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0)
	{
		return false;
	}

	CAN_RxHeaderTypeDef rxHeader;
	HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, frame.Data.Bytes);

	// Only modify frame if the message is received properly
	if (status == HAL_OK)
	{
		bool isExtended  = rxHeader.IDE == CAN_ID_EXT;
		frame.Id         = isExtended ? rxHeader.ExtId : rxHeader.StdId;
		frame.Length     = rxHeader.DLC;
		frame.IsRTR      = rxHeader.RTR == CAN_RTR_REMOTE;
		frame.IsExtended = isExtended;

		return true;
	}

	return false;
}

bool CANBus::Receive(CANBus::Frame& frame)
{
	return TranslateNextFrame(&this->_interface, frame);
}

} // namespace PSR

extern "C"
{
	// General CAN callback defined by HAL library
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
	{
		PSR::CANBus::Frame frame;

		PSR::CANBus::Callback callbacks[PSR::_callbackStorageSize];
		int callbackCount = 0;

		// Scan callback storage for all instances of this interface.
		for (int i = 0; i < PSR::_callbackStorageSize; ++i)
		{
			if (PSR::_canRxCallbackStore[i].interface == hcan)
			{
				callbacks[callbackCount++] = PSR::_canRxCallbackStore[i].callback;
			}
		}

		if (callbackCount > 0)
		{
			// Receive all available frames
			while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
			{
				if (TranslateNextFrame(hcan, frame))
				{
					// Execute all callbacks that were found for each frame.
					for (int i = 0; i < callbackCount; ++i)
					{
						callbacks[i](frame);
					}
				}
			}
		}
	}
}

#endif // defined(BOARD_STM32)
