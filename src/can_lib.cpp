/**
 * @file can_lib.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief STM32 CAN implementation file
 * @version 2.0
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef STM32_PROCESSOR
#error "A processor type is not selected"
#else

#include "can_lib.hpp"

namespace PSR
{

std::vector<std::tuple<CanBus*, CanBus::Interface>> CanBus::RegisteredInterfaces = std::vector<std::tuple<CanBus*, CanBus::Interface>>();

void CanBus::Init()
{
	if (!this->_fifo0Callbacks.empty())
		HAL_CAN_ActivateNotification(this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);
	if (!this->_fifo1Callbacks.empty())
		HAL_CAN_ActivateNotification(this->_interface, CAN_IT_RX_FIFO1_MSG_PENDING);

	this->_interface->RxFifo0MsgPendingCallback = CanBus::RxCallbackFifo0;
	this->_interface->RxFifo1MsgPendingCallback = CanBus::RxCallbackFifo1;

		this->_interface->Init.AutoRetransmission = ENABLE;
	HAL_CAN_Init(this->_interface);
	HAL_CAN_Start(this->_interface);
}

CanBus::TransmitStatus CanBus::Transmit(const Frame& frame)
{
	CAN_TxHeaderTypeDef txHeader;

	while (HAL_CAN_GetTxMailboxesFreeLevel(this->_interface) == 0)
	{}

	txHeader.ExtId = frame.IsExtended ? frame.Id & CanBus::EXT_ID_MASK : 0;
	txHeader.StdId = frame.IsExtended ? 0 : frame.Id & CanBus::STD_ID_MASK;
	txHeader.IDE   = frame.IsExtended ? CAN_ID_EXT : CAN_ID_STD;
	txHeader.DLC   = frame.Length;
	txHeader.RTR   = frame.IsRTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

	uint32_t mailbox;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(this->_interface, &txHeader, (uint8_t*)frame.Data.Bytes, &mailbox);

	switch (status)
	{
	case HAL_OK:
		return CanBus::TransmitStatus::Success;
	case HAL_ERROR:
		return CanBus::TransmitStatus::Error;
	default:
		return CanBus::TransmitStatus::Unknown;
	}
}

/**
 * @brief Try to receive a frame from the interface and update a reference to a frame
 *
 * @param hcan A pointer to the CAN interface
 * @param frame The frame to be updated with the received frame
 * @param fifo The number of the FIFO buffer to receive from
 * @return bool Whether there was a frame available and it was successfully translated.
 */
static bool TranslateNextFrame(CAN_HandleTypeDef* hcan, CanBus::Frame& frame, uint32_t fifo)
{
	if (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) == 0)
	{
		return false;
	}

	CAN_RxHeaderTypeDef rxHeader;
	HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, fifo, &rxHeader, frame.Data.Bytes);

	// Only modify frame if the message is received properly
	if (status == HAL_OK)
	{
		bool isExtended   = rxHeader.IDE == CAN_ID_EXT;
		frame.Id          = isExtended ? rxHeader.ExtId : rxHeader.StdId;
		frame.Length      = rxHeader.DLC;
		frame.IsRTR       = rxHeader.RTR == CAN_RTR_REMOTE;
		frame.IsExtended  = isExtended;
		frame.FilterIndex = rxHeader.FilterMatchIndex;

		return true;
	}

	return false;
}

bool CanBus::Receive(CanBus::Frame& frame)
{
	return TranslateNextFrame(this->_interface, frame, CAN_RX_FIFO0) || TranslateNextFrame(this->_interface, frame, CAN_RX_FIFO1);
}

bool CanBus::RemoveRxCallback(Callback callback, uint32_t fifo)
{
	CAN_TypeDef* can = this->_interface->Instance;
	std::vector<RxCallbackStore>& callbacks = fifo == CAN_RX_FIFO0 ? this->_fifo0Callbacks : this->_fifo1Callbacks;

	bool found = false;
	for (auto it = callbacks.begin(); it != callbacks.end(); it++)
	{
		if ((*it).Function == callback)
		{
			can->FMR = CAN_FMR_FINIT;
			can->FA1R &= ~(1 << (*it).FilterNumber);
			can->FMR = 0;

			callbacks.erase(it);
			found = true;
		}
	}

	return found;
}

bool CanBus::AddRxCallback(Callback callback, const Filter& filter, uint32_t fifo)
{
	CAN_TypeDef* can = this->_interface->Instance;
	can->FMR = CAN_FMR_FINIT;
	uint32_t i = 0;
	while (i < CanBus::MAX_FILTERS)
	{
		if ((can->FA1R & (1 << i)) == 0)
			break;
		i++;
	}

	if (i == CanBus::MAX_FILTERS)
		return false;

	can->FS1R |= (1 << i);

	can->FFA1R &= ~(1 << i);
	can->FFA1R |= fifo == 1 ? (1 << i) : 0;

	can->sFilterRegister[i].FR1 = filter.Id << 3;
	can->sFilterRegister[i].FR2 = filter.Mask << 3;

	can->FA1R |= 1 << i;
	can->FMR = 0;

	CanBus::RxCallbackStore store;
	store.Function = callback;
	store.FilterNumber = i;

	switch (fifo)
	{
	case CAN_RX_FIFO0:

		this->_fifo0Callbacks.push_back(store);
		break;
	case CAN_RX_FIFO1:
		this->_fifo1Callbacks.push_back(store);
		break;
	default:
		break;
	}

	if (!this->_fifo0Callbacks.empty())
		HAL_CAN_ActivateNotification(this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);
	if (!this->_fifo1Callbacks.empty())
		HAL_CAN_ActivateNotification(this->_interface, CAN_IT_RX_FIFO1_MSG_PENDING);

	return true;
}

void RxCallback(CanBus::Interface hcan, uint32_t fifo)
{
	for (std::tuple<CanBus*, CanBus::Interface>& it : CanBus::RegisteredInterfaces)
	{
		if (std::get<1>(it) == hcan)
		{
			CanBus* canbus = std::get<0>(it);

			CanBus::Frame frame;
			if (TranslateNextFrame(hcan, frame, fifo))
			{
				if (fifo == CAN_RX_FIFO0)
				{
					for (auto& callback : canbus->_fifo0Callbacks)
					{
						if (callback.FilterNumber == frame.FilterIndex)
							callback.Function(frame);
					}
				}
				else if (fifo == CAN_RX_FIFO1)
				{
					for (auto& callback : canbus->_fifo1Callbacks)
					{
						if (callback.FilterNumber == frame.FilterIndex)
							callback.Function(frame);
					}
				}
			}
		}
	}
}

void CanBus::RxCallbackFifo0(CAN_HandleTypeDef* hcan)
{
	RxCallback(hcan, CAN_RX_FIFO0);
}

void CanBus::RxCallbackFifo1(CAN_HandleTypeDef* hcan)
{
	RxCallback(hcan, CAN_RX_FIFO1);
}

} // namespace PSR

#endif // defined(STM32_PROCESSOR)
