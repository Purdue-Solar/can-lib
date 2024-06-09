/**
 * @file can_lib_bxcan.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief STM32 bxCAN implementation file
 * @version 2.1
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef STM32_PROCESSOR
#error "A STM32 processor is not selected"
#else

#include "can_lib.hpp"

#if PSR_CAN_MODE == 1

namespace PSR
{

std::vector<std::tuple<CanBus*, CanBus::Interface*>> CanBus::RegisteredInterfaces = std::vector<std::tuple<CanBus*, CanBus::Interface*>>();

CanBus::CanBus(CanBus::Interface* interface) : _interface(interface), _fifo0Callbacks(), _fifo1Callbacks()
{
	interface->RxFifo0MsgPendingCallback = RxCallbackFifo0;
	interface->RxFifo1MsgPendingCallback = RxCallbackFifo1;
}

bool CanBus::Init()
{
	bool found = false;
	for (std::tuple<CanBus*, CanBus::Interface*>& it : RegisteredInterfaces)
	{
		if (std::get<0>(it) == this)
		{
			found = true;
			break;
		}
	}

	if (!found)
	{
		RegisteredInterfaces.push_back(std::make_tuple(this, interface));
	}

	if (!this->_fifo0Callbacks.empty())
	{
		if (HAL_CAN_ActivateNotification(this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
			return false;
	}
	if (!this->_fifo1Callbacks.empty())
	{
		if (HAL_CAN_ActivateNotification(this->_interface, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
			return false;
	}

	this->_interface->RxFifo0MsgPendingCallback = CanBus::RxCallbackFifo0;
	this->_interface->RxFifo1MsgPendingCallback = CanBus::RxCallbackFifo1;

	this->_interface->Init.AutoRetransmission = ENABLE;
	if (HAL_CAN_Init(this->_interface) != HAL_OK)
		return false;
	if (HAL_CAN_Start(this->_interface) != HAL_OK)
		return false;

	return true;
}

bool CanBus::Transmit(const Frame& frame) const
{
	this->TxStartEvent(this);
	CAN_TxHeaderTypeDef txHeader;

	while (HAL_CAN_GetTxMailboxesFreeLevel(this->_interface) == 0) {}

	txHeader.ExtId = frame.IsExtended ? frame.Id & CanBus::EXT_ID_MASK : 0;
	txHeader.StdId = frame.IsExtended ? 0 : frame.Id & CanBus::STD_ID_MASK;
	txHeader.IDE   = frame.IsExtended ? CAN_ID_EXT : CAN_ID_STD;
	txHeader.DLC   = frame.Length;
	txHeader.RTR   = frame.IsRTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

	uint32_t mailbox;
	bool status = HAL_CAN_AddTxMessage(this->_interface, &txHeader, (uint8_t*)frame.Data.Bytes, &mailbox) == HAL_OK;
	if (!status)
		this->TxErrorEvent(this);

	this->TxEndEvent(this);
	return status;
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
		bool isExtended       = rxHeader.IDE == CAN_ID_EXT;
		frame.Id              = isExtended ? rxHeader.ExtId : rxHeader.StdId;
		frame.Length          = rxHeader.DLC;
		frame.IsRTR           = rxHeader.RTR == CAN_RTR_REMOTE;
		frame.IsExtended      = isExtended;
		frame.IsFilterMatched = true;
		frame.FilterIndex     = rxHeader.FilterMatchIndex;

		return true;
	}

	return false;
}

bool CanBus::Receive(CanBus::Frame& frame)
{
	this->RxStartEvent(this);

	bool status = TranslateNextFrame(this->_interface, frame, CAN_RX_FIFO0) || TranslateNextFrame(this->_interface, frame, CAN_RX_FIFO1);
	if (!status)
		this->RxErrorEvent(this);

	this->RxEndEvent(this);
	return status;
}

// Hack to enable comparison of function pointers
template <typename T, typename... U> size_t getAddress(std::function<T(U...)> f)
{
	typedef T(fnType)(U...);
	fnType** fnPointer = f.template target<fnType*>();
	return (size_t)*fnPointer;
}

bool CanBus::AddRxCallback(Callback callback, const Filter& filter, uint32_t fifo)
{
	CAN_TypeDef* can = this->_interface->Instance;
	can->FMR         = CAN_FMR_FINIT;
	uint32_t i       = 0;
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
	store.Function     = callback;
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

void CanBus::RxCallback(CanBus::Interface* hcan, uint32_t fifo)
{
	for (std::tuple<CanBus*, CanBus::Interface*>& it : CanBus::RegisteredInterfaces)
	{
		if (std::get<1>(it) == hcan)
		{
			CanBus* canbus = std::get<0>(it);
			canbus->RxStartEvent(canbus);

			CanBus::Frame frame;
			if (TranslateNextFrame(hcan, frame, fifo))
			{
				if (fifo == CAN_RX_FIFO0)
				{
					for (auto& callback : canbus->_fifo0Callbacks)
					{
						if (callback.FilterNumber == frame.FilterIndex)
							callback.Function(canbus, frame);
					}
				}
				else if (fifo == CAN_RX_FIFO1)
				{
					for (auto& callback : canbus->_fifo1Callbacks)
					{
						if (callback.FilterNumber == frame.FilterIndex)
							callback.Function(canbus, frame);
					}
				}
			}
			else
			{
				canbus->RxErrorEvent(canbus);
			}

			canbus->RxEndEvent(canbus);
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

#endif

#endif // defined(STM32_PROCESSOR)
