/**
 * @file can_lib_fdcan.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief STM32 FDCAN implementation file
 * @version 2.1
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef STM32_PROCESSOR
#error "A STM32 processor is not selected"
#else

#include "can_lib.hpp"
#include "interrupt_queue.hpp"
#include <cmath>
#ifdef PRINT_DEBUG
#include <cstdio>
#endif

#if PSR_CAN_MODE == 2

namespace PSR
{

std::vector<std::tuple<CanBus*, CanBus::Interface*>> CanBus::RegisteredInterfaces = std::vector<std::tuple<CanBus*, CanBus::Interface*>>();

CanBus::CanBus(CanBus::Interface* interface) : _interface(interface), _fifo0Callbacks(), _fifo1Callbacks()
{
	interface->RxFifo0Callback = CanBus::RxCallbackFifo0;
	interface->RxFifo1Callback = CanBus::RxCallbackFifo1;
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
		RegisteredInterfaces.push_back(std::make_tuple(this, this->_interface));
	}

	HAL_FDCAN_Stop(this->_interface);

	if (!this->_fifo0Callbacks.empty())
	{
		if (HAL_FDCAN_ActivateNotification(this->_interface, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
			return false;
	}
	if (!this->_fifo1Callbacks.empty())
	{
		if (HAL_FDCAN_ActivateNotification(this->_interface, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
			return false;
	}

	this->_interface->RxFifo0Callback = CanBus::RxCallbackFifo0;
	this->_interface->RxFifo1Callback = CanBus::RxCallbackFifo1;

	this->_interface->Init.AutoRetransmission = ENABLE;
	this->_interface->Init.TransmitPause      = DISABLE;

	this->_interface->Init.StdFiltersNbr = 0;
	this->_interface->Init.ExtFiltersNbr = 0;

	if (HAL_FDCAN_Init(this->_interface) != HAL_OK)
		return false;
	 if (HAL_FDCAN_ConfigGlobalFilter(this->_interface, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
	 	return false;
	if (HAL_FDCAN_Start(this->_interface) != HAL_OK)
		return false;

#ifdef PRINT_DEBUG
	printf("\tInitialized CanBus.\n");
#endif

	return true;
}

static void PrintFrameInfo(const PSR::CanBus::Frame& frame, const char* prefix)
{
	printf("CAN %s: (",  prefix);

	if (frame.IsExtended)
		printf("Id: %8lX, ", frame.Id);
	else
		printf("Id: %3lX, ", frame.Id);

	printf("Len: %ld, Data:", frame.Length);
	for (size_t i = 0; i < frame.Length; i++)
		printf(" %X%X", frame.Data.Bytes[i] >> 4, frame.Data.Bytes[i] & 0xF);

	printf(")\n");
}

bool CanBus::Transmit(const Frame& frame) const
{
	constexpr uint32_t timeout = 20;

	this->TxStartEvent(this);
	FDCAN_TxHeaderTypeDef txHeader;

#ifdef PRINT_DEBUG
	PrintFrameInfo(frame, "TX");
#endif

	uint32_t tickStart = HAL_GetTick();
	while (HAL_FDCAN_GetTxFifoFreeLevel(this->_interface) == 0)
	{
		uint32_t tick = HAL_GetTick();
		if ((tick - tickStart) > timeout)
			return false;
	}

	txHeader.Identifier          = frame.Id & (frame.IsExtended ? CanBus::EXT_ID_MASK : CanBus::STD_ID_MASK);
	txHeader.IdType              = frame.IsExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	txHeader.TxFrameType         = frame.IsRTR ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
	txHeader.DataLength          = frame.Length & 0xF;
	txHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
	txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker       = 0;

	bool status = HAL_FDCAN_AddMessageToTxFifoQ(this->_interface, &txHeader, (uint8_t*)frame.Data.Bytes) == HAL_OK;
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
static bool TranslateNextFrame(FDCAN_HandleTypeDef* hfdcan, CanBus::Frame& frame, uint32_t fifo)
{
	if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) == 0)
	{
		return false;
	}

	FDCAN_RxHeaderTypeDef rxHeader;
	HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rxHeader, frame.Data.Bytes);

	// Only modify frame if the message is received properly
	if (status == HAL_OK)
	{
		bool isExtended       = rxHeader.IdType == FDCAN_EXTENDED_ID;
		frame.Id              = rxHeader.Identifier & (isExtended ? CanBus::EXT_ID_MASK : CanBus::STD_ID_MASK);
		frame.Length          = rxHeader.DataLength;
		frame.IsRTR           = rxHeader.RxFrameType == FDCAN_REMOTE_FRAME;
		frame.IsExtended      = isExtended;
		frame.IsFilterMatched = rxHeader.IsFilterMatchingFrame == 0;
		frame.FilterIndex     = rxHeader.FilterIndex;

		return true;
	}

	return false;
}

bool CanBus::Receive(CanBus::Frame& frame) const
{
	this->RxStartEvent(this);

	bool status = TranslateNextFrame(this->_interface, frame, CanBus::RX_FIFO0) || TranslateNextFrame(this->_interface, frame, CanBus::RX_FIFO1);
	if (!status)
		this->RxErrorEvent(this);
	else
	{
#ifdef PRINT_DEBUG
	PrintFrameInfo(frame, "RX");
#endif
	}

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
	HAL_FDCAN_Stop(this->_interface);
	HAL_FDCAN_Init(this->_interface);

	FDCAN_FilterTypeDef fdcanFilter;

	switch (filter.Type)
	{
	case CanBus::FilterType::RANGE:
		fdcanFilter.FilterType = FDCAN_FILTER_RANGE;
		fdcanFilter.FilterID1  = filter.Id;
		fdcanFilter.FilterID2  = filter.Id2;
		break;
	case CanBus::FilterType::DUAL:
		fdcanFilter.FilterType = FDCAN_FILTER_DUAL;
		fdcanFilter.FilterID1  = filter.Id;
		fdcanFilter.FilterID2  = filter.Id2;
		break;
	case CanBus::FilterType::ID_MASK:
		fdcanFilter.FilterType = FDCAN_FILTER_MASK;
		fdcanFilter.FilterID1  = filter.Id;
		fdcanFilter.FilterID2  = filter.Mask;
		break;
	default:
		return false;
	}

	int currentFilterIndex = 0;
	if (!filter.IsExtended)
	{
		currentFilterIndex = this->_interface->Init.StdFiltersNbr;

		if (currentFilterIndex >= 28)
			return false;

		this->_interface->Init.StdFiltersNbr++;
		HAL_FDCAN_Init(this->_interface);

		fdcanFilter.IdType       = FDCAN_STANDARD_ID;
		fdcanFilter.FilterIndex  = currentFilterIndex;
		fdcanFilter.FilterConfig = fifo == CanBus::RX_FIFO0 ? FDCAN_FILTER_TO_RXFIFO0_HP : FDCAN_FILTER_TO_RXFIFO1_HP;
	}
	else
	{
		currentFilterIndex = this->_interface->Init.ExtFiltersNbr;

		if (currentFilterIndex >= 8)
			return false;

		this->_interface->Init.ExtFiltersNbr++;
		HAL_FDCAN_Init(this->_interface);

		fdcanFilter.IdType       = FDCAN_EXTENDED_ID;
		fdcanFilter.FilterIndex  = currentFilterIndex;
		fdcanFilter.FilterConfig = fifo == CanBus::RX_FIFO0 ? FDCAN_FILTER_TO_RXFIFO0_HP : FDCAN_FILTER_TO_RXFIFO1_HP;
	}

	if (HAL_FDCAN_ConfigFilter(this->_interface, &fdcanFilter) != HAL_OK)
		return false;

	CanBus::RxCallbackStore store;
	store.Function     = callback;
	store.Type         = filter.Type;
	store.IsExtended   = filter.IsExtended;
	store.FilterNumber = currentFilterIndex;

	switch (fifo)
	{
	case CanBus::RX_FIFO0:
		this->_fifo0Callbacks.push_back(store);
		break;
	case CanBus::RX_FIFO1:
		this->_fifo1Callbacks.push_back(store);
		break;
	default:
		return false;
		break;
	}

	if (!this->_fifo0Callbacks.empty())
		HAL_FDCAN_ActivateNotification(this->_interface, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	if (!this->_fifo1Callbacks.empty())
		HAL_FDCAN_ActivateNotification(this->_interface, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

	return HAL_FDCAN_Start(this->_interface) == HAL_OK;
}

void CanBus::RxCallback(CanBus::Interface* hcan, uint32_t fifo)
{
	if (fifo != CanBus::RX_FIFO0 && fifo != CanBus::RX_FIFO1)
		return;

	for (std::tuple<CanBus*, CanBus::Interface*>& it : CanBus::RegisteredInterfaces)
	{
		if (std::get<1>(it) == hcan)
		{
			CanBus* canbus = std::get<0>(it);
			if (canbus->RxStartEvent)
				canbus->RxStartEvent(canbus);

			CanBus::Frame frame;
			if (TranslateNextFrame(hcan, frame, fifo))
			{
#ifdef PRINT_DEBUG
				PrintFrameInfo(frame, "RX");
#endif
				if (fifo == CanBus::RX_FIFO0)
				{
					for (auto& callback : canbus->_fifo0Callbacks)
					{
						if (callback.FilterNumber == frame.FilterIndex && callback.IsExtended == frame.IsExtended)
						{
							auto function = [canbus, frame, callback]() {callback.Function(canbus, frame);};
							InterruptQueue::AddInterrupt(Interrupt(function, HAL_GetTick(), "CAN FIFO 0 Interrupt", true));
						}
					}
				}
				else if (fifo == CanBus::RX_FIFO1)
				{
					for (auto& callback : canbus->_fifo1Callbacks)
					{
						if (callback.FilterNumber == frame.FilterIndex && callback.IsExtended == frame.IsExtended)
						{
							auto function = [canbus, frame, callback]() {callback.Function(canbus, frame);};
							InterruptQueue::AddInterrupt(Interrupt(function, HAL_GetTick(), "CAN FIFO 1 Interrupt", true));
						}
					}
				}
			}
			else
			{
#ifdef PRINT_DEBUG
				printf("CAN RX Error.\n");
#endif
				if (canbus->RxErrorEvent)
					canbus->RxErrorEvent(canbus);
			}

			if (canbus->RxEndEvent)
				canbus->RxEndEvent(canbus);
		}
	}
}

void CanBus::RxCallbackFifo0(FDCAN_HandleTypeDef* hfdcan, uint32_t rxFifo0ITs)
{
	RxCallback(hfdcan, CanBus::RX_FIFO0);
}

void CanBus::RxCallbackFifo1(FDCAN_HandleTypeDef* hfdcan, uint32_t rxFifo1ITs)
{
	RxCallback(hfdcan, CanBus::RX_FIFO1);
}

} // namespace PSR

#endif

#endif // defined(STM32_PROCESSOR)
