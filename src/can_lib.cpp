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

CanBus::CanBus(CanBus::Interface interface)
	: _interface(interface), _fifo0Callbacks(), _fifo1Callbacks(), TxStartEvent(), TxEndEvent(), TxErrorEvent(), RxStartEvent(), RxEndEvent(), RxErrorEvent()
{
	interface->RxFifo0MsgPendingCallback = RxCallbackFifo0;
	interface->RxFifo1MsgPendingCallback = RxCallbackFifo1;

	bool found = false;
	for (std::tuple<CanBus*, Interface>& it : RegisteredInterfaces) {
		if (std::get<0>(it) == this) {
			found = true;
			break;
		}
	}

	if (!found) {
		RegisteredInterfaces.push_back(std::make_tuple(this, interface));
	}
}

bool CanBus::Init()
{
	if (!this->_fifo0Callbacks.empty()) {
		if (HAL_CAN_ActivateNotification(this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
			return false;
	}
	if (!this->_fifo1Callbacks.empty()) {
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

bool CanBus::Transmit(const Frame& frame)
{
	// Disallow transmitting remote frames
	if (frame.IsRTR)
		return false;

	if (this->TxStartEvent)
		this->TxStartEvent(this);

	CAN_HandleTypeDef* interface = this->_interface;

	// Wait for a free mailbox
	while ((interface->Instance->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) {
	}

	CAN_TxHeaderTypeDef txHeader = { 0 };
	if (frame.IsExtended)
		txHeader.ExtId = frame.Id & CanBus::EXT_ID_MASK;
	else
		txHeader.StdId = frame.Id & CanBus::STD_ID_MASK;
	txHeader.IDE = frame.IsExtended ? CAN_ID_EXT : CAN_ID_STD;
	txHeader.DLC = frame.Length;

	uint32_t mailbox;
	bool status = HAL_CAN_AddTxMessage(interface, &txHeader, frame.Data.Bytes, &mailbox) == HAL_OK;
	if (!status && this->TxErrorEvent)
		this->TxErrorEvent(this);

	if (this->TxEndEvent)
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
bool CanBus::TranslateNextFrame(CanBus::Frame& frame, uint32_t fifo)
{
	CAN_HandleTypeDef* hcan = this->_interface;

	if (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) == 0) {
		return false;
	}

	CAN_RxHeaderTypeDef rxHeader;
	HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, fifo, &rxHeader, frame.Data.Bytes);

	// Only modify frame if the message is received properly
	if (status == HAL_OK) {
		frame.IsExtended  = rxHeader.IDE == CAN_ID_EXT;
		frame.Id          = frame.IsExtended ? rxHeader.ExtId : rxHeader.StdId;
		frame.Length      = rxHeader.DLC;
		frame.IsRTR       = rxHeader.RTR == CAN_RTR_REMOTE;
		frame.FilterIndex = rxHeader.FilterMatchIndex;

		return true;
	}

	this->RxErrorEvent(this);

	return false;
}

bool CanBus::Receive(CanBus::Frame& frame)
{
	if (this->RxStartEvent)
		this->RxStartEvent(this);

	bool status = TranslateNextFrame(frame, CAN_RX_FIFO0) || TranslateNextFrame(frame, CAN_RX_FIFO1);

	if (this->RxEndEvent)
		this->RxEndEvent(this);
	return status;
}

// Hack to enable comparison of function pointers
template <typename T, typename... U> const void* getAddress(std::function<T(U...)> f)
{
	typedef T(fnType)(U...);
	fnType** fnPointer = f.template target<fnType*>();
	return (void*)*fnPointer;
}

bool CanBus::RemoveRxCallback(Callback callback, uint32_t fifo)
{
	CAN_TypeDef* can = this->_interface->Instance;

	std::vector<RxCallbackStore>& callbacks = fifo == CAN_RX_FIFO0 ? this->_fifo0Callbacks : this->_fifo1Callbacks;

	bool found = false;
	for (auto it = callbacks.begin(); it != callbacks.end(); it++) {
		if (getAddress((*it).Function) == getAddress(callback)) {
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
	if (!callback)
		return false;

	// Invalid FIFO
	if (fifo > CAN_RX_FIFO1)
		return false;

	CAN_TypeDef* can = this->_interface->Instance;
	can->FMR         = CAN_FMR_FINIT;
	uint32_t i       = 0;
	while (i < CanBus::MAX_FILTERS) {
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

	switch (fifo) {
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

void CanBus::RxCallback(CanBus::Interface hcan, uint32_t fifo)
{
	for (std::tuple<CanBus*, CanBus::Interface>& it : CanBus::RegisteredInterfaces) {
		if (std::get<1>(it) == hcan) {
			CanBus* canbus = std::get<0>(it);
			if (canbus->RxStartEvent)
				canbus->RxStartEvent(canbus);

			CanBus::Frame frame;
			if (canbus->TranslateNextFrame(frame, fifo)) {
				if (fifo == CAN_RX_FIFO0) {
					for (auto& callback : canbus->_fifo0Callbacks) {
						if (callback.FilterNumber == frame.FilterIndex)
							callback.Function(canbus, frame);
					}
				}
				else if (fifo == CAN_RX_FIFO1) {
					for (auto& callback : canbus->_fifo1Callbacks) {
						if (callback.FilterNumber == frame.FilterIndex)
							callback.Function(canbus, frame);
					}
				}
			}

			if (canbus->RxEndEvent)
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

#endif // defined(STM32_PROCESSOR)
