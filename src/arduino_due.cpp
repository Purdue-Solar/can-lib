/**
 * @file arduino_due.c
 * @author Purdue Soalr Racing (Aidan Orr)
 * @brief
 * @version 0.1
 * @date 2022-09-27
 *
 * @copyright Copyright (c) 2022
 *
 */

#if defined(BOARD_ARDUINO_DUE) || defined(ARDUINO_ARCH_SAM)

#ifndef BOARD_ARDUINO_DUE
#define BOARD_ARDUINO_DUE
#endif

#include "can_lib.h"
#include "due_can.h"
#include <stdbool.h>

namespace PSR
{

CANBus::CANBus(CANBus::Interface& interface, const CANBus::Config& config)
	: _interface(interface), _config(config), _rxCallback(NULL)
{
}

void CANBus::Init()
{
	CANRaw& can = this->_interface;
	can->begin(this->_config.BaudRate);
}

CANBus::TransmitStatus CANBus::Transmit(const CANBus::Frame& frame)
{
	CAN_FRAME dueFrame;

	CANRaw& can = this->_interface;

	dueFrame.id = frame.Id;
	dueFrame.length = frame.Length;
	dueFrame.data.value = frame.Data.Value;
	dueFrame.rtr = frame.IsRTR ? 1 : 0;

	bool status = can.sendFrame(dueFrame);
	return status ? CANBus::TransmitStatus::Success : CANBus::TransmitStatus::Error;
}

static CANBus::Callback _canRxCallback = NULL;

static void _canGeneralCallback(CAN_FRAME* dueFrame)
{
	CANBus::Frame frame;


	if (_canRxCallback != NULL)
	{
		frame.Id = dueFrame->id;
		frame.Length = dueFrame->length;
		frame.IsRTR = dueFrame->rtr != 0;
		frame.Data.Value = dueFrame->data.value;

		(*_canRxCallback)(frame);
	}
}

void CANBus::SetRxCallback(CANBus::Callback callback)
{
	PSR::_canRxCallback = callback;
	this->_rxCallback = callback;

	CANRaw& can = this->_interface;
	can.setGeneralCallback(_canGeneralCallback);
}

void CANBus::ClearRxCallback()
{
	PSR::_canRxCallback = NULL;
	this->_rxCallback = NULL;
}

} // namespace PSR

#endif // BOARD_ARDUION_DUE