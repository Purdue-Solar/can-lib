/**
 * @file arduino_due.c
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Arduino Due CAN implementation file
 * @version 0.9
 *
 * @copyright Copyright (c) 2023
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

void CANBus::Init()
{
	this->_interface.begin(this->_config.BaudRate);
}

CANBus::TransmitStatus CANBus::Transmit(const CANBus::Frame& frame)
{
	CAN_FRAME dueFrame;

	CANRaw& can = this->_interface;

	dueFrame.id         = frame.Id;
	dueFrame.length     = frame.Length;
	dueFrame.data.value = frame.Data.Value;
	dueFrame.rtr        = frame.IsRTR ? 1 : 0;

	bool status = can.sendFrame(dueFrame);
	return status ? CANBus::TransmitStatus::Success : CANBus::TransmitStatus::Error;
}

static bool TranslateFrame(const CAN_FRAME& dueFrame, CANBus::Frame& frame)
{
	frame.Id         = dueFrame.id;
	frame.Length     = dueFrame.length;
	frame.IsRTR      = dueFrame.rtr != 0;
	frame.IsExtended = dueFrame.extended != 0;
	frame.Data.Value = dueFrame.data.value;
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
		auto lambdaCallback = [this](CAN_FRAME* dueFrame) -> void
		{
			CANBus::Frame frame;
			if (TranslateFrame(*dueFrame, frame))
			{
				this->_rxCallback(frame);
			}
		};

		this->_interface.setGeneralCallback((void (*)(CAN_FRAME*)) & lambdaCallback);
	}
	else
	{
		this->_interface.setGeneralCallback(nullptr);
	}

	return true;
}

bool CANBus::Receive(CANBus::Frame& frame)
{
	if (this->_interface.rx_avail())
	{
		CAN_FRAME dueFrame;
		this->_interface.read(dueFrame);
		TranslateFrame(dueFrame, frame);
		
		return true;
	}

	return false;
}

} // namespace PSR

#endif // BOARD_ARDUION_DUE