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

void CAN_Init(CAN_Interface* interface, CAN_Config config)
{
	CANRaw* can = interface;
	can->begin(config.BaudRate);
	can->watchForRange(config.FilterIdLow, config.FilterIdHigh);
}

CAN_TransmitStatus CAN_Transmit(CAN_Interface* interface, CAN_Frame* frame)
{
	CAN_FRAME dueFrame;

	CANRaw* can = interface;

	dueFrame.id = frame->Id;
	dueFrame.length = frame->Length;
	dueFrame.data.value = frame->Data.Value;
	dueFrame.rtr = frame->RTR ? 1 : 0;

	bool status = can->sendFrame(dueFrame);
	return status ? TransmitStatus_Success : TransmitStatus_Error;
}

CAN_Callback* _canRxCallback = NULL;
static void _canGeneralCallback(CAN_FRAME* dueFrame)
{
	CAN_Frame frame;

	if (_canRxCallback != NULL)
	{
		frame.Id = dueFrame->id;
		frame.Length = dueFrame->length;
		frame.RTR = dueFrame->rtr != 0;
		frame.Data.Value = dueFrame->data.value;

		(*_canRxCallback)(&frame);
	}
}

void CAN_SetRxCallback(CAN_Interface* interface, CAN_Callback* callback)
{
	CANRaw* can = interface;
	_canRxCallback = callback;
	can->setGeneralCallback(_canGeneralCallback);
}

void CAN_ClearRxCallback(CAN_Interface* interface)
{
	_canRxCallback = NULL;
}

#endif // BOARD_ARDUION_DUE