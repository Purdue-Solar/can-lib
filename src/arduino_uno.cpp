/**
 * @file arduino_uno.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#if defined(BOARD_ARDUINO_UNO) || defined(ARDUINO_ARCH_AVR)

#ifndef BOARD_ARDUINO_UNO
#define BOARD_ARDUINO_UNO
#endif

#include "can_lib.h"
#include "mcp2515.h"
#include <stdbool.h>
#include <stdlib.h>

extern "C"
{
	void CAN_Init(CAN_Interface* interface, CAN_Config config)
	{
		MCP2515* can = interface;

		can->reset();
		int speed;

		switch (config.BaudRate)
		{
		case 50000:
			speed = CAN_50KBPS;
			break;
		case 100000:
			speed = CAN_100KBPS;
			break;
		case 125000:
			speed = CAN_125KBPS;
			break;
		case 200000:
			speed = CAN_200KBPS;
			break;
		case 250000:
			speed = CAN_250KBPS;
			break;
		case 500000:
			speed = CAN_500KBPS;
			break;
		case 1000000:
			speed = CAN_1000KBPS;
			break;
		default:
			speed = *(int*)NULL; // SEGFAULT because I'm too lazy to set a default.
			break;
		}

		can->setBitrate(speed);
		can->setNormalMode();
	}

	CAN_TransmitStatus CAN_Transmit(CAN_Interface* interface, CAN_Frame* frame)
	{
		struct can_frame mcpFrame;
		MCP2515* can = interface;

		/*
		 * Controller Area Network Identifier structure
		 *
		 * bit 0-28 : CAN identifier (11/29 bit)
		 * bit 29   : error message frame flag (0 = data frame, 1 = error message)
		 * bit 30   : remote transmission request flag (1 = rtr frame)
		 * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
		 */
		uint32_t isRTR = frame->IsRTR ? 1 : 0;
		uint32_t isExtended = frame->IsExtended ? 1 : 0;
		uint32_t id = (isExtended << 31) | (isRTR << 30) | frame->Id;

		mcpFrame.can_id = id;
		mcpFrame.can_dlc = frame->Length;
		memcpy(mcpFrame.data, frame->Data.Bytes, frame->Length);

		MCP2515::ERROR status = can->sendMessage(&mcpFrame);

		return status == MCP2515::ERROR_OK ? TransmitStatus_Success : TransmitStatus_Error;
	}

	static MCP2515* _callbackMCP;
	static CAN_Callback* _canRxCallback = NULL;
	static void _canInterrupt()
	{
		struct can_frame mcpFrame;
		CAN_Frame frame;

		if (_callbackMCP != NULL)
		{
			uint8_t irq = _callbackMCP->getInterrupts();

			if (irq != 0 && _canInterrupt != NULL)
			{
				while (_callbackMCP->readMessage(&mcpFrame) == MCP2515::ERROR_OK)
				{
					frame.Id = mcpFrame.can_id & 0x1FFFFFFF;
					frame.IsExtended = (mcpFrame.can_id >> 31) & 1 == 1;
					frame.IsRTR = (mcpFrame.can_id >> 30) & 1 == 1;
					frame.Length = mcpFrame.can_dlc;
					memcpy(frame.Data.Bytes, mcpFrame.data, mcpFrame.can_dlc);

					_canRxCallback(&frame);
				}
			}
		}
	}

	void CAN_SetRxCallback(CAN_Interface* interface, CAN_Callback* callback)
	{
		MCP2515* can = interface;
		_callbackMCP = can;
		_canRxCallback = callback;
		attachInterrupt(0, _canInterrupt, FALLING);
	}

	void CAN_ClearRxCallback(CAN_Interface* interface)
	{
		_canRxCallback = NULL;
		_callbackMCP = NULL;
		detachInterrupt(0);
	}
}

#endif