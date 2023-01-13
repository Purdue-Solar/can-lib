/**
 * @file arduino_mega.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Arduino Uno CAN implementation file
 * @version 0.8
 *
 * @copyright Copyright (c) 2023
 *
 */

#if defined(BOARD_ARDUINO_MEGA) || defined(ARDUINO_ARCH_AVR)

#ifndef BOARD_ARDUINO_MEGA
#define BOARD_ARDUINO_MEGA
#endif

#include "bit_operations.h"
#include "can_lib.h"
#include "mcp2515.h"
#include <stdbool.h>
#include <string.h>

namespace PSR
{

void CANBus::Init()
{
	MCP2515& can = this->_interface;
	can.reset();
	int speed;

	switch (this->_config.BaudRate)
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
		speed = CAN_100KBPS;
		break;
	}

	can.setBitrate(speed);
	can.setNormalMode();
}

CANBus::TransmitStatus CANBus::Transmit(const CANBus::Frame& frame)
{
	struct can_frame mcpFrame;
	MCP2515& can = this->_interface;

	/*
	 * Controller Area Network Identifier structure
	 *
	 * bit 0-28 : CAN identifier (11/29 bit)
	 * bit 29   : error message frame flag (0 = data frame, 1 = error message)
	 * bit 30   : remote transmission request flag (1 = rtr frame)
	 * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
	 */
	uint32_t isRTR      = frame.IsRTR ? 1 : 0;
	uint32_t isExtended = frame.IsExtended ? 1 : 0;

	uint32_t id = (isExtended << 31) | (isRTR << 30) | frame.Id;

	mcpFrame.can_id  = id;
	mcpFrame.can_dlc = frame.Length;
	memcpy(mcpFrame.data, frame.Data.Bytes, frame.Length);

	MCP2515::ERROR status = can.sendMessage(&mcpFrame);

	return status == MCP2515::ERROR_OK ? CANBus::TransmitStatus::Success : CANBus::TransmitStatus::Error;
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

static void _canGeneralCallback();

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
	}
	else
	{
		// Remove all instances of current CAN object from storage
		for (int i = 0; i < _callbackStorageSize; ++i)
		{
			if (_canRxCallbackStore[i].busObject == this)
			{
				_canRxCallbackStore[i] = { nullptr, nullptr, nullptr };
			}
		}
	}

	if (IsCallbackStorageEmpty())
		detachInterrupt(digitalPinToInterrupt(2));
	else
		attachInterrupt(digitalPinToInterrupt(2), _canGeneralCallback, FALLING);

	return true;
}

/**
 * @brief Try to receive a frame from the interface and update a reference to a frame
 *
 * @param mcp A pointer to the MCP2515 CAN interface
 * @param frame The frame to be updated with the received frame
 * @return bool Whether there was a frame available and it was successfully translated.
 */
static bool TranslateNextFrame(MCP2515* mcp, CANBus::Frame& frame)
{
	if (!mcp->checkReceive())
	{
		return false;
	}

	struct can_frame mcpFrame;
	if (mcp->readMessage(&mcpFrame) == MCP2515::ERROR_OK)
	{
		/*
		 * Controller Area Network Identifier structure
		 *
		 * bit 0-28 : CAN identifier (11/29 bit)
		 * bit 29   : error message frame flag (0 = data frame, 1 = error message)
		 * bit 30   : remote transmission request flag (1 = rtr frame)
		 * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
		 */
		frame.Id         = mcpFrame.can_id & 0x1FFFFFFF;
		frame.IsRTR      = isBitSet(mcpFrame.can_id, 30);
		frame.IsExtended = isBitSet(mcpFrame.can_id, 31);
		frame.Length     = mcpFrame.can_dlc;

		frame.Data.Value = 0; // Zero payload as a precaution
		memcpy(frame.Data.Bytes, mcpFrame.data, mcpFrame.can_dlc);

		return true;
	}

	return false;
}

bool CANBus::Receive(CANBus::Frame& frame)
{
	return TranslateNextFrame(&this->_interface, frame);
}

static void _canGeneralCallback()
{
	CANBus::Frame frame;

	// Get interrupts for each registered callback.
	for (int i = 0; i < _callbackStorageSize; ++i)
	{
		if (_canRxCallbackStore[i].interface != nullptr)
		{
			uint8_t irq = _canRxCallbackStore[i].interface->getInterrupts();

			if (irq > 0)
			{
				// Receive and translate all frames on this interface
				while (_canRxCallbackStore[i].interface->checkReceive())
				{
					if (TranslateNextFrame(_canRxCallbackStore[i].interface, frame))
					{
						// Execute callback on each registered callback that has an identical interface
						for (int j = 0; j < _callbackStorageSize; ++j)
						{
							if (_canRxCallbackStore[i].interface == _canRxCallbackStore[i].interface)
								_canRxCallbackStore[i].callback(frame);
						}
					}
				}
			}
		}
	}
}

} // namespace PSR

#endif