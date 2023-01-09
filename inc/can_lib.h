/**
 * @file can_lib.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Multi-target CAN library for Purdue Solar Racing
 * @version 0.3
 * @date 2022-09-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CAN_LIB_H
#define __CAN_LIB_H

#if !(defined(BOARD_ARDUINO_DUE) || defined(BOARD_STM32) || defined(BOARD_ARDUINO_UNO) || defined(BOARD_ARDUINO_MEGA))
#error "A microcontroller board is not selected"
#endif

#include <stdbool.h>
#include <stdint.h>

// STM32 Includes
#ifdef BOARD_STM32
#include "stm32_includer.h"
#include STM32_INCLUDE(BOARD_STM32, hal.h)
#include STM32_INCLUDE(BOARD_STM32, hal_def.h)
#include STM32_INCLUDE(BOARD_STM32, hal_can.h)
#endif

// Arduino Due Includes
#ifdef BOARD_ARDUINO_DUE
#include "due_can.h"
#endif // BOARD_ARDUINO_DUE

// Arduino Uno/Mega Includes
#ifdef BOARD_ARDUINO_UNO
#include "uno_common.h"
#endif

#ifdef BOARD_ARDUINO_MEGA
#include "mega_common.h"
#endif

/**
 * @brief Default CAN baud rate of 100kb/s
 */
#define CAN_BAUD_RATE 100000

namespace PSR
{

class CANBus
{
  public:
	static const uint32_t STD_ID_MASK = 0x7FF;
	static const uint32_t EXT_ID_MASK = 0x1FFFFFFF;

#include "_can_interface_alias.h"

	/**
	 * @brief Status of a CAN transmission
	 */
	enum struct TransmitStatus
	{
		Unknown,
		Success,
		Error
	};

	/**
	 * @brief Defines the mode used to receive data over the CAN bus.
	 */
	enum struct ReceiveMode
	{
		Callback,
		Polling
	};

	/**
	 * @brief Represents a CAN payload in many ways
	 * @remark Assumes little endian byte ordering
	 */
	union Payload
	{
		uint64_t Value; // Payload reprented as a 64 bit number
		struct
		{
			uint32_t Lower; // Lower 32 bits of the payload
			uint32_t Upper; // Upper 32 bits of the payload
		};
		uint32_t DoubleWords[2]; // Payload represented as an array of 32 bit words
		uint16_t Words[4];       // Payload represented as an array of 16 bit words
		uint8_t Bytes[8];        // Payload represented as an array of bytes
	};

	/**
	 * @brief Represents a CAN Frame
	 */
	struct Frame
	{
		uint32_t Id;     // 11 or 29 bit CAN Identifier
		bool IsRTR;      // Remote Transmission Request
		bool IsExtended; // Whether the frame is an extended or standard frame
		uint32_t Length; // Length of payload in bytes
		Payload Data;    // CAN Payload
	};

	/**
	 * @brief CAN Configuration
	 */
	struct Config
	{
		uint32_t BaudRate;		// Baud rate to use for transmitting and receiving. May not be used.
		bool AutoRetransmit;	// Whether frames should be retransmitted on failure.
		uint32_t FilterMask;	// Mask to use for filtering CAN ids.
		ReceiveMode Mode;		// Receive mode to use. Supported modes: polling or callback based receive.
	};

	/**
	 * @brief Defines a general callback for CAN
	 *
	 * @param frame The received CAN frame
	 * @return void
	 */
	typedef void (*Callback)(const Frame&);

  private:
	Interface& _interface;
	Config _config;
	Callback _rxCallback;

  public:
	/**
	 * @brief Create a new CAN object
	 *
	 * @param interface A handle to the CAN interface
	 * @param config Configuration for CAN interface
	 */
	CANBus(Interface& interface, const Config& config);

	/**
	 * @brief Initialize CAN communication
	 */
	void Init();

	/**
	 * @brief Transmit a CAN frame
	 *
	 * @param frame The frame data to send
	 * @return A status representing whether the frame was successfully sent
	 */
	TransmitStatus Transmit(const Frame& frame);

	/**
	 * @brief Set a callback that receives all available CAN frames.
	 *
	 * @param callback A function pointer to the callback to set. Passing a nullptr will clear the callback.
	 * @return Whether the callback was set correctly. Returns false if the ReceiveMode is set to a value other than ReceiveMode::Callback
	 */
	bool SetCallback(Callback callback);

	/**
	 * @brief Poll whether a new frame is available.
	 * 
	 * @param frame The received frame. Only modified if the function returns true.
	 * @return Whether a new frame is available
	 */
	bool Receive(Frame& frame);

	/**
	 * @brief Destroy the CANBus object
	 */
	~CANBus()
	{
		SetCallback(nullptr);
	}
};

} // namespace PSR

#endif // __CAN_LIB_H
