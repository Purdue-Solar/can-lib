/**
 * @file can_lib.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Multi-target CAN library for Purdue Solar Racing
 * @version 1.0
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#ifndef STM32_PROCESSOR
#error "A microcontroller board is not selected"
#endif

#include <cstdbool>
#include <cstdint>
#include <functional>
#include <tuple>
#include <vector>

// STM32 Includes
#include "stm32_includer.h"
#include STM32_INCLUDE(STM32_PROCESSOR, hal.h)
#include STM32_INCLUDE(STM32_PROCESSOR, hal_def.h)
#include STM32_INCLUDE(STM32_PROCESSOR, hal_can.h)

namespace PSR
{

class CanBus
{
  public:
	static constexpr uint32_t STD_ID_MASK = 0x7FF;
	static constexpr uint32_t EXT_ID_MASK = 0x1FFFFFFF;

	typedef CAN_HandleTypeDef* Interface;

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
	 * @brief Represents a CAN filter
	 */
	struct Filter
	{
		uint32_t Id;   // 11 or 29 bit CAN Identifier
		uint32_t Mask; // Mask for the filter
	};

	/**
	 * @brief Represents a CAN Frame
	 */
	struct Frame
	{
		uint32_t Id;          // 11 or 29 bit CAN Identifier
		bool IsRTR;           // Remote Transmission Request
		bool IsExtended;      // Whether the frame is an extended or standard frame
		uint32_t FilterIndex; // The filter that matched the frame
		uint32_t Length;      // Length of payload in bytes
		Payload Data;         // CAN Payload
	};

	/**
	 * @brief Defines a general callback for CAN
	 *
	 * @param frame The received CAN frame
	 * @return void
	 */
	typedef void (*Callback)(const Frame&);

	struct RxCallbackStore
	{
		Callback Function;
		uint32_t FilterNumber;
	};

	struct Priority
	{
		constexpr static uint32_t Highest = 0;
		constexpr static uint32_t High    = 1;
		constexpr static uint32_t Normal  = 2;
		constexpr static uint32_t Low     = 3;
	};

	typedef union
	{
		struct
		{
			uint32_t Dst : 8;      // 8 bit destination ID
			uint32_t Src : 8;      // 8 bit source ID
			uint32_t Message : 6;  // 6 bit message ID
			uint32_t Type : 5;     // 5 bit device type
			uint32_t Priority : 2; // 2 bit priority, lower is higher priority
		};
		uint32_t Value;
	} CanId;

	static constexpr uint32_t MAX_FILTERS = 8;
	static std::vector<std::tuple<CanBus*, Interface>> RegisteredInterfaces;

	Interface _interface;
	std::vector<RxCallbackStore> _fifo0Callbacks;
	std::vector<RxCallbackStore> _fifo1Callbacks;

  private:
	static void RxCallbackFifo0(CAN_HandleTypeDef* hcan);
	static void RxCallbackFifo1(CAN_HandleTypeDef* hcan);

  public:
	/**
	 * @brief Create a new CAN object
	 *
	 * @param interface A handle to the CAN interface
	 */
	CanBus(Interface interface)
		: _interface(interface), _fifo0Callbacks(), _fifo1Callbacks()
	{
		interface->RxFifo0MsgPendingCallback = RxCallbackFifo0;
		interface->RxFifo1MsgPendingCallback = RxCallbackFifo1;

		bool found = false;
		for (std::tuple<CanBus*, Interface>& it : RegisteredInterfaces)
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
	}

	/**
	 * @brief Initialize CAN communication
	 */
	void Init();

	/**
	 * @brief Transmit a CAN frame
	 *
	 * @param frame The frame data to send
	 * @return TransmitStatus A status representing whether the frame was successfully sent
	 */
	TransmitStatus Transmit(const Frame& frame);

	/**
	 * @brief Add a callback that receives frames that match a specific filter.
	 *
	 * @param callback A function pointer to the callback to add.
	 * @param filter The filter to match frames against.
	 * @param fifo The number of the FIFO buffer to receive from
	 * @return bool Whether the callback was added correctly.
	 */
	bool AddRxCallback(Callback callback, const Filter& filter, uint32_t fifo = CAN_RX_FIFO0);

	/**
	 * @brief Remove a callback that receives frames
	 *
	 * @param callback A function pointer to the callback to remove.
	 * @return bool Whether the callback was removed correctly.
	 */
	bool RemoveRxCallback(Callback callback, uint32_t fifo = CAN_RX_FIFO0);

	/**
	 * @brief Poll whether a new frame is available.
	 *
	 * @param frame The received frame. Only modified if the function returns true.
	 * @return bool Whether a new frame is available
	 */
	bool Receive(Frame& frame);

	/**
	 * @brief Destroy the CANBus object
	 */
	~CanBus()
	{
	}
};

} // namespace PSR
