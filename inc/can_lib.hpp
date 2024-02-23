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
		uint32_t Words[2];     // Payload represented as an array of 32 bit words
		uint16_t HalfWords[4]; // Payload represented as an array of 16 bit words
		uint8_t Bytes[8];      // Payload represented as an array of bytes

		/**
		 * @brief Construct a new Payload object
		 */
		constexpr Payload() : Value(0) {}
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
		uint32_t FilterIndex; // The filter that matched the frame (only used when receiving frames)
		uint32_t Length;      // Length of payload in bytes
		Payload Data;         // CAN Payload

		/**
		 * @brief Construct a new Frame object
		 */
		constexpr Frame() : Id(0), IsRTR(false), IsExtended(false), FilterIndex(0), Length(0), Data() {}
	};

	/**
	 * @brief Defines a general callback for CAN
	 *
	 * @param frame The received CAN frame
	 * @return void
	 */
	using Callback = std::function<void(CanBus*, const Frame&)>;

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

	union CanId
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

		static constexpr uint8_t DstOffset      = 0;
		static constexpr uint8_t SrcOffset      = 8;
		static constexpr uint8_t MessageOffset  = 16;
		static constexpr uint8_t TypeOffset     = 22;
		static constexpr uint8_t PriorityOffset = 27;

		static constexpr uint8_t MulticastDestination = 0xFF;

		constexpr CanId() : Value(0) {}

		constexpr CanId(uint32_t value) : Value(value) {}

		constexpr CanId(uint8_t dst, uint8_t src, uint8_t message, uint8_t type, uint8_t priority) : Dst(dst), Src(src), Message(message), Type(type), Priority(priority) {}

		static constexpr CanId FromValue(uint32_t value)
		{
			return CanId(value);
		}

		static constexpr CanId DstMask()
		{
			CanId id;
			id.Value = 0xFF;
			return id;
		}

		static constexpr CanId SrcMask()
		{
			CanId id;
			id.Value = 0xFF << SrcOffset;
			return id;
		}

		static constexpr CanId MessageMask()
		{
			CanId id;
			id.Value = 0x3F << MessageOffset;
			return id;
		}

		static constexpr CanId TypeMask()
		{
			CanId id;
			id.Value  = 0x1F << TypeOffset;
			return id;
		}

		static constexpr CanId PriorityMask()
		{
			CanId id;
			id.Value = 0x3 << PriorityOffset;
			return id;
		}

		constexpr bool operator==(const CanId& other) const
		{
			return Value == other.Value;
		}

		constexpr bool operator!=(const CanId& other) const
		{
			return Value != other.Value;
		}

		constexpr CanId operator|(const CanId& other) const
		{
			CanId id = { 0 };
			id.Value = Value | other.Value;
			return id;
		}

		constexpr CanId operator&(const CanId& other) const
		{
			CanId id = { 0 };
			id.Value = Value & other.Value;
			return id;
		}
	};

	static constexpr uint32_t MAX_FILTERS = 8;

	// Static Private Definitions
  private:
	static std::vector<std::tuple<CanBus*, Interface>> RegisteredInterfaces;
	static void RxCallbackFifo0(CAN_HandleTypeDef* hcan);
	static void RxCallbackFifo1(CAN_HandleTypeDef* hcan);
	static void RxCallback(CanBus::Interface hcan, uint32_t fifo);

	// Private Instance Definitions
  private:
	Interface _interface;                         // The handle to the CAN interface
	std::vector<RxCallbackStore> _fifo0Callbacks; // The callbacks for FIFO 0
	std::vector<RxCallbackStore> _fifo1Callbacks; // The callbacks for FIFO 1

	// Public Instance Definitions
  public:
	std::function<void(CanBus*)> TxStartEvent; // The event to call when a transmission starts
	std::function<void(CanBus*)> TxEndEvent;   // The event to call when a transmission completes
	std::function<void(CanBus*)> TxErrorEvent; // The event to call when a transmission errors
	std::function<void(CanBus*)> RxStartEvent; // The event to call when a reception starts
	std::function<void(CanBus*)> RxEndEvent;   // The event to call when a reception completes
	std::function<void(CanBus*)> RxErrorEvent; // The event to call when a reception errors

  public:
	CanBus() : _interface(), _fifo0Callbacks(), _fifo1Callbacks(), TxStartEvent(), TxEndEvent(), TxErrorEvent(), RxStartEvent(), RxEndEvent(), RxErrorEvent() {}

	/**
	 * @brief Create a new CAN object
	 *
	 * @param interface A handle to the CAN interface
	 */
	CanBus(Interface interface);

	/**
	 * @brief Initialize CAN communication
	 *
	 * @return bool Whether the CAN interface was initialized correctly
	 */
	bool Init();

	/**
	 * @brief Transmit a CAN frame
	 *
	 * @param frame The frame data to send
	 * @return bool Whether the frame was transmitted correctly
	 */
	bool Transmit(const Frame& frame);

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

  private:
	bool TranslateNextFrame(Frame& frame, uint32_t fifo);

  public:
	/**
	 * @brief Poll whether a new frame is available.
	 *
	 * @param frame The received frame. Only modified if the function returns true.
	 * @return bool Whether a new frame is available
	 */
	bool Receive(Frame& frame);

	/**
	 * @brief Destroy the CanBus object
	 */
	~CanBus()
	{
		for (auto it = RegisteredInterfaces.begin(); it != RegisteredInterfaces.end(); it++) {
			if (std::get<0>(*it) == this) {
				RegisteredInterfaces.erase(it);
			}
		}
	}
};

} // namespace PSR
