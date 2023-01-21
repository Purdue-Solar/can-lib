/**
 * @file bms.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Battery management system CAN communication
 * @version 0.1
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __BMS_H
#define __BMS_H

#include "can_lib.h"
#include <cstdint>

namespace PSR
{

class BmsCAN
{
  public:
	enum struct PacketId : uint32_t
	{
		TEMPERATURE_AND_STATE = 0,
		CURRENT_AND_VOLTAGE = 1,
		UNKNOWN
	};

  private:
	CANBus& _can;
	uint8_t _deviceId;

	uint32_t CreateId(PacketId packet)
	{
		return this->_deviceId | (uint32_t)(packet) << 8;
	}

  public:
	BmsCAN() = default;

	BmsCAN(CANBus& can, uint8_t deviceId)
		: _can(can), _deviceId(deviceId)
	{}

	/**
	 * @brief Test whether the received frame is a valid BMS frame and updates the packet type with the received type
	 *
	 * @param frame The received frame
	 * @param packetType The packet type of the received frame.
	 * @return bool whether the frame is a BMS frame for this device.
	 */
	bool IsBmsFrame(const CANBus::Frame& frame, PacketId& packetType);

	struct TemperatureAndStateMessage
	{
		static constexpr PacketId PacketType = PacketId::TEMPERATURE_AND_STATE;

		float InternalTemperature;
		float HighestCellTemperature;
		float LowestCellTemperature;
		uint8_t RelayState;
		float StateOfCharge;
	};

	TemperatureAndStateMessage DecodeTemperatureAndState(const CANBus::Frame& frame);

	struct CurrentAndVoltageMessage
	{
		static constexpr PacketId PacketType = PacketId::CURRENT_AND_VOLTAGE;

		float PackCurrent;
		float PackVoltage;
		float HighestCellVoltage;
		float LowestCellVoltage;
	};

	CurrentAndVoltageMessage DecodeCurrentAndVoltage(const CANBus::Frame& frame);
};

} // namespace PSR

#endif // end of include guard for bms.h