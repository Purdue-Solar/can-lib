/**
 * @file bms.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Battery management system CAN communication
 * @version 0.1
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bms.h"
#include "bit_operations.h"
#include "can_lib.h"

namespace PSR
{

bool BmsCAN::IsBmsFrame(const CANBus::Frame& frame, BmsCAN::PacketId& packetType)
{
	uint32_t id = bitExtract(frame.Id, 0, 8);
	if (id == this->_deviceId)
	{
		uint32_t statusId = bitExtract(frame.Id, 8, 21);
		packetType        = (BmsCAN::PacketId)statusId;
		return true;
	}

	return false;
}

BmsCAN::TemperatureAndStateMessage BmsCAN::DecodeTemperatureAndState(const CANBus::Frame& frame)
{
	constexpr float stateOfChargeMultiplier = 2;

	BmsCAN::TemperatureAndStateMessage status;

	status.InternalTemperature    = frame.Data.Bytes[0];
	status.HighestCellTemperature = frame.Data.Bytes[1];
	status.LowestCellTemperature  = frame.Data.Bytes[2];

	status.RelayState    = frame.Data.Bytes[6];
	status.StateOfCharge = frame.Data.Bytes[7] / stateOfChargeMultiplier;

	return status;
}

BmsCAN::CurrentAndVoltageMessage BmsCAN::DecodeCurrentAndVoltage(const CANBus::Frame& frame)
{
	constexpr float currentMultiplier     = 10;
	constexpr float packVoltageMultiplier = 100;
	constexpr float cellVoltageMultiplier = 10000;

	BmsCAN::CurrentAndVoltageMessage status;

	status.PackCurrent        = ((int16_t)reverseEndianness(frame.Data.Words[0])) / currentMultiplier;
	status.PackVoltage        = reverseEndianness(frame.Data.Words[1]) / packVoltageMultiplier;
	status.HighestCellVoltage = reverseEndianness(frame.Data.Words[2]) / cellVoltageMultiplier;
	status.LowestCellVoltage  = reverseEndianness(frame.Data.Words[3]) / cellVoltageMultiplier;

	return status;
}

} // namespace PSR
