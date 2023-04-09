/**
 * @file vesc.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief VESC motor controller implementation
 * @version 1.0
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "vesc.h"
#include "bit_operations.h"
#include "can_lib.h"

namespace PSR
{

void VescCAN::SetDutyCycle(float duty)
{
	constexpr uint32_t dutyMultiplier = 100000;
	constexpr uint32_t frameSize      = 4;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::SET_DUTY);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(duty * dutyMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetCurrent(float current)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 4;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::SET_CURRENT);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(current * currentMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetBrakeCurrent(float current)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 4;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::SET_CURRENT_BRAKE);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(current * currentMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetRPM(int32_t rpm)
{
	constexpr uint32_t frameSize = 4;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::SET_RPM);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness(rpm);

	this->_can.Transmit(frame);
}

void VescCAN::SetPosition(float position)
{
	constexpr uint32_t positionMultiplier = 10000000;
	constexpr uint32_t frameSize          = 4;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::SET_POS);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(position * positionMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetRelativeCurrent(float current)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 4;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::SET_CURRENT_REL);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(current * currentMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetRelativeBrakeCurrent(float current)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 4;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::SET_CURRENT_BRAKE_REL);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(current * currentMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetCurrentLimits(float lower, float upper)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 8;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::CONF_CURRENT_LIMITS);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(lower * currentMultiplier));
	frame.Data.Upper = reverseEndianness((int32_t)(upper * currentMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetCurrentLimitsAndStore(float lower, float upper)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 8;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::CONF_STORE_CURRENT_LIMITS);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(lower * currentMultiplier));
	frame.Data.Upper = reverseEndianness((int32_t)(upper * currentMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetInputCurrentLimits(float lower, float upper)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 8;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::CONF_CURRENT_LIMITS_IN);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(lower * currentMultiplier));
	frame.Data.Upper = reverseEndianness((int32_t)(upper * currentMultiplier));

	this->_can.Transmit(frame);
}

void VescCAN::SetInputCurrentLimitsAndStore(float lower, float upper)
{
	constexpr uint32_t currentMultiplier = 1000;
	constexpr uint32_t frameSize         = 8;

	CANBus::Frame frame;
	frame.IsExtended = true;
	frame.IsRTR      = false;
	frame.Id         = CreateId(PacketId::CONF_STORE_CURRENT_LIMITS_IN);
	frame.Length     = frameSize;
	frame.Data.Lower = reverseEndianness((int32_t)(lower * currentMultiplier));
	frame.Data.Upper = reverseEndianness((int32_t)(upper * currentMultiplier));

	this->_can.Transmit(frame);
}

bool VescCAN::IsVescFrame(const CANBus::Frame& frame, PacketId& packetType)
{
	uint32_t id = bitExtract(frame.Id, 0, 8);
	if (id == this->_deviceId)
	{
		uint32_t statusId = bitExtract(frame.Id, 8, 21);
		packetType        = (VescCAN::PacketId)statusId;
		return true;
	}

	return false;
}

VescCAN::StatusMessage1 VescCAN::DecodeStatusMessage1(const CANBus::Frame& frame)
{
	constexpr float currentMultiplier   = 10;
	constexpr float dutyCycleMultiplier = 1000;

	VescCAN::StatusMessage1 status;

	status.RPM                  = (int32_t)reverseEndianness(frame.Data.Lower);
	status.TotalCurrentConsumed = (int16_t)reverseEndianness(frame.Data.Words[2]) / currentMultiplier;
	status.DutyCycle            = (int16_t)reverseEndianness(frame.Data.Words[3]) / dutyCycleMultiplier;

	return status;
}

VescCAN::StatusMessage2 VescCAN::DecodeStatusMessage2(const CANBus::Frame& frame)
{
	constexpr float ampHoursMultiplier = 10000;

	VescCAN::StatusMessage2 status;

	status.AmpHoursConsumed     = (int32_t)reverseEndianness(frame.Data.Lower) / ampHoursMultiplier;
	status.AmpHoursRegenerative = (int32_t)reverseEndianness(frame.Data.Upper) / ampHoursMultiplier;

	return status;
}

VescCAN::StatusMessage3 VescCAN::DecodeStatusMessage3(const CANBus::Frame& frame)
{
	constexpr float wattHoursMultiplier = 10000;

	VescCAN::StatusMessage3 status;

	status.WattHoursConsumed     = (int32_t)reverseEndianness(frame.Data.Lower) / wattHoursMultiplier;
	status.WattHoursRegenerative = (int32_t)reverseEndianness(frame.Data.Upper) / wattHoursMultiplier;

	return status;
}

VescCAN::StatusMessage4 VescCAN::DecodeStatusMessage4(const CANBus::Frame& frame)
{
	constexpr float temperatureMultiplier = 10;
	constexpr float currentMultiplier     = 10;

	VescCAN::StatusMessage4 status;

	status.MosfetTemperature = (int16_t)reverseEndianness(frame.Data.Words[0]) / temperatureMultiplier;
	status.MotorTemperature  = (int16_t)reverseEndianness(frame.Data.Words[1]) / temperatureMultiplier;
	status.TotalInputCurrent = (int16_t)reverseEndianness(frame.Data.Words[2]) / currentMultiplier;
	status.PidPosition       = (int16_t)reverseEndianness(frame.Data.Words[3]);

	return status;
}

VescCAN::StatusMessage5 VescCAN::DecodeStatusMessage5(const CANBus::Frame& frame)
{
	constexpr float voltageMultiplier = 10;

	VescCAN::StatusMessage5 status;

	status.Tachometer   = (int32_t)reverseEndianness(frame.Data.Lower);
	status.InputVoltage = (int16_t)reverseEndianness(frame.Data.Words[2]) / voltageMultiplier;

	return status;
}

} // namespace PSR
