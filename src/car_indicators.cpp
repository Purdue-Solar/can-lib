/**
 * @file car_indicators.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Implementation of car indicator statuses
 * @version 0.1
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "car_indicators.h"

namespace PSR
{

void CarIndicators::TransmitIndicatorStatus(CarIndicators::IndicatorFlags indicators, CarIndicators::Direction direction)
{
	constexpr uint32_t frameSize = 2;

	CANBus::Frame frame;
	frame.IsExtended    = true;
	frame.IsRTR         = false;
	frame.Id            = this->_deviceId;
	frame.Length        = frameSize;
	frame.Data.Bytes[0] = (uint8_t)indicators;
	frame.Data.Bytes[1] = (uint8_t)direction;

	this->_can.Transmit(frame);
}

bool CarIndicators::IsIndicatorFrame(const CANBus::Frame& frame)
{
	return bitExtract(frame.Id, 0, 8) == this->_deviceId;
}

CarIndicators::IndicatorStatus DecodeIndicatorStatus(const CANBus::Frame& frame)
{
	CarIndicators::IndicatorStatus status;

	status.Flags     = (CarIndicators::IndicatorFlags)frame.Data.Bytes[0];
	status.Direction = (CarIndicators::Direction)frame.Data.Bytes[1];

	return status;
}

} // namespace PSR