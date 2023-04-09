/**
 * @file car_indicators.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief CAN messages related to the car indicators.
 * @version 0.1
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CAR_INDICATORS_H
#define __CAR_INDICATORS_H

#include "can_lib.h"

namespace PSR
{

class CarIndicators
{
  private:
	CANBus& _can;
	uint8_t _deviceId;

  public:
	/**
	 * @brief Construct a new Car Indicators object
	 *
	 * @param can a reference to the CAN Bus object
	 * @param deviceId an 8 bit id for referencing the indicators
	 */
	CarIndicators(CANBus& can, uint8_t deviceId)
		: _can(can), _deviceId(deviceId)
	{}

	enum struct IndicatorFlags
	{
		Lights       = 0b0001,
		Wipers       = 0b0010,
		LeftBlinker  = 0b0100,
		RightBlinker = 0b1000
	};
	
	enum struct Direction
	{
		Neutral = 0,
		Forward = 1,
		Reverse = 2,
		Invalid = 3
	};

	struct IndicatorStatus
	{
		IndicatorFlags Flags;
		Direction Direction;
	};

	/**
	 * @brief Transmits the car's current indicator status and motor direction
	 * 
	 * @param indicators bit field containing the different indicators
	 * @param direction direction that the motor is driving
	 */
	void TransmitIndicatorStatus(IndicatorFlags indicators, Direction direction);

	/**
	 * @brief Checks whether the current CAN frame contains indicator data
	 *
	 * @param frame the frame to decode
	 * @return bool whether the current frame is an indicator frame
	 */
	bool IsIndicatorFrame(const CANBus::Frame& frame)

	/**
	 * @brief 
	 * 
	 * @param frame reference to CAN frame
	 * @return The decoded indicator status
	 */
	IndicatorStatus DecodeIndicatorStatus(const CANBus::Frame& frame);
};

} // namespace PSR

#endif // end of include guard for car_indicators.h