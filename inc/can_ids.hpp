/**
 * @file can_ids.hpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Base CAN IDs for custom PSR CAN messages
 * @version 0.1
 * @date 2024-02-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstdint>
namespace PSR
{

// 5 Bit Can Type
struct CanType
{
	static constexpr uint8_t MPPT             = 0x0;
	static constexpr uint8_t BMS              = 0x1;
	static constexpr uint8_t MOTOR_CONTROLLER = 0x2;
	static constexpr uint8_t DISPLAY          = 0x3;
	static constexpr uint8_t DISTRIBUTION     = 0x4;
	static constexpr uint8_t PERIPHERALS      = 0x5;
	static constexpr uint8_t STEERING         = 0x6;
	static constexpr uint8_t GENERIC          = 0x1F;
};

struct CanIdBase
{
	static constexpr uint8_t MPPT             = CanType::MPPT << 4;
	static constexpr uint8_t BMS              = CanType::BMS << 4;
	static constexpr uint8_t MOTOR_CONTROLLER = CanType::MOTOR_CONTROLLER << 4;
	static constexpr uint8_t DISPLAY          = CanType::DISPLAY << 4;
	static constexpr uint8_t DISTRIBUTION     = CanType::DISTRIBUTION << 4;
	static constexpr uint8_t PERIPHERALS      = CanType::PERIPHERALS << 4;
	static constexpr uint8_t STEERING         = CanType::STEERING << 4;
};

struct GenericMessage
{
	// Generic message rate in milliseconds
	static constexpr uint32_t GenericRate = 2000;

	// Generic message IDs
	static constexpr uint8_t HEARTBEAT         = 0x00;
	static constexpr uint8_t VOLTAGE_CURRENT_0 = 0x01;
	static constexpr uint8_t VOLTAGE_CURRENT_1 = 0x02;
	static constexpr uint8_t VOLTAGE_CURRENT_2 = 0x03;
	static constexpr uint8_t VOLTAGE_CURRENT_3 = 0x04;

	static constexpr uint8_t ERRORS_0 = 0x20;
	static constexpr uint8_t ERRORS_1 = 0x21;
	static constexpr uint8_t ERRORS_2 = 0x22;
	static constexpr uint8_t ERRORS_3 = 0x23;

	static constexpr uint8_t RESET    = 0x3F;
};

} // namespace PSR
