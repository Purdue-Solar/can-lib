/**
 * @file can_types.h
 * @author your name (you@domain.com)
 * @brief CAN type definitions
 * @version 0.1
 * @date 2022-09-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CAN_TYPES_H
#define __CAN_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// STM32 F3 aliases
#ifdef BOARD_STM32F3

#include "stm32f3xx/common.h"
#define CAN_Handle CAN_HandleTypeDef

#endif // BOARD_STM32F3

// STM32 F4 aliases
#ifdef BOARD_STM32F4

#include "stm32f4xx/common.h"
#define CAN_Handle CAN_HandleTypeDef

#endif // BOARD_STM32F4

// Arduino Due aliases
#ifdef BOARD_ARDUINO_DUE

#include "arduino_due/common.h"
#define CAN_Handle CANRaw

#endif // BOARD_ARDUINO_DUE

//
// Common Type Definitions
//

/**
 * @brief Represents a CAN payload in many ways
 * 
 */
typedef union
{
	uint64_t data;			// Payload reprented as a 64 bit number
	struct 
	{
		uint32_t lower;		// Lower 32 bits of the payload
		uint32_t upper;		// Upper 32 bits of the payload
	};
	struct 
	{
		uint16_t w0;		// First 16 bit word of the payload
		uint16_t w1;		// Second 16 bit word of the payload
		uint16_t w2;		// Third 16 bit word of the payload
		uint16_t w3;		// Fourth 16 bit word of the payload
	};
	uint8_t bytes[8];		// Payload represented as an array of bytes
} CAN_Payload;


/**
 * @brief Represents a standard (non-extended) CAN Frame
 * 
 */
typedef struct
{
	uint16_t id;		// 11bit CAN Identifier
	bool rtr;			// Remote Transmission Request
	uint8_t length;		// Length of payload in bytes
	CAN_Payload data;	// CAN Payload
} CAN_Frame;

#endif // __CAN_TYPES_H