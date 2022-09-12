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
//#define CAN_ExternalTxHeader CAN_TxHeaderTypeDef

#endif // BOARD_STM32F3

// STM32 F4 aliases
#ifdef BOARD_STM32F4

#include "stm32f4xx/common.h"

#define CAN_Handle CAN_HandleTypeDef
//#define CAN_ExternalTxHeader CAN_TxHeaderTypeDef

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
 * @remark Assumes little endian architecture
 */
typedef union
{
	uint64_t Data;			// Payload reprented as a 64 bit number
	struct 
	{
		uint32_t Lower;		// Lower 32 bits of the payload
		uint32_t Upper;		// Upper 32 bits of the payload
	};
	struct 
	{
		uint16_t W0;		// First 16 bit word of the payload
		uint16_t W1;		// Second 16 bit word of the payload
		uint16_t W2;		// Third 16 bit word of the payload
		uint16_t W3;		// Fourth 16 bit word of the payload
	};
	uint8_t Bytes[8];		// Payload represented as an array of bytes
} CAN_Payload;


/**
 * @brief Represents a standard (non-extended) CAN Frame
 * 
 */
typedef struct
{
	uint16_t Id;		// 11 bit CAN Identifier
	bool RTR;			// Remote Transmission Request
	uint8_t Length;		// Length of payload in bytes
	CAN_Payload Data;	// CAN Payload
} CAN_Frame;

/**
 * @brief CAN Configuration
 * 
 */
typedef struct
{
	bool AutoRetransmit;
	uint16_t FilterIdLow;
	uint16_t FilterIdHigh;
} CAN_Config;


/**
 * @brief Defines a general callback for CAN
 * 
 * @param frame The received CAN frame
 * @return void 
 */
typedef void CAN_Callback(CAN_Frame *frame);

/**
 * @brief Status of a CAN transmission
 * 
 */
enum CAN_TransmitStatus
{
	Unknown,
	Success,
	Failure
};

#endif // __CAN_TYPES_H