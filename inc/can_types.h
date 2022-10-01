/**
 * @file can_types.h
 * @author Purdue Solar Racing (Aidan Orr)
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

//
// Common Type Definitions
//

/**
 * @brief Represents a CAN payload in many ways
 * @remark Assumes little endian architecture
 */
typedef union
{
	uint64_t Value; // Payload reprented as a 64 bit number
	struct
	{
		uint32_t Lower;	// Lower 32 bits of the payload
		uint32_t Upper;	// Upper 32 bits of the payload
	};
	uint16_t Words[4];	// Payload represented as an array of words
	uint8_t Bytes[8];	// Payload represented as an array of bytes
} CAN_Payload;

/**
 * @brief Represents a standard (non-extended) CAN Frame
 *
 */
typedef struct
{
	uint32_t Id;	  // 11 bit CAN Identifier
	bool RTR;		  // Remote Transmission Request
	uint32_t Length;  // Length of payload in bytes
	CAN_Payload Data; // CAN Payload
} CAN_Frame;

/**
 * @brief CAN Configuration
 *
 */
typedef struct CAN_Config
{
	uint32_t BaudRate;
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
typedef enum
{
	TransmitStatus_Unknown,
	TransmitStatus_Success,
	TransmitStatus_Error
} CAN_TransmitStatus;

//
// Board Specific Definitions
//

//
// STM32 aliases
//

#ifdef BOARD_STM32F

#if BOARD_STM32F == 0
#include "stm32f0xx/common.h"
#elif BOARD_STM32F == 1
#include "stm32f1xx/common.h"
#elif BOARD_STM32F == 2
#include "stm32f2xx/common.h"
#elif BOARD_STM32F == 3
#include "stm32f3xx/common.h"
#elif BOARD_STM32F == 4
#include "stm32f4xx/common.h"
#else
#error "STM32 board type not recognized"
#endif

typedef struct
{
	CAN_HandleTypeDef* Handle;
	uint32_t Mailbox;
} CAN_Interface;

#endif // BOARD_STM32F

	//
	// Arduino Due aliases
	//

#ifdef BOARD_ARDUINO_DUE

#include "arduino_due/common.h"

#define CAN_Interface CANRaw

#endif // BOARD_ARDUINO_DUE

#endif // __CAN_TYPES_H