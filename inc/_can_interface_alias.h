/**
 * @file _can_interface_alias.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief CAN Interface aliases
 * @version 1.0
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ___CAN_INTERFACE_ALIAS_H
#define ___CAN_INTERFACE_ALIAS_H

//
// STM32 aliases
//

#ifdef BOARD_STM32

typedef CAN_HandleTypeDef Interface;

#endif // BOARD_STM32F

//
// Arduino Due aliases
//

#ifdef BOARD_ARDUINO_DUE

typedef CANRaw Interface;

#endif // BOARD_ARDUINO_DUE

//
// Arduino Uno/Mega (AVR) aliases
//

#ifdef BOARD_ARDUINO_AVR

typedef MCP2515 Interface;

#endif

#endif // end of include guard for _can_interface_alias.h