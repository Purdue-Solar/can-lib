/**
 * @file can_interface_alias.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief CAN Interface aliases
 * @version 0.2
 * @date 2022-09-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CAN_INTERFACE_ALIAS_H
#define __CAN_INTERFACE_ALIAS_H

//
// STM32 aliases
//

#ifdef BOARD_STM32F

typedef CAN_HandleTypeDef Interface;

#endif // BOARD_STM32F

//
// Arduino Due aliases
//

#ifdef BOARD_ARDUINO_DUE

typedef CANRaw Interface;

#endif // BOARD_ARDUINO_DUE

//
// Arduino Uno/Mega aliases
//

#if defined(BOARD_ARDUINO_UNO) || defined(BOARD_ARDUINO_MEGA)

typedef MCP2515 Interface;

#endif

#endif // __CAN_INTERFACE_ALIAS_H