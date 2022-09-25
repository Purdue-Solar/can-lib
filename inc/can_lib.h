/**
 * @file can_lib.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Multi-target CAN library for Purdue Solar Racing
 * @version 0.1
 * @date 2022-09-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CAN_LIB_H
#define __CAN_LIB_H

#if !(defined(BOARD_ARDUINO_DUE) || defined(BOARD_STM32F))
	#error "A microcontroller board is not selected"
#endif

#include "can_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Initialize CAN communication
 *
 * @param interface A handle to the CAN interface
 * @param config Configuration for CAN interface
 */
void CAN_Init(CAN_Interface* interface, CAN_Config config);

/**
 * @brief Transmit a CAN frame
 *
 * @param interface A handle to the CAN interface
 * @param frame The frame data to send
 * @return A status representing whether the frame was successfully sent
 */
CAN_TransmitStatus CAN_Transmit(CAN_Interface* interface, CAN_Frame* frame);

/**
 * @brief Set a callback that receives all available CAN frames
 * 
 * @param interface A handle to the CAN interface
 * @param callback Function pointer to the callback handler
 */
void CAN_SetRxCallback(CAN_Interface* interface, CAN_Callback* callback);

/**
 * @brief Clear the callback for receiving CAN frames
 * 
 * @param interface A handle to the CAN interface
 */
void CAN_ClearRxCallback(CAN_Interface* interface);

#ifdef __cplusplus
}
#endif

#endif // __CAN_LIB_H