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

#include <stdbool.h>

#include "can_types.h"

/**
 * @brief Initialize CAN communication
 *
 * @param handle A handle to the CAN interface
 */
void CAN_Init(CAN_Handle *handle);

/**
 * @brief Transmit a CAN frame
 *
 * @param handle A handle to the CAN interface
 * @param frame The frame data to send
 * @return A boolean representing whether the frame was successfully sent
 */
bool CAN_Transmit(CAN_Handle *handle, CAN_Frame *frame);

/**
 * @brief Receive a CAN frame in a blocking way
 *
 * @param handle A handle to the CAN interface
 * @param frame Pointer to retrieved frame
 * 
 */
void CAN_Receive(CAN_Handle *handle, CAN_Frame *frame);

#endif // __CAN_LIB_H