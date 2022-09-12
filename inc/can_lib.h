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

#include "can_types.h"

/**
 * @brief Initialize CAN communication
 *
 * @param handle A handle to the CAN interface
 * @param config Configuration for CAN interface
 */
void CAN_Init(CAN_Handle *handle, CAN_Config config);

/**
 * @brief Transmit a CAN frame
 *
 * @param handle A handle to the CAN interface
 * @param frame The frame data to send
 * @return A status representing whether the frame was successfully sent
 */
CAN_TransmitStatus CAN_Transmit(CAN_Handle *handle, CAN_Frame *frame);

/**
 * @brief Receive the next CAN frame in a blocking way
 *
 * @param handle A handle to the CAN interface
 * @param frame Pointer to retrieved frame
 */
void CAN_ReceiveNext(CAN_Handle *handle, CAN_Frame *frame);

/**
 * @brief Set a callback that receives all available CAN frames
 * 
 * @param handle A handle to the CAN interfaces
 * @param callback Function pointer to the callback handler
 */
void CAN_SetGeneralCallback(CAN_Handle *handle, CAN_Callback *callback);

#endif // __CAN_LIB_H