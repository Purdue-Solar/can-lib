/**
 * @file can_ids.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Complete list of IDs for different CAN interfaces and functions
 * @version 1.0
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CAN_IDS_H
#define __CAN_IDS_H

#define CAN_DeviceMask   0x000000FF
#define CAN_FunctionMask 0x1FFFFF00

#define CAN_System_MCU        0x71
#define CAN_System_BMS        0x20
#define CAN_System_Indicators 0x80

#endif // __CAN_IDS_H