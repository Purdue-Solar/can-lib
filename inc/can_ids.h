/**
 * @file can_ids.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Complete list of IDs for different CAN interfaces and functions
 * @version 0.2
 * @date 2022-09-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CAN_IDS_H
#define __CAN_IDS_H

#define CAN_DeviceMask		0x000000FF
#define CAN_FunctionMask	0x1FFFFF00

#define CAN_System_VCU		0x000
// VCU Functions

#define CAN_System_MCU		113
// MCU Functions

#define CAN_System_BMS		0x020
// BMS Functions

#define CAN_System_MPPT		0x040
// MPPT Functions

#endif // __CAN_IDS_H