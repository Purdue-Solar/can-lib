/**
 * @file can_ids.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Complete list of IDs for different CAN interfaces and functions
 * @version 0.1
 * @date 2022-09-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CAN_IDS_H
#define __CAN_IDS_H

#define CAN_SystemMask		0x1FFFFFF0
#define CAN_FunctionMask	0x0000000F

#define CAN_System_VCU		0x000
// VCU Functions

#define CAN_System_MCU		0x010
// MCU Functions

#define CAN_System_BMS		0x020
// BMS Functions

#define CAN_System_MPPT		0x040
// MPPT Functions

#endif // __CAN_IDS_H