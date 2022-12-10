/**
 * @file f_common.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief STM32-F Series board common headers
 * @version 0.1
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __F_COMMON_H
#define __F_COMMON_H

#define STRINGIFY(x)  _STRINGIFY(x)
#define _STRINGIFY(x) #x

#define CAT(x, y)        CAT2(x, y)
#define CAT2(x, y)       x##y
#define CAT3(x, y, z)    CAT(x, CAT2(y, z))
#define CAT4(x, y, z, w) CAT(x, CAT3(y, z, w))

#define INCLUDE_FILE_2(a, b)    STRINGIFY(CAT(a, b))
#define INCLUDE_FILE_3(a, b, c) STRINGIFY(CAT3(a, b, c))

#ifndef BOARD_STM32F
#error STM32F board type not defined
#endif

#define __BOARD_ALIAS CAT(f, BOARD_STM32F)

#include INCLUDE_FILE_3(stm32, __BOARD_ALIAS, xx_hal.h);
#include INCLUDE_FILE_3(stm32, __BOARD_ALIAS, xx_hal_def.h);
#include INCLUDE_FILE_3(stm32, __BOARD_ALIAS, xx_hal_can.h);

#endif // End of include guard for f_common.h