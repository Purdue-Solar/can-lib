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

#if !(defined(BOARD_ARDUINO_DUE) || defined(BOARD_STM32F) || defined(BOARD_ARDUINO_UNO))
#error "A microcontroller board is not selected"
#endif

#include <stdbool.h>
#include <stdint.h>

// STM Includes
#ifdef BOARD_STM32F

#if BOARD_STM32F == 0
#include "stm32/f0_common.h"
#elif BOARD_STM32F == 1
#include "stm32/f1_common.h"
#elif BOARD_STM32F == 2
#include "stm32/f2_common.h"
#elif BOARD_STM32F == 3
#include "stm32/f3_common.h"
#elif BOARD_STM32F == 4
#include "stm32/f4_common.h"
#else
#error STM32 board type not recognized
#endif

#endif

// Arduino Due Includes
#ifdef BOARD_ARDUINO_DUE

#include "due_can.h"

#endif // BOARD_ARDUINO_DUE

// Arduino Uno/Mega Includes
#if defined(BOARD_ARDUINO_UNO) || defined(BOARD_ARDUINO_MEGA)

#ifdef BOARD_ARDUINO_UNO
#include "uno_common.h"
#else
#include "mega_common.h"
#endif

#endif

/**
 * @brief Default CAN baud rate of 100kb/s
 */
#define CAN_BAUD_RATE 100000

namespace PSR
{

class CANBus
{
  public:
    static const uint32_t STD_ID_MASK = 0x7FF;
    static const uint32_t EXT_ID_MASK = 0x1FFFFFFF;

#include "_can_interface_alias.h"

    /**
     * @brief Status of a CAN transmission
     *
     */
    typedef enum
    {
        Unknown,
        Success,
        Error
    } TransmitStatus;

    /**
     * @brief Represents a CAN payload in many ways
     * @remark Assumes little endian architecture
     */
    typedef union {
        uint64_t Value; // Payload reprented as a 64 bit number
        struct
        {
            uint32_t Lower; // Lower 32 bits of the payload
            uint32_t Upper; // Upper 32 bits of the payload
        };
        uint32_t DoubleWords[2]; // Payload represented as an array of double words
        uint16_t Words[4];       // Payload represented as an array of words
        uint8_t Bytes[8];        // Payload represented as an array of bytes
    } Payload;

    /**
     * @brief Represents a CAN Frame
     */
    typedef struct
    {
        uint32_t Id; // 11 or 29 bit CAN Identifier
        bool IsRTR;  // Remote Transmission Request
        bool IsExtended;
        uint32_t Length; // Length of payload in bytes
        Payload Data;    // CAN Payload
    } Frame;

    /**
     * @brief CAN Configuration
     */
    typedef struct
    {
        uint32_t BaudRate;
        bool AutoRetransmit;
        uint32_t FilterMask;
    } Config;

    /**
     * @brief Defines a general callback for CAN
     *
     * @param frame The received CAN frame
     * @return void
     */
    typedef void (*Callback)(const Frame&);

  private:
    Interface& _interface;
    Config _config;
    Callback _rxCallback;

  public:
    /**
     * @brief Create a new CAN object
     *
     * @param interface A handle to the CAN interface
     * @param config Configuration for CAN interface
     */
    CANBus(Interface& interface, const Config& config);

    /**
     * @brief Initialize CAN communication
     */
    void Init();

    /**
     * @brief Transmit a CAN frame
     *
     * @param frame The frame data to send
     * @return A status representing whether the frame was successfully sent
     */
    TransmitStatus Transmit(const Frame& frame);

    /**
     * @brief Set a callback that receives all available CAN frames
     *
     * @param callback Function pointer to the callback handler
     */
    void SetRxCallback(Callback callback);

    /**
     * @brief Clear the callback for receiving CAN frames
     */
    void ClearRxCallback();
};

} // namespace PSR

#endif // __CAN_LIB_H
