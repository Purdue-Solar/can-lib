/**
 * @file vesc.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief VESC CAN implementation file
 * @version 0.1
 * @date 2022-12-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "vesc.h"
#include "can_lib.h"

namespace PSR
{

VescCAN::VescCAN(CANBus& can, uint8_t controllerId)
    : _can(&can), _controllerId(controllerId)
{
}

VescCAN::VescCAN(CANBus* can, uint8_t controllerId)
    : _can(can), _controllerId(controllerId)
{
}

uint32_t VescCAN::CreateId(PacketId packet)
{
    return this->_controllerId | (uint32_t)(packet) << 8;
}

static int32_t ReverseEndiannessInt32(int32_t value)
{
    uint32_t b0 = (value >> 24) & 0xFF;
    uint32_t b1 = (value >> 16) & 0xFF;
    uint32_t b2 = (value >> 8) & 0xFF;
    uint32_t b3 = (value >> 0) & 0xFF;

    return (b3 << 24) | (b2 << 16) | (b1 << 8) | (b0 << 0);
}

static uint32_t ReverseEndiannessUInt32(uint32_t value)
{
    uint32_t b0 = (value >> 24) & 0xFF;
    uint32_t b1 = (value >> 16) & 0xFF;
    uint32_t b2 = (value >> 8) & 0xFF;
    uint32_t b3 = (value >> 0) & 0xFF;

    return (b3 << 24) | (b2 << 16) | (b1 << 8) | (b0 << 0);
}

void VescCAN::SetDutyCycle(float duty)
{
    const uint32_t dutyMultiplier = 100000;
    const uint32_t frameSize      = 4;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::SET_DUTY);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(duty * dutyMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetCurrent(float current)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 4;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::SET_CURRENT);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(current * currentMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetBrakeCurrent(float current)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 4;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::SET_CURRENT_BRAKE);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(current * currentMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetRPM(float rpm)
{
    const uint32_t rpmMultiplier = 1;
    const uint32_t frameSize     = 4;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::SET_RPM);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(rpm * rpmMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetPosition(float position)
{
    const uint32_t positionMultiplier = 10000000;
    const uint32_t frameSize          = 4;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::SET_POS);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(position * positionMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetRelativeCurrent(float current)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 4;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::SET_CURRENT_REL);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(current * currentMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetRelativeBrakeCurrent(float current)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 4;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::SET_CURRENT_BRAKE_REL);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(current * currentMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetCurrentLimits(float lower, float upper)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 8;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::CONF_CURRENT_LIMITS);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(lower * currentMultiplier));
    frame.Data.Upper = ReverseEndiannessInt32((int32_t)(upper * currentMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetCurrentLimitsAndStore(float lower, float upper)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 8;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::CONF_STORE_CURRENT_LIMITS);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(lower * currentMultiplier));
    frame.Data.Upper = ReverseEndiannessInt32((int32_t)(upper * currentMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetInputCurrentLimits(float lower, float upper)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 8;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::CONF_CURRENT_LIMITS_IN);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(lower * currentMultiplier));
    frame.Data.Upper = ReverseEndiannessInt32((int32_t)(upper * currentMultiplier));

    this->_can->Transmit(frame);
}

void VescCAN::SetInputCurrentLimitsAndStore(float lower, float upper)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 8;

    CANBus::Frame frame;
    frame.IsExtended = true;
    frame.IsRTR      = false;
    frame.Id         = CreateId(PacketId::CONF_STORE_CURRENT_LIMITS_IN);
    frame.Length     = frameSize;
    frame.Data.Lower = ReverseEndiannessInt32((int32_t)(lower * currentMultiplier));
    frame.Data.Upper = ReverseEndiannessInt32((int32_t)(upper * currentMultiplier));

    this->_can->Transmit(frame);
}

// TODO: Add data receive functions

} // namespace PSR
