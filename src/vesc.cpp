#include "vesc.h"
#include "can_lib.h"

namespace PSR
{

VescCAN::VescCAN(CANBus& can, const uint8_t controllerId)
    : _can(can), _controllerId(controllerId)
{
}

uint32_t VescCAN::CreateId(uint32_t packet)
{
    return this->_controllerId | (uint32_t)(packet) << 8;
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
    frame.Data.Lower = (uint32_t)(duty * dutyMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(current * currentMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(current * currentMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(rpm * rpmMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(position * positionMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(current * currentMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(current * currentMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(lower * currentMultiplier);
    frame.Data.Upper = (uint32_t)(upper * currentMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(lower * currentMultiplier);
    frame.Data.Upper = (uint32_t)(upper * currentMultiplier);

    this->_can.Transmit(frame);
}

void VescCAN::SetInputCurrentLimits(float lower, float upper)
{
    const uint32_t currentMultiplier = 1000;
    const uint32_t frameSize         = 8;

    CANBus::Frame frame;
    frame.IsExtended = true;
	frame.IsRTR    	 = false;
    frame.Id         = CreateId(PacketId::CONF_CURRENT_LIMITS_IN);
    frame.Length     = frameSize;
    frame.Data.Lower = (uint32_t)(lower * currentMultiplier);
    frame.Data.Upper = (uint32_t)(upper * currentMultiplier);

    this->_can.Transmit(frame);
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
    frame.Data.Lower = (uint32_t)(lower * currentMultiplier);
    frame.Data.Upper = (uint32_t)(upper * currentMultiplier);

    this->_can.Transmit(frame);
}

} // namespace PSR
