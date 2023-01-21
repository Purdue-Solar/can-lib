/**
 * @file vesc.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief VESC motor controller CAN communication
 * @version 0.8
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __VESC_H
#define __VESC_H

#include "can_lib.h"
#include <cstdint>

namespace PSR
{

class VescCAN
{
  public:
	enum struct PacketId : uint32_t
	{
		SET_DUTY                     = 0,
		SET_CURRENT                  = 1,
		SET_CURRENT_BRAKE            = 2,
		SET_RPM                      = 3,
		SET_POS                      = 4,
		FILL_RX_BUFFER               = 5,
		FILL_RX_BUFFER_LONG          = 6,
		PROCESS_RX_BUFFER            = 7,
		PROCESS_SHORT_BUFFER         = 8,
		STATUS                       = 9,
		SET_CURRENT_REL              = 10,
		SET_CURRENT_BRAKE_REL        = 11,
		SET_CURRENT_HANDBRAKE        = 12,
		SET_CURRENT_HANDBRAKE_REL    = 13,
		STATUS_2                     = 14,
		STATUS_3                     = 15,
		STATUS_4                     = 16,
		PING                         = 17,
		PONG                         = 18,
		DETECT_APPLY_ALL_FOC         = 19,
		DETECT_APPLY_ALL_FOC_RES     = 20,
		CONF_CURRENT_LIMITS          = 21,
		CONF_STORE_CURRENT_LIMITS    = 22,
		CONF_CURRENT_LIMITS_IN       = 23,
		CONF_STORE_CURRENT_LIMITS_IN = 24,
		CONF_FOC_ERPMS               = 25,
		CONF_STORE_FOC_ERPMS         = 26,
		STATUS_5                     = 27,
		POLL_TS5700N8501_STATUS      = 28,
		CONF_BATTERY_CUT             = 29,
		CONF_STORE_BATTERY_CUT       = 30,
		SHUTDOWN                     = 31,
		IO_BOARD_ADC_1_TO_4          = 32,
		IO_BOARD_ADC_5_TO_8          = 33,
		IO_BOARD_ADC_9_TO_12         = 34,
		IO_BOARD_DIGITAL_IN          = 35,
		IO_BOARD_SET_OUTPUT_DIGITAL  = 36,
		IO_BOARD_SET_OUTPUT_PWM      = 37,
		BMS_V_TOT                    = 38,
		BMS_I                        = 39,
		BMS_AH_WH                    = 40,
		BMS_V_CELL                   = 41,
		BMS_BAL                      = 42,
		BMS_TEMPS                    = 43,
		BMS_HUM                      = 44,
		BMS_SOC_SOH_TEMP_STAT        = 45,
		PSW_STAT                     = 46,
		PSW_SWITCH                   = 47,
		BMS_HW_DATA_1                = 48,
		BMS_HW_DATA_2                = 49,
		BMS_HW_DATA_3                = 50,
		BMS_HW_DATA_4                = 51,
		BMS_HW_DATA_5                = 52,
		BMS_AH_WH_CHG_TOTAL          = 53,
		BMS_AH_WH_DIS_TOTAL          = 54,
		UPDATE_PID_POS_OFFSET        = 55,
		POLL_ROTOR_POS               = 56,
		NOTIFY_BOOT                  = 57,
		STATUS_6                     = 58,
		UNKNOWN
	};

  private:
	CANBus& _can;
	uint8_t _deviceId;

	uint32_t CreateId(PacketId packet)
	{
		return this->_deviceId | (uint32_t)(packet) << 8;
	}

  public:
	/**
	 * @brief Empty constructor for VESC CAN object
	 */
	VescCAN() = default;

	/**
	 * @brief Construct a new VESC CAN object
	 *
	 * @param can A reference to a CANBus interface
	 * @param controllerId 8 bit VESC controller id
	 */
	VescCAN(CANBus& can, uint8_t controllerId)
		: _can(can), _deviceId(controllerId)
	{}

	/**
	 * @brief Set the motor duty cycle
	 *
	 * @param duty The duty cycle to set (-1...1)
	 */
	void SetDutyCycle(float duty);

	/**
	 * @brief Set the motor current
	 *
	 * @param current The current to set (Amps)
	 */
	void SetCurrent(float current);

	/**
	 * @brief Set the motor braking current
	 *
	 * @param current The braking current to set (Amps)
	 */
	void SetBrakeCurrent(float current);

	/**
	 * @brief Set the motor RPM
	 *
	 * @param rpm The RPM to set
	 */
	void SetRPM(float rpm);

	/**
	 * @brief Set the motor position
	 *
	 * @param position The position to set
	 */
	void SetPosition(float position);

	/**
	 * @brief Set current relative to upper and lower current limits
	 *
	 * @param current The current relative to the limits (-1...1)
	 */
	void SetRelativeCurrent(float current);

	/**
	 * @brief Set brake current relative to upper and lower current limits
	 *
	 * @param current The brake current relative to the limits (-1...1)
	 */
	void SetRelativeBrakeCurrent(float current);

	/**
	 * @brief Set the upper and lower current limits
	 *
	 * @param lower The lower current limit (Amps)
	 * @param upper The upper current limit (Amps)
	 */
	void SetCurrentLimits(float lower, float upper);

	/**
	 * @brief Set the upper and lower current limits and store them in the EEPROM
	 *
	 * @param lower The lower current limit (Amps)
	 * @param upper The upper current limit (Amps)
	 */
	void SetCurrentLimitsAndStore(float lower, float upper);

	/**
	 * @brief Set the upper and lower input current limits
	 *
	 * @param lower The lower current limit (Amps)
	 * @param upper The upper current limit (Amps)
	 */
	void SetInputCurrentLimits(float lower, float upper);

	/**
	 * @brief Set the upper and lower input current limits and store them in the EEPROM
	 *
	 * @param lower The lower current limit (Amps)
	 * @param upper The upper current limit (Amps)
	 */
	void SetInputCurrentLimitsAndStore(float lower, float upper);

	/**
	 * @brief Test whether the received frame is a valid VESC frame and updates the packet type with the received type
	 *
	 * @param frame The received frame
	 * @param packetType The packet type of the received frame.
	 * @return bool whether the frame is a VESC frame for this controller.
	 */
	bool IsVescFrame(const CANBus::Frame& frame, PacketId& packetType);

	struct StatusMessage1
	{
		static constexpr PacketId PacketType = PacketId::STATUS;

		int32_t RPM;
		float TotalCurrentConsumed;
		float DutyCycle;
	};

	StatusMessage1 DecodeStatusMessage1(const CANBus::Frame& frame);

	struct StatusMessage2
	{
		static constexpr PacketId PacketType = PacketId::STATUS_2;

		float AmpHoursConsumed;
		float AmpHoursRegenerative;
	};

	StatusMessage2 DecodeStatusMessage2(const CANBus::Frame& frame);

	struct StatusMessage3
	{
		static constexpr PacketId PacketType = PacketId::STATUS_3;

		float WattHoursConsumed;
		float WattHoursRegenerative;
	};

	StatusMessage3 DecodeStatusMessage3(const CANBus::Frame& frame);

	struct StatusMessage4
	{
		static constexpr PacketId PacketType = PacketId::STATUS_4;

		float MosfetTemperature;
		float MotorTemperature;
		float TotalInputCurrent;
		int16_t PidPosition;
	};

	StatusMessage4 DecodeStatusMessage4(const CANBus::Frame& frame);

	struct StatusMessage5
	{
		static constexpr PacketId PacketType = PacketId::STATUS_5;

		int32_t Tachometer;
		float InputVoltage;
	};

	StatusMessage5 DecodeStatusMessage5(const CANBus::Frame& frame);
};

} // namespace PSR

#endif // End of include guard for vesc.h
