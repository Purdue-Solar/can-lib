/**
 * @file can_stream.h
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief
 * @version 0.1
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CAN_STREAM_H
#define __CAN_STREAM_H

#ifdef EXPERIMENTAL_CAN_STREAM

#include "can_lib.h"
#include <functional>
#include <cstdint>
#include <vector>

#ifdef BOARD_STM32
#include "stm32_includer.h"
#include STM32_INCLUDE(BOARD_STM32, hal_crc.h)
#endif

namespace PSR
{

typedef CRC_HandleTypeDef CRCInterface;

class CANStream
{
  public:
	static constexpr size_t DefaultBlockSize = 1024;

	static constexpr uint32_t MagicId = 0x3FF;
	
	enum class StreamState : uint8_t
	{
		StartTransmission = 0,
		ReadyToTransmit = 1,
		RequestBytes = 2,
		SendBytes = 3,
		BlockFinished = 4,
		ReservedState5 = 5,
		ReservedState6 = 6,
		EndTransmission = 7
	};

  private:
	CANBus& _can;
	CRCInterface& _crc;
	uint8_t _id;

	uint32_t CreateStreamId(uint8_t src, uint8_t dst, StreamState state)
	{
		uint32_t s = src;
		uint32_t d = dst;
		uint32_t t = (int)state & 0x7;

		return (MagicId << 19) | (t << 11) | (s << 8) | d;
	}

  public:
	CANStream(CANBus& can, CRCInterface* crc, uint8_t id)
		: _can(can), _crc(*crc), _id(id)
	{}

	CANStream(CANBus& can, CRCInterface& crc, uint8_t id)
		: _can(can), _crc(crc), _id(id)
	{}

  private:
	void RequestStartTransmission(uint8_t requestNode, uint64_t resourceId);
	void RequestBytes(uint8_t requestNode, uint32_t count, uint32_t offset);
	void RequestEndTransmission(uint8_t requestNode, uint64_t resourceId);

	void RespondReadyToTransmit(uint8_t dstNode, uint32_t size);
	void RespondBytes(uint8_t dstNode, uint8_t* data, uint8_t length);
	void RespondBlockFinished(uint8_t dstNode, uint32_t sent, uint32_t checksum);


  public:
	std::vector<uint8_t>* RetrieveResource(uint8_t src, uint64_t resouceId, size_t blockSize = DefaultBlockSize);

	void RetrieveResource(
		uint8_t src,
		uint64_t resourceId,
		std::function<void(size_t, uint8_t*, size_t)> blockHandler,
		size_t blockSize = DefaultBlockSize);
};

} // namespace PSR

#endif // EXPERIMENTAL_CAN_STREAM

#endif // end of include guard for can_stream.h
