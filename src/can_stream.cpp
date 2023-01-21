/**
 * @file can_stream.cpp
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief Implementation file for streaming data over CAN
 * @version 0.1
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifdef EXPERIMENTAL_CAN_STREAM

#ifdef BOARD_STM32

#include "can_stream.h"
#include "can_lib.h"

#include <cstring>
#include <functional>
#include <vector>

namespace PSR
{

void CANStream::RequestStartTransmission(uint8_t requestNode, uint64_t resourceId)
{
	uint32_t requestId = CreateStreamId(requestNode, _id, CANStream::StreamState::StartTransmission);

	CANBus::Frame frame = {
		.Id         = requestId,
		.IsRTR      = true,
		.IsExtended = true,
		.Length     = sizeof(uint64_t),
		.Data       = { .Value = resourceId }
	};

	this->_can.Transmit(frame);
}

void CANStream::RequestBytes(uint8_t requestNode, uint32_t count, uint32_t offset)
{
	uint32_t requestId = CreateStreamId(requestNode, _id, CANStream::StreamState::RequestBytes);

	CANBus::Frame frame = {
		.Id         = requestId,
		.IsRTR      = true,
		.IsExtended = true,
		.Length     = sizeof(uint32_t) + sizeof(uint32_t),
		.Data       = { .DoubleWords = { count, offset } }
	};

	this->_can.Transmit(frame);
}

void CANStream::RequestEndTransmission(uint8_t requestNode, uint64_t resourceId)
{
	uint32_t requestId = CreateStreamId(requestNode, _id, CANStream::StreamState::EndTransmission);

	CANBus::Frame frame = {
		.Id         = requestId,
		.IsRTR      = true,
		.IsExtended = true,
		.Length     = sizeof(uint64_t),
		.Data       = { .Value = resourceId }
	};

	this->_can.Transmit(frame);
}

std::vector<uint8_t>* CANStream::RetrieveResource(uint8_t src, uint64_t resouceId, size_t blockSize)
{
	std::vector<uint8_t>* buffer = new std::vector<uint8_t>(blockSize);

	std::function<void(size_t, uint8_t*, size_t)> blockHandler = [buffer](size_t index, uint8_t* data, size_t length)
	{
		buffer->reserve(index + length);
		memcpy(buffer->data() + index, data, length);
	};

	this->RetrieveResource(src, resouceId, blockHandler, blockSize);

	return buffer;
}

void CANStream::RetrieveResource(
	uint8_t src,
	uint64_t resourceId,
	std::function<void(size_t, uint8_t*, size_t)> blockHandler,
	size_t blockSize)
{
	this->RequestStartTransmission(src, resourceId);

	uint32_t expectedBytes = 0;
	uint32_t receivedBytes = 0;

	this->RequestEndTransmission(src, resourceId);
}

} // namespace PSR

#endif // BOARD_STM32

#endif // EXPERIMENTAL_CAN_STREAM