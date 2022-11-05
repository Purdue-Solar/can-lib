/**
 * @file stm32f4xx.c
 * @author Purdue Solar Racing (Aidan Orr)
 * @brief
 * @version 0.1
 * @date 2022-09-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#if defined(BOARD_STM32F)

#include "can_lib.h"

namespace PSR
{

CANBus::CANBus(CANBus::Interface& interface, const CANBus::Config& config)
    : _interface(interface), _config(config), _rxCallback(NULL)
{
}

void CANBus::Init()
{
    CAN_FilterTypeDef filter;

    // Config filter ranges
    filter.FilterMaskIdLow  = (uint16_t)this->_config.FilterMask;
    filter.FilterMaskIdHigh = (uint16_t)(this->_config.FilterMask >> 16);
    filter.FilterMode       = CAN_FILTERMODE_IDLIST;

    // Config filter banks
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;

    // TODO: Fully understand filter setup
    // HAL_CAN_ConfigFilter(this->_interface, &filter);

    this->_interface.Init.AutoRetransmission = this->_config.AutoRetransmit ? ENABLE : DISABLE;
    HAL_CAN_Start(&this->_interface);
}

CANBus::TransmitStatus CANBus::Transmit(const Frame& frame)
{
    CAN_TxHeaderTypeDef txHeader;

    while (HAL_CAN_IsTxMessagePending(&this->_interface, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2))
    {
    }
    HAL_Delay(1);

    txHeader.ExtId = frame.IsExtended ? frame.Id & CANBus::EXT_ID_MASK : 0;
    txHeader.StdId = frame.IsExtended ? 0 : frame.Id & CANBus::STD_ID_MASK;
    txHeader.IDE   = frame.IsExtended ? CAN_ID_EXT : CAN_ID_STD;
    txHeader.DLC   = frame.Length;
    txHeader.RTR   = frame.IsRTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

    uint32_t mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&this->_interface, &txHeader, (uint8_t*)frame.Data.Bytes, &mailbox);

    switch (status)
    {
    case HAL_OK:
        return CANBus::Success;
    case HAL_ERROR:
        return CANBus::Error;
    default:
        return CANBus::Unknown;
    }
}

// Callback CAN object storage
static const int _storageSize = 4;

static struct
{
    CANBus::Interface* interface;
    CANBus::Callback callback;
} _canRxCallbackCANObjects[_storageSize] = {
    {NULL, NULL},
    {NULL, NULL},
    {NULL, NULL},
    {NULL, NULL}
};

void CANBus::SetRxCallback(CANBus::Callback callback)
{
    this->_rxCallback = callback;

    // Fill first spot in storage with current CAN object pointer
    for (int i = 0; i < _storageSize; ++i)
    {
        if (_canRxCallbackCANObjects[i].interface == NULL)
        {
            _canRxCallbackCANObjects[i] = { &this->_interface, callback };
            break;
        }
    }

    // Activate interrupt handler
    HAL_CAN_ActivateNotification(&this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CANBus::ClearRxCallback()
{
    this->_rxCallback = NULL;

    // Remove all instances of current CAN object from storage
    bool anyNotThis = false;
    for (int i = 0; i < _storageSize; ++i)
    {
        if (_canRxCallbackCANObjects[i].interface == &this->_interface)
        {
            _canRxCallbackCANObjects[i] = { NULL, NULL };
        }
        else if (_canRxCallbackCANObjects[i].interface != NULL)
        {
            anyNotThis = true;
        }
    }

    // Deactivate interrupt handler
    if (!anyNotThis)
        HAL_CAN_DeactivateNotification(&this->_interface, CAN_IT_RX_FIFO0_MSG_PENDING);
}

} // namespace PSR

extern "C"
{
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
    {
        CAN_RxHeaderTypeDef rxHeader;
        PSR::CANBus::Frame frame;
        PSR::CANBus::Callback rxCallback;

        for (int i = 0; i < PSR::_storageSize; ++i)
        {
            if (PSR::_canRxCallbackCANObjects[i].interface == hcan)
            {
                rxCallback = PSR::_canRxCallbackCANObjects[i].callback;
                break;
            }
        }
        if (rxCallback != NULL)
        {
            // Receive all available frames
            while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
            {
                HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, frame.Data.Bytes);

                if (status == HAL_OK)
                {
                    bool isExtended  = rxHeader.IDE == CAN_ID_EXT;
                    frame.Id         = isExtended ? rxHeader.ExtId : rxHeader.StdId;
                    frame.Length     = rxHeader.DLC;
                    frame.IsRTR      = rxHeader.RTR == CAN_RTR_REMOTE;
                    frame.IsExtended = isExtended;

                    (*rxCallback)(frame);
                }
            }
        }
    }
}

#endif // defined(BOARD_STM32F)
