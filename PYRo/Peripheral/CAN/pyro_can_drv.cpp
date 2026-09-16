#include "pyro_can_drv.h"
#include "main.h"
#include <cstring>

namespace pyro
{
namespace
{
void abort_pending_tx(CAN_HandleTypeDef *hcan)
{
    if (nullptr == hcan)
        return;

    // bxCAN有3个发送邮箱，全部中止
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
    {
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
    }
}

status_t recover_bus_off(CAN_HandleTypeDef *hcan)
{
    // bxCAN通过BOF标志位判断bus-off
    if (0U == __HAL_CAN_GET_FLAG(hcan, CAN_FLAG_BOF))
        return PYRO_OK;

    abort_pending_tx(hcan);

    (void)HAL_CAN_Stop(hcan);
    (void)HAL_CAN_ResetError(hcan);

    if (HAL_OK != HAL_CAN_Start(hcan))
        return PYRO_ERROR;

    return PYRO_BUSY;
}
} // namespace

// ==========================================
// can_msg_buffer_t Implementation
// ==========================================
can_msg_buffer_t::can_msg_buffer_t(const uint32_t id)
    : _id(id), _buffer{}, _is_fresh(false), _last_update_time(0)
{
}

can_msg_buffer_t::~can_msg_buffer_t() = default;

uint32_t can_msg_buffer_t::get_id() const
{
    return _id;
}

bool can_msg_buffer_t::is_fresh() const
{
    return _is_fresh;
}

void can_msg_buffer_t::mark_read()
{
    _is_fresh = false;
}

TickType_t can_msg_buffer_t::get_last_update_time() const
{
    return _last_update_time;
}

void can_msg_buffer_t::update_data(const uint8_t *data)
{
    const UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    memcpy(_buffer.data(), data, 8);
    _last_update_time = xTaskGetTickCountFromISR();
    _is_fresh         = true;

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

bool can_msg_buffer_t::get_data(std::array<uint8_t, 8> &data) const
{
    taskENTER_CRITICAL();
    memcpy(data.data(), _buffer.data(), 8);
    const bool fresh_status = _is_fresh;
    taskEXIT_CRITICAL();

    return fresh_status;
}

// ==========================================
// can_drv_t Implementation
// ==========================================
can_drv_t::can_drv_t(CAN_HandleTypeDef *hcan) : _hcan(hcan), _extended_rx_callback(nullptr)
{
    _registerlist.clear();
    can_map()[hcan] = this;
}

can_drv_t::~can_drv_t()
{
    can_map().erase(_hcan);
}

map_t<CAN_HandleTypeDef *, can_drv_t *> &can_drv_t::can_map()
{
    static map_t<CAN_HandleTypeDef *, can_drv_t *> instance;
    return instance;
}

pyro::status_t can_drv_t::init() const
{
    CAN_FilterTypeDef can_filter;

    // 32位掩码模式，不过滤任何ID（标准帧+扩展帧都接收）
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation     = CAN_FILTER_ENABLE;

    // 根据是CAN1还是CAN2设置FilterBank和SlaveStartFilterBank
    if (_hcan->Instance == CAN1)
    {
        can_filter.FilterBank          = 0;
        can_filter.SlaveStartFilterBank = 14;  // CAN2从bank14开始
    }
    else
    {
        can_filter.FilterBank = 14;  // CAN2用bank14
    }

    if (HAL_OK != HAL_CAN_ConfigFilter(_hcan, &can_filter))
        return pyro::PYRO_ERROR;

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::start() const
{
    if (HAL_OK != HAL_CAN_Start(_hcan))
        return pyro::PYRO_ERROR;

    // ===== 必须手动使能FIFO0接收中断 =====
    // bxCAN的HAL_CAN_Start()不会自动使能中断，
    // 必须调用HAL_CAN_ActivateNotification置位IER寄存器的FMPIE位
    if (HAL_OK != HAL_CAN_ActivateNotification(
            _hcan, CAN_IT_RX_FIFO0_MSG_PENDING))
    {
        return pyro::PYRO_ERROR;
    }

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::send_msg(const uint32_t id, const uint8_t *data) const
{
    if (nullptr == _hcan || nullptr == data)
        return pyro::PYRO_PARAM_ERROR;

    const status_t recover_status = recover_bus_off(_hcan);
    if (PYRO_OK != recover_status)
        return recover_status;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId              = id;
    tx_header.ExtId              = 0;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox = 0;

    taskENTER_CRITICAL();
    if (0U == HAL_CAN_GetTxMailboxesFreeLevel(_hcan))
    {
        abort_pending_tx(_hcan);
        taskEXIT_CRITICAL();
        return pyro::PYRO_BUSY;
    }

    const HAL_StatusTypeDef hal_status =
        HAL_CAN_AddTxMessage(_hcan, &tx_header, const_cast<uint8_t *>(data), &tx_mailbox);

    if (HAL_OK != hal_status)
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_ERROR;
    }
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::send_extended_msg(const uint32_t id, const uint8_t *data) const
{
    if (nullptr == _hcan || nullptr == data)
        return pyro::PYRO_PARAM_ERROR;

    const status_t recover_status = recover_bus_off(_hcan);
    if (PYRO_OK != recover_status)
        return recover_status;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId              = 0;
    tx_header.ExtId              = id;
    tx_header.IDE                = CAN_ID_EXT;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox = 0;

    taskENTER_CRITICAL();
    if (0U == HAL_CAN_GetTxMailboxesFreeLevel(_hcan))
    {
        abort_pending_tx(_hcan);
        taskEXIT_CRITICAL();
        return pyro::PYRO_BUSY;
    }

    const HAL_StatusTypeDef hal_status =
        HAL_CAN_AddTxMessage(_hcan, &tx_header, const_cast<uint8_t *>(data), &tx_mailbox);

    if (HAL_OK != hal_status)
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_ERROR;
    }
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::register_rx_msg(can_msg_buffer_t *msg_buffer)
{
    const uint32_t id = msg_buffer->get_id();

    taskENTER_CRITICAL();
    if (this->_registerlist.exist(id))
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_ERROR;
    }
    this->_registerlist[id] = msg_buffer;
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::register_extended_rx_callback(extended_rx_callback_t callback)
{
    if (nullptr == callback)
        return pyro::PYRO_PARAM_ERROR;

    taskENTER_CRITICAL();
    _extended_rx_callback = callback;
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

void can_drv_t::dispatch_extended_rx(uint32_t id, const uint8_t *data) const
{
    if (_extended_rx_callback)
    {
        _extended_rx_callback(id, data);
    }
}

pyro::status_t can_drv_t::handle_rx_msg(const uint32_t id, const uint8_t *data)
{
    if (!this->_registerlist.exist(id))
    {
        return pyro::PYRO_NOT_FOUND;
    }

    can_msg_buffer_t *msg = this->_registerlist[id];
    msg->update_data(data);

    return pyro::PYRO_OK;
}

}; // namespace pyro

// ==========================================
// Global Hardware Interruption Callbacks
// ==========================================
void can_global_handle(CAN_HandleTypeDef *hcan, const uint32_t identifier,
                       const uint8_t *data)
{
    auto &map = pyro::can_drv_t::can_map();
    if (map.exist(hcan))
    {
        (void)map[hcan]->handle_rx_msg(identifier, data);
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    if (HAL_OK != HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data))
    {
        return;
    }

    auto &map = pyro::can_drv_t::can_map();
    if (!map.exist(hcan))
        return;

    if (CAN_ID_EXT == rx_header.IDE)
    {
        // 扩展帧：统一回调分发（Cybergear/达秒电机用）
        map[hcan]->dispatch_extended_rx(rx_header.ExtId, data);
    }
    else
    {
        // 标准帧：按ID查找消息缓冲（DJI电机用）
        (void)map[hcan]->handle_rx_msg(rx_header.StdId, data);
    }
}
