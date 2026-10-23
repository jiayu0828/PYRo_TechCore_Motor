#include "pyro_can_drv.h"
#include "main.h"
#include "pyro_log.h"
#include "can.h"
#include <cstring>

namespace pyro
{
namespace
{
/**
 * @brief 将CAN句柄转为可读CAN名字字符串
 * @param hcan can句柄指针
 * @return "CAN1"/"CAN2"/"CAN_UNKNOWN"
 */
const char *can_to_name(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
        return "CAN1";
    if (hcan == &hcan2)
        return "CAN2";
    return "CAN_UNKNOWN";
}

/*
 * 运行期日志开关：只关心 CAN1 时把 CAN2 静默。
 * 置 false 后 CAN2 的发送丢帧 / Bus-Off 恢复日志全部不打印，
 * 计数（txbusy/txerr）仍然照常累加，调试器里依然可见。
 */
constexpr bool CAN_LOG_ENABLE_CAN2 = false;

/**
 * @brief 该 CAN 实例是否需要打印运行期日志
 */
bool can_log_enabled(const char *can_name)
{
    return (0 != strcmp(can_name, "CAN2")) || CAN_LOG_ENABLE_CAN2;
}

/**
 * @brief 配置"全通"过滤器：32 位掩码模式，掩码全 0，标准帧和扩展帧都接收
 *
 * init() 与 set_mode() 都会调用，保证切换模式后接收路径一致。
 */
pyro::status_t apply_accept_all_filter(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter{};

    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation     = CAN_FILTER_ENABLE;

    // CAN2 的过滤器从 bank14 开始，CAN1 从 bank0 开始
    if (hcan->Instance == CAN1)
    {
        can_filter.FilterBank           = 0;
        can_filter.SlaveStartFilterBank = 14;
    }
    else
    {
        can_filter.FilterBank = 14;
    }

    return (HAL_OK == HAL_CAN_ConfigFilter(hcan, &can_filter))
               ? pyro::PYRO_OK
               : pyro::PYRO_ERROR;
}

/* 发送失败边沿状态：仅在"正常→丢帧"和"丢帧→正常"两个时刻各打印一次 */
volatile uint32_t g_can_tx_busy_cnt  = 0;
volatile uint32_t g_can_tx_error_cnt = 0;

struct tx_fail_state_t
{
    bool     dropping;     /* 已确认处于丢帧状态（对外可见） */
    uint32_t dropped;      /* 本次丢帧期间累计丢弃帧数 */
    uint32_t since_tick;   /* 本次丢帧开始时刻 */
    uint32_t first_fail_tick; /* 本轮首次失败时刻，用于去抖 */
};

/*
 * 去抖窗口：首次失败后必须持续失败超过该时长才上报。
 * 没接电机时帧无人 ACK，硬件反复重传占满邮箱，abort_pending_tx() 清空后
 * 下一帧又能入队，导致"丢-恢复"逐帧翻转；只有持续失败才算真实故障。
 */
constexpr uint32_t TX_FAIL_DEBOUNCE_MS = 200U;

tx_fail_state_t s_tx_fail[2]; /* [0]=CAN1, [1]=CAN2 */

/**
 * @brief 取该 CAN 实例对应的边沿状态
 */
tx_fail_state_t &tx_fail_state_of(const char *can_name)
{
    return (0 == strcmp(can_name, "CAN2")) ? s_tx_fail[1] : s_tx_fail[0];
}

/**
 * @brief 发送失败上报（边沿触发）
 *
 * send_msg 运行在 1kHz 控制周期上，若每次失败都打印会把日志通道打满。
 * 这里只在状态翻转时打印：
 *   - 首次失败：WARN "tx start dropping"，附带失败原因与帧 ID
 *   - 恢复成功：INFO "tx recovered"，附带本次丢帧总数与持续时长
 * 丢帧期间只累加计数，不产生任何日志。
 *
 * 注意：该函数可能在中断上下文被调用，只做计数与状态判断，
 * 真正的日志输出由 elog 的互斥锁保护。
 */
void can_log_tx_fail(const char *can_name, uint32_t id, const char *reason)
{
    tx_fail_state_t &st = tx_fail_state_of(can_name);

    st.dropped++;
    if (!can_log_enabled(can_name))
        return;

    if (st.dropping)
        return;

    const uint32_t now = HAL_GetTick();
    if (0U == st.first_fail_tick)
    {
        st.first_fail_tick = now;
        return;
    }

    /* 持续失败未达去抖窗口，暂不上报 */
    if ((now - st.first_fail_tick) < TX_FAIL_DEBOUNCE_MS)
        return;

    st.dropping   = true;
    st.since_tick = st.first_fail_tick;

    PYRO_LOGW("CAN", "%s tx start dropping, %s, first id=0x%03lX",
              can_name, reason, (unsigned long)id);
}

/**
 * @brief 发送成功上报（边沿触发）
 *
 * 由 send_msg / send_extended_msg 在成功入队后调用。
 * 只有从丢帧状态恢复时才打印一条 INFO，正常发送时不产生日志。
 */
void can_log_tx_ok(const char *can_name)
{
    tx_fail_state_t &st = tx_fail_state_of(can_name);

    /* 成功即清零本轮失败计时，抖动不会累积 */
    st.first_fail_tick = 0;

    if (!can_log_enabled(can_name))
    {
        st.dropping = false;
        st.dropped  = 0;
        return;
    }

    if (!st.dropping)
        return;

    const uint32_t elapsed = HAL_GetTick() - st.since_tick;
    PYRO_LOGI("CAN", "%s tx recovered, dropped %lu frames in %lu ms",
              can_name, (unsigned long)st.dropped, (unsigned long)elapsed);

    st.dropping = false;
    st.dropped  = 0;
}

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

    const char *can_name = can_to_name(hcan);
    const bool log_on = can_log_enabled(can_name);
    if (log_on)
    {
        PYRO_LOGW("CAN", "%s Bus-Off detected, try auto-recover", can_name);
    }

    abort_pending_tx(hcan);

    (void)HAL_CAN_Stop(hcan);
    (void)HAL_CAN_ResetError(hcan);

    if (HAL_OK != HAL_CAN_Start(hcan))
    {
        if (log_on)
        {
            PYRO_LOGE("CAN", "%s auto-recover start failed", can_name);
        }
        return PYRO_ERROR;
    }

    if (log_on)
    {
        PYRO_LOGI("CAN", "%s auto-recover Bus-Off success", can_name);
    }
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
    const char *can_name = can_to_name(_hcan);

    if (PYRO_OK != apply_accept_all_filter(_hcan))
    {
        if (can_log_enabled(can_name))
        {
            PYRO_LOGE("CAN", "%s config filter fail", can_name);
        }
        return pyro::PYRO_ERROR;
    }

    if (can_log_enabled(can_name))
    {
        PYRO_LOGI("CAN", "%s init ok", can_name);
    }
    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::start() const
{
    const char *can_name = can_to_name(_hcan);
    const bool log_on    = can_log_enabled(can_name);

    if (HAL_OK != HAL_CAN_Start(_hcan))
    {
        if (log_on)
        {
            PYRO_LOGE("CAN", "%s start failed", can_name);
        }
        return pyro::PYRO_ERROR;
    }

    // ===== 必须手动使能FIFO0接收中断 =====
    // bxCAN的HAL_CAN_Start()不会自动使能中断，
    // 必须调用HAL_CAN_ActivateNotification置位IER寄存器的FMPIE位
    if (HAL_OK != HAL_CAN_ActivateNotification(
            _hcan, CAN_IT_RX_FIFO0_MSG_PENDING))
    {
        if (log_on)
        {
            PYRO_LOGE("CAN", "%s activate notification failed", can_name);
        }
        return pyro::PYRO_ERROR;
    }

    if (log_on)
    {
        PYRO_LOGI("CAN", "%s start ok", can_name);
    }
    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::set_mode(const uint32_t mode)
{
    if (nullptr == _hcan)
        return pyro::PYRO_PARAM_ERROR;

    if (_hcan->Init.Mode == mode)
        return pyro::PYRO_OK;

    const char *can_name = can_to_name(_hcan);
    const bool log_on    = can_log_enabled(can_name);

    if (HAL_CAN_STATE_LISTENING == _hcan->State)
    {
        if (HAL_OK != HAL_CAN_Stop(_hcan))
        {
            if (log_on)
            {
                PYRO_LOGE("CAN", "%s stop for mode switch failed", can_name);
            }
            return pyro::PYRO_ERROR;
        }
    }

    /* HAL_CAN_Init 会把 Init.Mode 直接写进 BTR 的 LBKM/SILM 位 */
    _hcan->Init.Mode = mode;

    if (HAL_OK != HAL_CAN_Init(_hcan))
    {
        if (log_on)
        {
            PYRO_LOGE("CAN", "%s re-init (mode=0x%lX) failed", can_name,
                      (unsigned long)mode);
        }
        return pyro::PYRO_ERROR;
    }

    /* 过滤器在 Init 后仍然保留，这里重新写一次以确保与 init() 一致 */
    if (PYRO_OK != apply_accept_all_filter(_hcan))
        return pyro::PYRO_ERROR;

    if (HAL_OK != HAL_CAN_Start(_hcan))
        return pyro::PYRO_ERROR;

    /* bxCAN 的 HAL_CAN_Start 不会自动开接收中断，必须手动激活 */
    if (HAL_OK != HAL_CAN_ActivateNotification(_hcan,
                                              CAN_IT_RX_FIFO0_MSG_PENDING))
        return pyro::PYRO_ERROR;

    return pyro::PYRO_OK;
}

uint32_t can_drv_t::get_mode() const
{
    return (nullptr == _hcan) ? CAN_MODE_NORMAL : _hcan->Init.Mode;
}

bool can_drv_t::is_loopback() const
{
    return 0U != (get_mode() & CAN_BTR_LBKM);
}

pyro::status_t can_drv_t::unregister_rx_msg(can_msg_buffer_t *msg_buffer)
{
    if (nullptr == msg_buffer)
        return pyro::PYRO_PARAM_ERROR;

    const uint32_t id = msg_buffer->get_id();

    taskENTER_CRITICAL();
    if (!this->_registerlist.exist(id))
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_NOT_FOUND;
    }
    this->_registerlist.erase(id);
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::run_loopback_selftest(
    const uint32_t timeout_ms, can_selftest_result_t *out)
{
    if ((nullptr == _hcan) || (nullptr == out))
        return pyro::PYRO_PARAM_ERROR;

    /* 0x7FF 是标准帧最大 ID，工程内没有电机使用，避免与已有接收冲突 */
    static constexpr uint32_t SELFTEST_STD_ID = 0x7FF;
    static const uint8_t payload[8] = {0xA5, 0x5A, 0x01, 0x02,
                                       0x03, 0x04, 0x05, 0x06};

    const char *can_name    = can_to_name(_hcan);
    const uint32_t prev_mode = _hcan->Init.Mode;

    *out = can_selftest_result_t{};

    if (PYRO_OK != set_mode(CAN_MODE_LOOPBACK))
    {
        PYRO_LOGE("CAN", "%s selftest: enter loopback FAILED", can_name);
        (void)set_mode(prev_mode);
        return pyro::PYRO_ERROR;
    }

    can_msg_buffer_t rx(SELFTEST_STD_ID);
    const bool registered = (PYRO_OK == register_rx_msg(&rx));

    if (registered)
    {
        /*
         * 从 Normal 切过来时可能残留 Bus-Off：bxCAN 需要连续检测到
         * 128 次 11 位隐性电平才退出，约 1.4ms@1Mbps。这里先静置再发，
         * 否则 send_msg 里的 recover_bus_off 会把恢复流程反复重启。
         */
        vTaskDelay(pdMS_TO_TICKS(5));

        const uint32_t t0 = HAL_GetTick();

        /* 邮箱里可能残留上一轮的报文，重试几次；recover_bus_off 也可能吞掉一帧 */
        for (uint8_t attempt = 0U; attempt < 10U; ++attempt)
        {
            const status_t st = send_msg(SELFTEST_STD_ID, payload);
            out->tx_ret        = static_cast<int>(st);
            if (PYRO_OK == st)
            {
                out->tx_ok = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        if (out->tx_ok)
        {
            while ((HAL_GetTick() - t0) <= timeout_ms)
            {
                if (rx.is_fresh())
                {
                    std::array<uint8_t, 8> data{};
                    if (rx.get_data(data))
                    {
                        out->rx_ok   = true;
                        out->data_ok = (0 == memcmp(data.data(), payload, 8));
                    }
                    rx.mark_read();
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        out->elapsed_ms = HAL_GetTick() - t0;
        (void)unregister_rx_msg(&rx);
    }

    out->esr = _hcan->Instance->ESR;

    (void)set_mode(prev_mode);
    abort_pending_tx(_hcan);

    return registered ? pyro::PYRO_OK : pyro::PYRO_ERROR;
}

pyro::status_t can_drv_t::send_msg(const uint32_t id, const uint8_t *data) const
{
    if (nullptr == _hcan || nullptr == data)
    {
        PYRO_LOGE("CAN", "send_msg invalid param, id=0x%03lX",
                  (unsigned long)id);
        return pyro::PYRO_PARAM_ERROR;
    }

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
        g_can_tx_busy_cnt++;
        /* 高频路径：边沿触发，只在开始丢帧时打印一次 */
        can_log_tx_fail(can_to_name(_hcan), id, "tx mailbox full");
        return pyro::PYRO_BUSY;
    }

    const HAL_StatusTypeDef hal_status =
        HAL_CAN_AddTxMessage(_hcan, &tx_header, const_cast<uint8_t *>(data), &tx_mailbox);

    if (HAL_OK != hal_status)
    {
        taskEXIT_CRITICAL();
        g_can_tx_error_cnt++;
        can_log_tx_fail(can_to_name(_hcan), id, "HAL_CAN_AddTxMessage failed");
        return pyro::PYRO_ERROR;
    }
    taskEXIT_CRITICAL();

    can_log_tx_ok(can_to_name(_hcan));
    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::send_extended_msg(const uint32_t id, const uint8_t *data) const
{
    if (nullptr == _hcan || nullptr == data)
    {
        PYRO_LOGE("CAN", "send_extended_msg invalid param, id=0x%08lX",
                  (unsigned long)id);
        return pyro::PYRO_PARAM_ERROR;
    }

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
        g_can_tx_busy_cnt++;
        can_log_tx_fail(can_to_name(_hcan), id, "tx mailbox full (ext)");
        return pyro::PYRO_BUSY;
    }

    const HAL_StatusTypeDef hal_status =
        HAL_CAN_AddTxMessage(_hcan, &tx_header, const_cast<uint8_t *>(data), &tx_mailbox);

    if (HAL_OK != hal_status)
    {
        taskEXIT_CRITICAL();
        g_can_tx_error_cnt++;
        can_log_tx_fail(can_to_name(_hcan), id,
                        "HAL_CAN_AddTxMessage failed (ext)");
        return pyro::PYRO_ERROR;
    }
    taskEXIT_CRITICAL();

    can_log_tx_ok(can_to_name(_hcan));
    return pyro::PYRO_OK;
}

/* 发送失败统计查询（供上层周期性上报） */
extern "C" uint32_t can_get_tx_busy_cnt(void)
{
    return g_can_tx_busy_cnt;
}

extern "C" uint32_t can_get_tx_error_cnt(void)
{
    return g_can_tx_error_cnt;
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
