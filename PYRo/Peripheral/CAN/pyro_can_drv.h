#ifndef CAN_DRV_H
#define CAN_DRV_H

#include <array>
#include <cstdint>
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "can.h"
#include "map.h"

namespace pyro
{
class can_msg_buffer_t
{
public:
    explicit can_msg_buffer_t(uint32_t id);
    ~can_msg_buffer_t();

    [[nodiscard]] uint32_t get_id() const;
    [[nodiscard]] bool is_fresh() const;
    void mark_read();
    void update_data(const uint8_t *data);
    [[nodiscard]] bool get_data(std::array<uint8_t, 8> &data) const;
    [[nodiscard]] TickType_t get_last_update_time() const;

private:
    uint32_t _id;
    std::array<uint8_t, 8> _buffer;
    volatile bool _is_fresh;
    volatile TickType_t _last_update_time;
};

class bsp_can;

// 扩展帧接收回调函数类型
using extended_rx_callback_t = void (*)(uint32_t id, const uint8_t *data);

/**
 * @brief CAN 回环自检结果
 *
 * 在 LOOPBACK 模式下，bxCAN 把发出的帧在芯片内部环回给自己，
 * 不依赖外部节点应答。因此：
 *   - 自检通过 => MCU 侧（外设/驱动/位时序/过滤器）正常，问题在外部接线或电机
 *   - 自检失败 => MCU 侧配置有问题，先修软件
 */
struct can_selftest_result_t
{
    bool     tx_ok;      ///< 测试帧是否成功写入发送邮箱
    bool     rx_ok;      ///< 是否收到环回的测试帧
    bool     data_ok;    ///< 收到的数据是否与发送一致
    int      tx_ret;     ///< send_msg 的原始返回值
    uint32_t esr;        ///< 失败时读到的 ESR（含 TEC/REC/LEC），便于诊断
    uint32_t elapsed_ms; ///< 从发送到收到（或超时）的耗时
};

class can_drv_t
{
    friend class bsp_can;
    const uint8_t MAX_ID_REGIST_NUM = 32;
    using can_id_regist_t           = uint16_t;

public:
    ~can_drv_t();

    status_t init() const;
    status_t start() const;

    // 标准帧发送（DJI电机用）
    status_t send_msg(uint32_t id, const uint8_t *data) const;

    // 扩展帧发送（达秒/Cybergear电机用）
    status_t send_extended_msg(uint32_t id, const uint8_t *data) const;

    // 标准帧接收注册（按ID查找，DJI电机用）
    status_t register_rx_msg(can_msg_buffer_t *msg_buffer);

    // 扩展帧接收回调注册（所有扩展帧统一回调，Cybergear/达秒电机用）
    status_t register_extended_rx_callback(extended_rx_callback_t callback);

    // 注销标准帧接收缓冲
    status_t unregister_rx_msg(can_msg_buffer_t *msg_buffer);

    /**
     * @brief 切换 CAN 工作模式
     * @param mode CAN_MODE_NORMAL / CAN_MODE_LOOPBACK / CAN_MODE_SILENT_LOOPBACK
     * @note 会 Stop -> 修改 Init.Mode -> Init -> Start，期间该路通信短暂中断
     */
    status_t set_mode(uint32_t mode);

    [[nodiscard]] uint32_t get_mode() const;
    [[nodiscard]] bool is_loopback() const;

    /**
     * @brief 回环自检：临时切到 LOOPBACK，自发一帧并验证能否收回，结束后恢复原模式
     * @param timeout_ms 等待环回帧的超时时间
     * @param out 自检结果
     * @return PYRO_OK 表示自检流程正常完成（结果看 out）；其他表示流程本身失败
     * @note 仅可在任务上下文调用（内部有 vTaskDelay）
     */
    status_t run_loopback_selftest(uint32_t timeout_ms,
                                   can_selftest_result_t *out);

    status_t handle_rx_msg(uint32_t id, const uint8_t *data);

    static map_t<CAN_HandleTypeDef *, can_drv_t *> &can_map();

    // 供中断回调调用的扩展帧分发
    void dispatch_extended_rx(uint32_t id, const uint8_t *data) const;

private:
    explicit can_drv_t(CAN_HandleTypeDef *hcan);

    CAN_HandleTypeDef *_hcan;
    map_t<uint32_t, can_msg_buffer_t *> _registerlist;

    // 扩展帧接收回调
    extended_rx_callback_t _extended_rx_callback;

};

}

#endif
