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
