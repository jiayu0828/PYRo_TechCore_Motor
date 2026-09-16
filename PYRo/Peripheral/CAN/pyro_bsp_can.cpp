/**
 * @file pyro_bsp_can.cpp
 * @brief Implementation of the Board Support Package for CAN.
 * CAN板级支持包实现文件。
 *
 * @author Antigravity
 * @version 1.0.0
 * @date 2026-07-03
 */

#include "pyro_bsp_can.h"
#include "can.h"

namespace pyro
{

can_drv_t& bsp_can::get_can1()
{
    static can_drv_t instance(&hcan1);
    return instance;
}

can_drv_t& bsp_can::get_can2()
{
    static can_drv_t instance(&hcan2);
    return instance;
}

can_drv_t* bsp_can::get_can(const which_can which)
{
    switch (which)
    {
        case can1:
            return &get_can1();
        case can2:
            return &get_can2();
        default:
            return nullptr;
    }
}

status_t bsp_can::init_all()
{
    // 必须先初始化CAN1，再初始化CAN2（bxCAN的CAN2依赖CAN1）
    CHECK_PYRO_RET(get_can1().init());
    CHECK_PYRO_RET(get_can2().init());

    CHECK_PYRO_RET(get_can1().start());
    CHECK_PYRO_RET(get_can2().start());
    return PYRO_OK;
}

} // namespace pyro
