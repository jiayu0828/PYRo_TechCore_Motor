#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dwt_drv.h"
#include "pyro_cybergear_motor_drv.h"
#include "FreeRTOS.h"
#include "task.h"

namespace pyro{
    can_drv_t *can1_drv = nullptr;
    can_drv_t *can2_drv = nullptr;

    extern "C"{
    void pyro_init_thread(void *argument){

        // 初始化DWT驱动（F407主频168MHz）
        dwt_drv_t::init(168);

        // 初始化UART驱动
        bsp_uart::init_all();

        // 初始化CAN驱动（F407只有CAN1/CAN2，无CAN3）
        can1_drv = &bsp_can::get_can1();
        can2_drv = &bsp_can::get_can2();

        // 必须先初始化CAN1，再初始化CAN2（bxCAN的CAN2依赖CAN1）
        // 用volatile变量保存返回值，方便调试器观察
        volatile status_t ret_can1_init  = can1_drv->init();
        volatile status_t ret_can2_init  = can2_drv->init();
        volatile status_t ret_can1_start = can1_drv->start();
        volatile status_t ret_can2_start = can2_drv->start();
        // 在这里加断点：4个返回值都应为0(PYRO_OK)
        (void)ret_can1_init;
        (void)ret_can2_init;
        (void)ret_can1_start;
        (void)ret_can2_start;

        // ===== 注册Cybergear扩展帧接收回调 =====
        // 根据实际接线选择CAN总线
        can1_drv->register_extended_rx_callback(
            cybergear_motor_drv_t::extended_rx_callback);

        // 如果电机接在CAN2上，取消对应注释：
        // can2_drv->register_extended_rx_callback(
        //     cybergear_motor_drv_t::extended_rx_callback);

        // 删除初始化任务
        vTaskDelete(nullptr);
    }
    }
}
