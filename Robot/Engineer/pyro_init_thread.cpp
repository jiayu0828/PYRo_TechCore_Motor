#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dwt_drv.h"
#include "pyro_cybergear_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"
#include "FreeRTOS.h"
#include "task.h"

#include "techcore_command.h"
#include "techcore_global.h"
#include "move_switch.h"
#include "move_control.h"

using namespace pyro;

// ============================================================
// 小米Cybergear电机ID配置(硬编码，修改只需改这里)
// ============================================================
static constexpr uint8_t CYBERGEAR_YAW_ID    = 1;
static constexpr uint8_t CYBERGEAR_PITCH_ID  = 2;
static constexpr uint8_t CYBERGEAR_ROLL_ID   = 3;
static constexpr uint8_t CYBERGEAR_MASTER_ID = 0x00;

// ============================================================
// 全局对象定义(在init任务中用new创建，保证初始化顺序可控)
// ============================================================
move_switch_t*          g_move_switch   = nullptr;

dji_m3508_motor_drv_t*  g_x_motor       = nullptr;
dji_m3508_motor_drv_t*  g_z_motor       = nullptr;
pyro::pid_t*            g_x_pos_pid     = nullptr;
pyro::pid_t*            g_x_rot_pid     = nullptr;
pyro::pid_t*            g_z_pos_pid     = nullptr;
pyro::pid_t*            g_z_rot_pid     = nullptr;
move_control_t*         g_x_axis_ctrl   = nullptr;
move_control_t*         g_z_axis_ctrl   = nullptr;

pyro::pid_t*            g_yaw_pos_pid   = nullptr;
pyro::pid_t*            g_yaw_rot_pid   = nullptr;
pyro::pid_t*            g_pitch_pos_pid = nullptr;
pyro::pid_t*            g_pitch_rot_pid = nullptr;
pyro::pid_t*            g_roll_pos_pid  = nullptr;
pyro::pid_t*            g_roll_rot_pid  = nullptr;
cybergear_motor_drv_t*  g_yaw_motor     = nullptr;
cybergear_motor_drv_t*  g_pitch_motor   = nullptr;
cybergear_motor_drv_t*  g_roll_motor    = nullptr;

volatile bool g_techcore_init_done = false;

// 键盘通信接收命令
command_recv_t cmd_recv = {0};

namespace pyro{
    can_drv_t *can1_drv = nullptr;
    can_drv_t *can2_drv = nullptr;

    extern "C"{
    void pyro_init_thread(void *argument){
        (void)argument;

        // ============================================================
        // 1. 底层外设初始化
        // ============================================================

        // DWT驱动（F407主频168MHz）
        dwt_drv_t::init(168);

        // UART驱动（含USART1蓝牙指令通信的DMA接收）
        bsp_uart::init_all();

        // 注册USART1蓝牙指令接收回调
        bsp_uart::get_uart1().add_rx_event_callback(
            [](uint8_t *p, uint16_t size, BaseType_t& xHigherPriorityTaskWoken) -> bool {
                (void)xHigherPriorityTaskWoken;
                command_recv_process(&cmd_recv, p, size);
                return true;
            },
            1 // owner ID
        );

        // CAN驱动（F407只有CAN1/CAN2）
        can1_drv = &bsp_can::get_can1();
        can2_drv = &bsp_can::get_can2();

        // 必须先初始化CAN1，再初始化CAN2（bxCAN的CAN2依赖CAN1）
        volatile status_t ret_can1_init  = can1_drv->init();
        volatile status_t ret_can2_init  = can2_drv->init();
        volatile status_t ret_can1_start = can1_drv->start();
        volatile status_t ret_can2_start = can2_drv->start();
        (void)ret_can1_init; (void)ret_can2_init;
        (void)ret_can1_start; (void)ret_can2_start;

        // 注册Cybergear扩展帧接收回调（小米电机反馈）
        can1_drv->register_extended_rx_callback(
            cybergear_motor_drv_t::extended_rx_callback);

        // ============================================================
        // 2. 限位开关
        // ============================================================
        g_move_switch = new move_switch_t();

        // ============================================================
        // 3. X/Z平移轴 - 大疆M3508 (CAN2)
        // ============================================================
        // X轴
        g_x_motor   = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can2);
        g_x_pos_pid = new pyro::pid_t(2.0f, 0.0f, 0.0f, 0.0f, 150.0f);
        g_x_rot_pid = new pyro::pid_t(0.05f, 0.0f, 0.0f, 0.0f, 20.0f);
        g_x_axis_ctrl = new move_control_t(g_x_motor, g_x_pos_pid, g_x_rot_pid);
        g_x_axis_ctrl->set_perimeter(3.0f);
        g_x_axis_ctrl->set_direction(-1.0f);

        // Z轴
        g_z_motor   = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can2);
        g_z_pos_pid = new pyro::pid_t(2.0f, 0.0f, 0.0f, 0.0f, 150.0f);
        g_z_rot_pid = new pyro::pid_t(0.07f, 0.01f, 0.0f, 8.4f, 20.0f);
        g_z_axis_ctrl = new move_control_t(g_z_motor, g_z_pos_pid, g_z_rot_pid);
        g_z_axis_ctrl->set_perimeter(3.0f);
        g_z_axis_ctrl->set_direction(-1.0f);
        g_z_axis_ctrl->set_gravity_offset(0.0f);

        // ============================================================
        // 4. YPR旋转轴 - 小米Cybergear (CAN1)
        // ============================================================
        g_yaw_pos_pid   = new pyro::pid_t(2.5f, 0.0f, 0.6f, 0.2f, 3.0f,30.0f,1,0,0,0);
        g_yaw_rot_pid   = new pyro::pid_t(2.0f, 0.01f, 0.0f, 1.0f, 12.0f,20.0f,1,0,0,0);//加上力矩项的低通滤波
        g_pitch_pos_pid = new pyro::pid_t(1.5f, 0.0f, 0.8f, 0.2f, 3.0f,30.0f,1,0,0,0);
        g_pitch_rot_pid = new pyro::pid_t(6.0f, 0.0f, 0.0f, 1.0f, 12.0f,20.0f,1,0,0,0);
        g_roll_pos_pid  = new pyro::pid_t(1.5f, 0.0f, 0.8f, 0.2f, 3.0f,30.0f,1,0,0,0);
        g_roll_rot_pid  = new pyro::pid_t(6.0f, 0.0f, 0.0f, 1.0f, 12.0f,20.0f,1,0,0,0);

        g_yaw_motor   = new cybergear_motor_drv_t(CYBERGEAR_YAW_ID,   CYBERGEAR_MASTER_ID, bsp_can::can1);
        g_pitch_motor = new cybergear_motor_drv_t(CYBERGEAR_PITCH_ID, CYBERGEAR_MASTER_ID, bsp_can::can1);
        g_roll_motor  = new cybergear_motor_drv_t(CYBERGEAR_ROLL_ID,  CYBERGEAR_MASTER_ID, bsp_can::can1);

        // ============================================================
        // 5. 使能所有电机，检查返回值
        // ============================================================
        volatile status_t ret;

        vTaskDelay(pdMS_TO_TICKS(3000)); // 等待电机总线稳定

        ret = g_yaw_motor->enable();
        // 可在此处加断点检查 ret == PYRO_OK
        (void)ret;
        vTaskDelay(pdMS_TO_TICKS(10));

        ret = g_pitch_motor->enable();
        (void)ret;
        vTaskDelay(pdMS_TO_TICKS(10));

        ret = g_roll_motor->enable();
        (void)ret;
        vTaskDelay(pdMS_TO_TICKS(10));

        // 注意: 不在电机内部设置力矩限制(set_torque_limit)，
        // 完全由软件输出端限幅 + 堵转保护来保证安全

        ret = g_x_motor->enable();
        (void)ret;
        vTaskDelay(pdMS_TO_TICKS(5));

        ret = g_z_motor->enable();
        (void)ret;
        vTaskDelay(pdMS_TO_TICKS(5));

        // ============================================================
        // 6. 初始化完成，置标志位
        // ============================================================
        g_techcore_init_done = true;

        // 删除初始化任务
        vTaskDelete(nullptr);
    }
    }
}
