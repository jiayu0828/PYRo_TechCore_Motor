/**
 * @file cybergear_scan_task.cpp
 * @brief Cybergear电机ID扫描任务
 *
 * 扫描CAN总线上的Cybergear电机，发现未知ID的电机。
 * 扫描结果保存在全局数组 g_cybergear_found_ids[] 中，
 * 可通过调试器查看，或在其他任务中引用。
 */

#include "pyro_cybergear_motor_drv.h"
#include "pyro_bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>

using namespace pyro;

// ============================================================
// 扫描结果（全局，方便调试器查看和其他任务引用）
// ============================================================
#define CYBERGEAR_MAX_MOTORS 8

volatile uint8_t  g_cybergear_found_ids[CYBERGEAR_MAX_MOTORS] = {0};
volatile uint8_t  g_cybergear_found_count = 0;
volatile bool     g_cybergear_scan_done = false;

// ============================================================
// 扫描配置（按需修改）
// ============================================================
static constexpr bsp_can::which_can SCAN_CAN      = bsp_can::can1;  // 扫描哪一路CAN
static constexpr uint8_t             SCAN_MASTER  = 0x00;            // 主机ID
static constexpr uint8_t             SCAN_START   = 0x01;            // 扫描起始ID
static constexpr uint8_t             SCAN_END     = 0xFF;            // 扫描结束ID
static constexpr uint32_t            SCAN_TIMEOUT = 10;               // 每个ID超时(ms)


// ============================================================
// 控制指令（调试器可修改）
// ============================================================
volatile float   g_cybergear_cmd_torque   = 0.0f;   ///< 前馈扭矩 (-12~12 Nm)
volatile float   g_cybergear_cmd_position = 0.0f;   ///< 目标位置 (-4π~4π rad)
volatile float   g_cybergear_cmd_velocity = 0.0f;   ///< 目标速度 (-30~30 rad/s)
volatile float   g_cybergear_cmd_kp       = 0.0f;   ///< 位置环Kp (0~500)
volatile float   g_cybergear_cmd_kd       = 0.0f;   ///< 位置环Kd (0~5)
volatile uint8_t g_cybergear_enable_cmd   = 1;      ///< 1=使能, 0=失能
volatile uint8_t g_cybergear_new_id       = 1;      ///< 设为非0触发更改ID，完成后自动清零

// ============================================================
// 反馈数据（调试器可查看）
// ============================================================
volatile float    g_cybergear_fb_position  = 0.0f;  ///< 当前位置 (rad)
volatile float    g_cybergear_fb_velocity  = 0.0f;  ///< 当前速度 (rad/s)
volatile float    g_cybergear_fb_torque    = 0.0f;  ///< 当前扭矩 (Nm)
volatile int8_t   g_cybergear_fb_temperature = 0;   ///< 当前温度 (°C)
volatile uint8_t  g_cybergear_fb_fault     = 0;     ///< 故障码 (见fault_code_t)
volatile uint8_t  g_cybergear_fb_mode_state = 0;    ///< 模式状态 (0=复位,1=标定,2=运行)
volatile float    g_cybergear_fb_bus_voltage = 0.0f;///< 母线电压 (V)
volatile bool     g_cybergear_fb_online     = false; ///< 电机在线状态
volatile uint8_t  g_cybergear_current_id     = 0;    ///< 当前电机ID

// ============================================================
// 扫描任务入口
// ============================================================
extern "C" void cybergear_scan_task(void *argument)
{
    (void)argument;

    // 等待CAN驱动初始化完成（pyro_init_thread中初始化）
    vTaskDelay(pdMS_TO_TICKS(1500));

    g_cybergear_scan_done = false;
    g_cybergear_found_count = 0;

    // 固定大小的局部数组，避免动态内存分配
    uint8_t found_ids[CYBERGEAR_MAX_MOTORS] = {0};

    // 执行扫描
    uint8_t found_count = cybergear_motor_drv_t::scan_bus(
        SCAN_CAN,
        found_ids,
        CYBERGEAR_MAX_MOTORS,
        SCAN_MASTER,
        SCAN_START,
        SCAN_END,
        SCAN_TIMEOUT
    );

    // 保存结果到全局数组
    g_cybergear_found_count = found_count;
    for (uint8_t i = 0; i < found_count; i++)
    {
        g_cybergear_found_ids[i] = found_ids[i];
    }

    g_cybergear_scan_done = true;
    if (0 == g_cybergear_found_count)
    {
        vTaskDelete(nullptr);
        return;
    }

    // 用扫描到的第一个ID创建电机实例（静态对象，避免动态内存）
    const uint8_t motor_id = g_cybergear_found_ids[0];
    static cybergear_motor_drv_t motor(motor_id, 0x00, bsp_can::can1);
    g_cybergear_current_id = motor_id;

    // 使能电机
    motor.enable();

    // 主控制循环
    while (1)
    {
        // ===== 使能/失能控制 =====
        if (g_cybergear_enable_cmd && !motor.is_enable())
        {
            motor.enable();
        }
        else if (!g_cybergear_enable_cmd && motor.is_enable())
        {
            motor.disable();
        }

        // ===== 更改电机ID =====
        // 注意：Cybergear必须在失能状态下才能修改ID，
        // 修改后需等待电机完成ID切换，再重新使能
        if (g_cybergear_new_id != 0 && g_cybergear_new_id != motor.get_motor_id())
        {
            // 1. 先失能电机（使能状态下修改ID会被电机忽略）
            motor.disable();
            vTaskDelay(pdMS_TO_TICKS(100));

            // 2. 发送设置ID命令，检查返回值
            volatile status_t set_id_ret = motor.set_motor_id(g_cybergear_new_id);
            if (PYRO_OK == set_id_ret)
            {
                g_cybergear_current_id = g_cybergear_new_id;
                // 3. 等待电机ID切换完成（协议说立即生效，但实际有延迟）
                vTaskDelay(pdMS_TO_TICKS(50));
                // 注意：设置ID后电机不会发确认帧，
                // 需重新扫描总线才能确认新ID是否生效
            }
            // 4. 不管成功失败都清零触发位
            g_cybergear_new_id = 0;
        }

        // ===== 发送MIT运动控制帧 =====
        // 注意：Cybergear只有收到MIT控制帧(类型1)才会回应反馈帧(类型2)
        // 所以必须持续发送MIT控制帧，才能持续收到反馈
        motor.send_motion_control(
            g_cybergear_cmd_torque,
            g_cybergear_cmd_position,
            g_cybergear_cmd_velocity,
            g_cybergear_cmd_kp,
            g_cybergear_cmd_kd);

        // ===== 读取反馈 =====
        // 反馈数据在CAN中断回调中自动更新，这里直接读取
        g_cybergear_fb_position    = motor.get_current_position();
        g_cybergear_fb_velocity    = motor.get_current_rotate();
        g_cybergear_fb_torque      = motor.get_current_torque();
        g_cybergear_fb_temperature = motor.get_temperature();
        g_cybergear_fb_fault       = static_cast<uint8_t>(motor.get_fault());
        g_cybergear_fb_mode_state  = static_cast<uint8_t>(motor.get_mode_state());
        g_cybergear_fb_bus_voltage = motor.get_bus_voltage();
        g_cybergear_fb_online      = motor.is_online();

        // 200Hz控制周期（5ms）
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    // 扫描完成，任务删除自己
    // 如果需要后续控制电机，在这里继续写控制逻辑，
    // 或者在其他任务中等待 g_cybergear_scan_done == true 后再操作。

}
