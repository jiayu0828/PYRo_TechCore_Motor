#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "pyro_task_monitor.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
    extern void pyro_init_thread(void *argument);
    extern void cybergear_scan_task(void *argument);
    extern void techcore_ctrl_task(void *argument);
}//用来记录任务线程

extern "C" void start_mission_planer_task(void const *argument)
{
    TaskHandle_t handle = nullptr;

    // 创建初始化任务（最高优先级，最先跑）
    if (pdPASS == xTaskCreate(
        pyro_init_thread,
        "pyro_init",
        512,
        nullptr,
        configMAX_PRIORITIES - 1,
        &handle))
    {
        (void)pyro_task_monitor_register(handle);
    }

    // 等待底层驱动初始化完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 小米电机扫描任务已注释 - 当前使用硬编码ID(1/2/3对应Yaw/Pitch/Roll)
    // 如需扫描未知ID的电机，取消下面注释即可(扫描完成后任务自动退出)
    // xTaskCreate(
    //     cybergear_scan_task,
    //     "cyber_scan",
    //     512,
    //     nullptr,
    //     configMAX_PRIORITIES - 1,
    //     nullptr);

    // 创建科技核心主控任务(X/Z平移轴 + YPR旋转轴 + 限位检测 + 校准)
    handle = nullptr;
    if (pdPASS == xTaskCreate(
        techcore_ctrl_task,
        "techcore_ctrl",
        2048,
        nullptr,
        configMAX_PRIORITIES - 1,
        &handle))
    {
        (void)pyro_task_monitor_register(handle);
    }

    // 规划器任务完成使命，删除自己
    vTaskDelete(nullptr);
}
