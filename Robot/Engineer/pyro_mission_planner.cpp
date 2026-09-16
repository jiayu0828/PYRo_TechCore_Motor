#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
    extern void pyro_init_thread(void *argument);
    extern void cybergear_scan_task(void *argument);
}//用来记录任务线程

extern "C" void start_mission_planer_task(void const *argument)
{
    // 创建初始化任务（最高优先级，最先跑）
    xTaskCreate(
        pyro_init_thread,
        "pyro_init",
        512,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr);

    // 等待底层驱动初始化完成
    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreate(
        cybergear_scan_task,
        "cyber_scan",
        2048,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr);

    // 规划器任务完成使命，删除自己
    vTaskDelete(nullptr);
}
