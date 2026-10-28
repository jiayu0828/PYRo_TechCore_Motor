/**
 * @file pyro_task_monitor.h
 * @brief 任务栈剩余空间监控（栈高水印）
 *
 * 任务创建后调用 pyro_task_monitor_register() 登记句柄，
 * 监控任务周期性调用 uxTaskGetStackHighWaterMark() 打印历史最小剩余栈。
 *
 * 依赖 FreeRTOS 配置：INCLUDE_uxTaskGetStackHighWaterMark = 1
 */

#ifndef __PYRO_TASK_MONITOR_H__
#define __PYRO_TASK_MONITOR_H__

#include "FreeRTOS.h"
#include "pyro_core_def.h"
#include "task.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** 监控表容量，超出后新任务不再登记 */
#define PYRO_TASK_MONITOR_MAX 12U

/**
 * @brief 登记一个任务句柄
 * @param handle 任务句柄，传 NULL 直接返回
 * @return PYRO_OK / PYRO_NO_MEMORY（表满）/ PYRO_PARAM_ERROR
 */
pyro::status_t pyro_task_monitor_register(TaskHandle_t handle);

/**
 * @brief 取消登记（任务删除前调用，避免访问已释放的 TCB）
 */
void pyro_task_monitor_unregister(TaskHandle_t handle);

/**
 * @brief 打印所有已登记任务的栈高水印
 * @note 仅可在任务上下文调用
 */
void pyro_task_monitor_report(void);

/**
 * @brief 启动监控任务（每 5s 打印一次）
 * @return PYRO_OK / PYRO_ERROR
 */
pyro::status_t pyro_task_monitor_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __PYRO_TASK_MONITOR_H__ */
