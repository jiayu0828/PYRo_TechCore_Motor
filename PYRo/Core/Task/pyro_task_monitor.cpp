/**
 * @file pyro_task_monitor.cpp
 * @brief 任务栈剩余空间监控实现
 */

#include "pyro_task_monitor.h"
#include "pyro_log.h"

#include <cstring>

namespace
{
/* 栈剩余低于该值（单位：word，Cortex-M 一个 word = 4 字节）时告警 */
constexpr UBaseType_t STACK_WARN_THRESHOLD_WORD = 32U;

/* 监控周期 */
constexpr uint32_t MONITOR_PERIOD_MS = 5000U;

/* 监控任务自身栈大小（word） */
constexpr uint16_t MONITOR_TASK_STACK = 512U;

/* 监控任务优先级：低于控制任务，避免抢占实时控制 */
constexpr UBaseType_t MONITOR_TASK_PRIO = 1U;

struct entry_t
{
    TaskHandle_t handle;
};

entry_t s_entries[PYRO_TASK_MONITOR_MAX];
UBaseType_t s_count = 0;

void monitor_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        pyro_task_monitor_report();
        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
}
} // namespace

pyro::status_t pyro_task_monitor_register(TaskHandle_t handle)
{
    if (nullptr == handle)
        return pyro::PYRO_PARAM_ERROR;

    taskENTER_CRITICAL();

    /* 已登记过则直接返回，避免重复 */
    for (UBaseType_t i = 0; i < s_count; i++)
    {
        if (s_entries[i].handle == handle)
        {
            taskEXIT_CRITICAL();
            return pyro::PYRO_OK;
        }
    }

    if (s_count >= PYRO_TASK_MONITOR_MAX)
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_NO_MEMORY;
    }

    s_entries[s_count].handle = handle;
    s_count++;

    taskEXIT_CRITICAL();
    return pyro::PYRO_OK;
}

void pyro_task_monitor_unregister(TaskHandle_t handle)
{
    if (nullptr == handle)
        return;

    taskENTER_CRITICAL();
    for (UBaseType_t i = 0; i < s_count; i++)
    {
        if (s_entries[i].handle == handle)
        {
            /* 用最后一个元素填补空缺 */
            s_entries[i] = s_entries[s_count - 1];
            s_count--;
            break;
        }
    }
    taskEXIT_CRITICAL();
}

void pyro_task_monitor_report(void)
{
    /* 先在临界区内拷贝句柄快照，避免遍历时被并发修改 */
    TaskHandle_t snapshot[PYRO_TASK_MONITOR_MAX];
    UBaseType_t count;

    taskENTER_CRITICAL();
    count = s_count;
    for (UBaseType_t i = 0; i < count; i++)
    {
        snapshot[i] = s_entries[i].handle;
    }
    taskEXIT_CRITICAL();

    for (UBaseType_t i = 0; i < count; i++)
    {
        const TaskHandle_t handle = snapshot[i];
        if (nullptr == handle)
            continue;

        /* 任务可能已自行删除（如 pyro_init），跳过已释放的 TCB */
        if (eDeleted == eTaskGetState(handle))
        {
            pyro_task_monitor_unregister(handle);
            continue;
        }

        const UBaseType_t remain_words = uxTaskGetStackHighWaterMark(handle);
        const char *name = pcTaskGetName(handle);

        PYRO_LOGI("SYS_MON", "%-14s remain_stack_min: %lu word (%lu byte)",
                  (nullptr != name) ? name : "?",
                  (unsigned long)remain_words,
                  (unsigned long)(remain_words * sizeof(StackType_t)));

        if (remain_words < STACK_WARN_THRESHOLD_WORD)
        {
            PYRO_LOGW("SYS_MON", "[STACK WARN] %s stack remain only %lu word",
                      (nullptr != name) ? name : "?",
                      (unsigned long)remain_words);
        }
    }
}

pyro::status_t pyro_task_monitor_start(void)
{
    TaskHandle_t handle = nullptr;

    const BaseType_t ret = xTaskCreate(monitor_task, "sys_mon",
                                       MONITOR_TASK_STACK, nullptr,
                                       MONITOR_TASK_PRIO, &handle);
    if (pdPASS != ret)
    {
        PYRO_LOGE("SYS_MON", "create monitor task failed");
        return pyro::PYRO_ERROR;
    }

    (void)pyro_task_monitor_register(handle);
    return pyro::PYRO_OK;
}
