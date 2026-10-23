/**
 * @file pyro_log.h
 * @brief PYRo 库统一日志抽象接口
 * 库内模块（驱动、任务、算法等）不直接依赖具体日志实现（如 EasyLogger），
 * 只通过一个日志回调指针输出。回调由上层（应用层）注入，未注入时日志静默丢弃。
 *
 * 使用方式：
 *   1. 应用层调用 pyro_log_set_handler() 注入日志实现；
 *   2. 库内模块使用 PYRO_LOGI/E/W/D/V 宏打印日志；
 *   3. 通过 PYRO_LOG_ENABLE 宏在编译期整体开关库内日志。
 */

#ifndef __PYRO_LOG_H__
#define __PYRO_LOG_H__

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------*/
/* PYRo 库日志编译期总开关 */
/* 定义为 1 即启用库内日志；定义为 0 可在编译期彻底移除库内日志代码（零开销） */
/* 可通过编译选项 -DPYRO_LOG_ENABLE=0 关闭，或在 CMake 中统一定义 */
#ifndef PYRO_LOG_ENABLE
#define PYRO_LOG_ENABLE 1
#endif
/*---------------------------------------------------------------------------*/

/* 日志等级，与 EasyLogger 等级保持一致，便于上层直接映射 */
typedef enum
{
    PYRO_LOG_LVL_ASSERT  = 0,
    PYRO_LOG_LVL_ERROR   = 1,
    PYRO_LOG_LVL_WARN    = 2,
    PYRO_LOG_LVL_INFO    = 3,
    PYRO_LOG_LVL_DEBUG   = 4,
    PYRO_LOG_LVL_VERBOSE = 5,
} pyro_log_level_t;

/**
 * @brief 库内日志回调类型（方案B：va_list 变体）
 * @param level 日志等级
 * @param tag   日志标签（模块名，如 "UART"/"CAN"/"TASK"）
 * @param format 格式化字符串
 * @param args  可变参数列表
 */
typedef void (*pyro_log_fn)(pyro_log_level_t level, const char *tag,
                            const char *format, va_list args);

/**
 * @brief 注册库内日志回调
 * @param fn 回调函数指针，传 NULL 表示关闭库内日志输出
 */
void pyro_log_set_handler(pyro_log_fn fn);

/**
 * @brief 库内日志统一输出入口（供宏调用，一般不直接使用）
 */
void pyro_log_output(pyro_log_level_t level, const char *tag,
                     const char *format, ...);

#if PYRO_LOG_ENABLE
#define PYRO_LOG(level, tag, ...)                                              \
    pyro_log_output((level), (tag), __VA_ARGS__)
#else
#define PYRO_LOG(level, tag, ...) ((void)0)
#endif

#define PYRO_LOGA(tag, ...) PYRO_LOG(PYRO_LOG_LVL_ASSERT, tag, __VA_ARGS__)
#define PYRO_LOGE(tag, ...) PYRO_LOG(PYRO_LOG_LVL_ERROR, tag, __VA_ARGS__)
#define PYRO_LOGW(tag, ...) PYRO_LOG(PYRO_LOG_LVL_WARN, tag, __VA_ARGS__)
#define PYRO_LOGI(tag, ...) PYRO_LOG(PYRO_LOG_LVL_INFO, tag, __VA_ARGS__)
#define PYRO_LOGD(tag, ...) PYRO_LOG(PYRO_LOG_LVL_DEBUG, tag, __VA_ARGS__)
#define PYRO_LOGV(tag, ...) PYRO_LOG(PYRO_LOG_LVL_VERBOSE, tag, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* __PYRO_LOG_H__ */
