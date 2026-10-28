/**
 * @file pyro_log.c
 * @brief PYRo 库统一日志抽象接口实现
 */

#include "pyro_log.h"

/* 库内日志回调，默认为 NULL（未注入时日志静默丢弃） */
static pyro_log_fn s_log_handler = NULL;

void pyro_log_set_handler(pyro_log_fn fn)
{
    s_log_handler = fn;
}

void pyro_log_output(pyro_log_level_t level, const char *tag,
                     const char *format, ...)
{
    if (s_log_handler == NULL)
    {
        return;
    }

    va_list args;
    va_start(args, format);
    s_log_handler(level, tag, format, args);
    va_end(args);
}
