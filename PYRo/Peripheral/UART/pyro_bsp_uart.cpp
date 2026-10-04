/**
* @file pyro_bsp_uart.cpp
 * @brief Implementation of the Board Support Package for UART.
 * 串口板级支持包实现文件。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-03-25
 */

#include "pyro_bsp_uart.h"
#include "usart.h"

namespace pyro
{

// Using local static variables to ensure a safe initialization order.
// 使用局部静态变量以确保安全的初始化顺序。

uart_drv_t& bsp_uart::get_uart1()
{
    // USART1: 调试串口，PA9(TX)/PB7(RX)
    static uart_drv_t instance(&huart1, 512);
    return instance;
}

uart_drv_t& bsp_uart::get_uart3()
{
    // USART3: DBUS遥控器，PC11(RX)，100000bps 8E2，仅接收
    // DBUS每帧18字节，256字节缓冲区足够
    static uart_drv_t instance(&huart3, 256);
    return instance;
}

uart_drv_t& bsp_uart::get_uart6()
{
    // USART6: 键盘通信，PG14(TX)/PG9(RX)，115200 8N1
    static uart_drv_t instance(&huart6, 512);
    return instance;
}

void bsp_uart::init_all()
{
    // Calling get_uartX() triggers the first-time construction of the instance.
    // 调用 get_uartX() 将触发实例的首次构造。
    get_uart1().enable_rx_dma();
    get_uart3().enable_rx_dma();
    get_uart6().enable_rx_dma();
}

} // namespace pyro
