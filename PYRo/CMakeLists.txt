cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# 1. 项目定义
# ==============================================================================
# 子模块直接继承上级配置，只需声明库目标
add_library(PYRo OBJECT)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()



# ==============================================================================
# 5. 源文件配置 (按分类排序)
# ==============================================================================
target_sources(PYRo PRIVATE
        Core/Def/pyro_log.c
        Core/ETL/map.cpp
        Core/Lock/pyro_mutex.cpp
        Core/Lock/pyro_rw_lock.cpp
        Core/Task/pyro_task.cpp
        Core/Task/pyro_task_monitor.cpp
        Core/Memory/pyro_core_mem.cpp
        Core/Task/pyro_task.cpp

        # --- Peripheral ---
        Peripheral/CAN/pyro_can_drv.cpp
        Peripheral/CAN/pyro_bsp_can.cpp
        Peripheral/DWT/pyro_dwt_drv.cpp
        Peripheral/UART/pyro_uart_drv.cpp
        Peripheral/UART/pyro_bsp_uart.cpp

        # --- Algorithm ---
        Algorithm/Kinematics/pyro_kin_hybrid.cpp
        Algorithm/Kinematics/pyro_kin_mec.cpp
        Algorithm/Kinematics/pyro_kin_rudder.cpp
        Algorithm/Kinematics/pyro_kin_wl.cpp
        Algorithm/OLS/pyro_algo_ols.cpp
        Algorithm/PID/pyro_algo_pid.cpp
        Algorithm/PD/pyro_algo_pd.cpp
        Algorithm/EKF/QuaternionEKF.cpp
        Algorithm/KF/kalman_filter.cpp
        Algorithm/Common/pyro_algo_common.cpp
        Algorithm/CRC/PYRo_crc.cpp
        Algorithm/Powercontrol/pyro_power_control.cpp
        Algorithm/Navigation/pyro_nav_hub.cpp

        # --- Component ---
	Component/Motor/pyro_cybergear_motor_drv.cpp
        Component/Motor/pyro_dji_motor_drv.cpp
        Component/Motor/pyro_dm_motor_drv.cpp
        Component/Motor/pyro_motor_base.cpp


        # --- Debug ---

        # --- Device ---
        Device/WS2812/ws2812_driver.cpp
        Device/WS2812/ws2812_middleware.cpp

        # --- Log ---
        Log/log.c
        Log/elog/elog.c
        Log/elog/elog_utils.c
        Log/elog/port/elog_port.c
        Log/RTT/SEGGER_RTT.c
        Log/RTT/SEGGER_RTT_printf.c
        Log/RTT/SEGGER_RTT_ASM_ARMv7M.S

)


# ==============================================================================
# 6. 包含路径与链接 (按路径排序)
# ==============================================================================
target_include_directories(PYRo PUBLIC
        # --- Core ---
        Core/Config
        Core/Def
        Core/ETL
        Core/FSM
        Core/Lock
        Core/Memory
        Core/Task

        # --- Peripheral ---
        Peripheral/CAN
        Peripheral/DWT
        Peripheral/UART

        # --- Device ---

        Device/WS2812
        
        # --- Algorithm ---
        Algorithm/Common
        Algorithm/Kinematics
        Algorithm/OLS
        Algorithm/PID
        Algorithm/PD
        Algorithm/EKF
        Algorithm/KF
        Algorithm/LESO
        Algorithm/CRC
        Algorithm/Powercontrol
        Algorithm/Navigation

        # --- Component ---

        Component/Motor


        # --- Module ---
        Module

        # --- Debug ---

        # --- Log ---
        Log
        Log/elog
        Log/elog/port
        Log/RTT

)


target_link_libraries(PYRo PUBLIC stm32cubemx dsppp_lib)
