
# =========================================================================
# Engineer 工程车代码库（PYRo_TechCore_Motor 电机测试工程）
# =========================================================================
add_library(Engineer OBJECT)

target_compile_options(Engineer PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# ===================== 源文件 =====================
target_sources(Engineer PRIVATE
        pyro_init_thread.cpp
        pyro_mission_planner.cpp
        TechCore/cybergear_scan_task.cpp

)

# ===================== 头文件包含路径 =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}            # Engineer 根目录
        ${CMAKE_CURRENT_SOURCE_DIR}/TechCore   # 应用层任务头文件
)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
