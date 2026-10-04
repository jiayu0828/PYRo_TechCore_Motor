#ifndef TECHCORE_GLOBAL_H
#define TECHCORE_GLOBAL_H

#include "pyro_core_def.h"
#include "pyro_bsp_can.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_cybergear_motor_drv.h"
#include "pyro_algo_pid.h"

#include "move_switch.h"
#include "move_control.h"

// ============================================================
// 科技核心全局对象声明
// 所有对象在 pyro_init_thread 中用 new 创建，
// techcore_ctrl_task 通过这些指针引用，保证初始化顺序可控。
// ============================================================

// --- 限位开关 ---
extern move_switch_t*          g_move_switch;

// --- X/Z平移轴: 大疆M3508 (CAN2) ---
extern pyro::dji_m3508_motor_drv_t*  g_x_motor;
extern pyro::dji_m3508_motor_drv_t*  g_z_motor;
extern pyro::pid_t*            g_x_pos_pid;
extern pyro::pid_t*            g_x_rot_pid;
extern pyro::pid_t*            g_z_pos_pid;
extern pyro::pid_t*            g_z_rot_pid;
extern move_control_t*         g_x_axis_ctrl;
extern move_control_t*         g_z_axis_ctrl;

// --- YPR旋转轴: 小米Cybergear (CAN1) ---
extern pyro::pid_t*            g_yaw_pos_pid;
extern pyro::pid_t*            g_yaw_rot_pid;
extern pyro::pid_t*            g_pitch_pos_pid;
extern pyro::pid_t*            g_pitch_rot_pid;
extern pyro::pid_t*            g_roll_pos_pid;
extern pyro::pid_t*            g_roll_rot_pid;
extern pyro::cybergear_motor_drv_t*  g_yaw_motor;
extern pyro::cybergear_motor_drv_t*  g_pitch_motor;
extern pyro::cybergear_motor_drv_t*  g_roll_motor;

// --- 初始化状态标志 ---
// true=所有电机和控制器初始化完成且使能成功，控制任务可安全运行
extern volatile bool g_techcore_init_done;

#endif // TECHCORE_GLOBAL_H
