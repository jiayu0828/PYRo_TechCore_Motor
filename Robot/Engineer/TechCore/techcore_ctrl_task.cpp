/**
 * @file techcore_ctrl_task.cpp
 * @brief 科技核心主控任务 - 统一发送架构
 *
 * 架构:
 *   每帧 → 更新反馈 → 状态机计算目标 → 末尾统一发送所有电机
 *   中间函数只写全局输出, 不直接调用send
 *
 * 系统状态:
 *   IDLE         空闲位置保持, Normal力矩
 *   CALIBRATION  校准中(指令1), 完成后自动HOLD
 *   POSE_MOVE    姿态调整中(指令2), 完成后自动HOLD
 *   HOLD         大力矩保持
 *   ESTOP        紧急停止, 全部失能
 */

#include "pyro_core_def.h"
#include "pyro_dwt_drv.h"
#include "FreeRTOS.h"
#include "task.h"

#include "techcore_global.h"
#include "techcore_command.h"

#include <cstdint>
#include <cstring>
#include <cstdio>
#include "pyro_bsp_uart.h"
using namespace pyro;

// ============================================================
// 状态枚举
// ============================================================
enum sys_state_t {
    SYS_IDLE = 0,
    SYS_CALIBRATION,
    SYS_POSE_MOVE,
    SYS_HOLD,
    SYS_ESTOP,
};

enum cali_phase_t {
    CALI_FIND_NEG = 0,
    CALI_FIND_POS,
    CALI_CENTER,
};

enum pose_state_t {
    POSE_X_MOVE = 0,
    POSE_Z_MOVE,
    POSE_YPR_CENTER,
    POSE_YPR_ROTATE,
    POSE_DONE,
};

// ============================================================
// 电机在线配置 (true=已接线)
// ============================================================
static const bool motor_online[3] = {true, true, true};  // Yaw, Pitch, Roll

// ============================================================
// 全局状态变量
// ============================================================
static sys_state_t  sys_state    = SYS_IDLE;
static bool         torque_hold  = false;   // true=大力矩HOLD, false=Normal
static bool         cali_done    = false;   // 校准完成标志，没完成前不上力

// ============================================================
// 校准状态
// ============================================================
static int          cali_axis     = 0;     // 0=Yaw 1=Pitch 2=Roll 3=XZ 4=Done
static cali_phase_t cali_phase    = CALI_FIND_NEG;
static float        cali_stall_t  = 0.0f;
static float        cali_find_t   = 0.0f;
static float        cali_neg_pos  = 0.0f;
static float        cali_pos_pos  = 0.0f;

// ============================================================
// 姿态调整状态
// ============================================================
static pose_state_t pose_state    = POSE_X_MOVE;
static float        pose_start_t  = 0.0f;
static int          pose_action_i = 0;
static const pose_action_t* pose_act = nullptr;

// ============================================================
// 零点偏移与动态限位 (校准后设置)
// ============================================================
static float yaw_offset = 0.0f, pitch_offset = 0.0f, roll_offset = 0.0f;
static float yaw_dmin = -3.14159f, yaw_dmax = 3.14159f;
static float pitch_dmin = PITCH_LIMIT_MIN, pitch_dmax = PITCH_LIMIT_MAX;
static float roll_dmin  = ROLL_LIMIT_MIN,  roll_dmax  = ROLL_LIMIT_MAX;

// ============================================================
// 目标值 (各状态函数写入, PID使用)
// ============================================================
volatile float g_x_target_pos   = 0.0f;
volatile float g_z_target_pos   = 0.0f;
volatile float g_yaw_target     = 0.0f;
volatile float g_pitch_target   = 0.0f;
volatile float g_roll_target    = 0.0f;

// ============================================================
// 反馈值 (每帧更新)
// ============================================================
volatile float g_x_fb_pos    = 0.0f;
volatile float g_z_fb_pos    = 0.0f;
volatile float g_yaw_fb_pos  = 0.0f;
volatile float g_yaw_vel_fb  = 0.0f;
volatile float g_pitch_fb_pos= 0.0f;
volatile float g_roll_fb_pos = 0.0f;

// ============================================================
// 限位开关
// ============================================================
volatile uint8_t sw_up      = 0;
volatile uint8_t sw_down    = 0;
volatile uint8_t sw_forward = 0;
volatile uint8_t sw_back    = 0;

// ============================================================
// 调试变量 (Live Watch)
// ============================================================
volatile uint8_t  g_sys_state_debug   = 0;
volatile uint8_t  g_cali_axis_debug   = 0;
volatile uint8_t  g_cali_phase_debug  = 0;
volatile uint8_t  g_pose_sub_debug   = 0;
volatile uint8_t  g_pose_action_debug = 0;
volatile float    g_yaw_debug_val     = 0.0f;
volatile float    g_yaw_trq_sent     = 0.0f;
volatile float    g_pitch_trq_sent   = 0.0f;
volatile float    g_roll_trq_sent    = 0.0f;
volatile float    g_pitch_fb_val     = 0.0f;
volatile float    g_pitch_target_speed = 0.0f;
// ============================================================
// 每帧电机输出 (统一发送)
// ============================================================
static struct {
    float yaw_trq;
    float pitch_trq;
    float roll_trq;
    float x_speed;     // X目标速度(rad/s), 速度模式
    float z_speed;     // Z目标速度(rad/s)
    bool  x_speed_mode; // true=速度控制, false=位置控制
    bool  z_speed_mode;
} out;

// ============================================================
// 辅助
// ============================================================
static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float torque_limit(void) {
    return torque_hold ? TORQUE_LIMIT_HOLD : TORQUE_LIMIT_NORMAL;
}

// ============================================================
// YPR PID计算 (只计算力矩写入out, 不发送)
// stall_enable: 是否启用堵转保护
// ============================================================
static void calc_ypr_pid()
{
    float lim = torque_limit();

    // Yaw (有力矩限制)
    if (motor_online[0]) {
        float vel = g_yaw_pos_pid->calculate(g_yaw_target, g_yaw_fb_pos);
        float trq = g_yaw_rot_pid->calculate(vel, g_yaw_motor->get_current_rotate());
        trq = clampf(trq, -lim, lim);
        out.yaw_trq = trq;
    }
    // Pitch (不限制力矩)
    if (motor_online[1]) {
        float vel = g_pitch_pos_pid->calculate(g_pitch_target, g_pitch_fb_pos);
        out.pitch_trq = g_pitch_rot_pid->calculate(vel, g_pitch_motor->get_current_rotate());
        g_pitch_fb_val = g_pitch_motor->get_current_rotate();
        g_pitch_target_speed = vel;
    }
    // Roll (不限制力矩)
    if (motor_online[2]) {
        float vel = g_roll_pos_pid->calculate(g_roll_target, g_roll_fb_pos);
        out.roll_trq = g_roll_rot_pid->calculate(vel, g_roll_motor->get_current_rotate());
    }
}

// ============================================================
// 保持非活跃YPR轴位置 (校准中用)
// ============================================================
static void hold_ypr_except(int active)
{
    float lim = torque_limit();
    struct { motor_base_t* m; pid_t* pos; pid_t* rot; float tgt; float fb; float* out; }
    ax[3] = {
        {g_yaw_motor,   g_yaw_pos_pid,   g_yaw_rot_pid,   g_yaw_target,   g_yaw_fb_pos,   &out.yaw_trq},
        {g_pitch_motor, g_pitch_pos_pid, g_pitch_rot_pid, g_pitch_target, g_pitch_fb_pos, &out.pitch_trq},
        {g_roll_motor,  g_roll_pos_pid,  g_roll_rot_pid,  g_roll_target,  g_roll_fb_pos,  &out.roll_trq},
    };
    for (int i = 0; i < 3; i++) {
        if (i == active || !motor_online[i]) continue;
        float vel = ax[i].pos->calculate(ax[i].tgt, ax[i].fb);
        float trq = ax[i].rot->calculate(vel, ax[i].m->get_current_rotate());
        // 只有Yaw(i=0)限制力矩, Pitch/Roll不限制
        *ax[i].out = (i == 0) ? clampf(trq, -lim, lim) : trq;
    }
}

// ============================================================
// X/Z速度设置 (只设目标速度, 不发送, 末尾统一发)
// 方向: X正=向前, X负=向后; Z正=向上, Z负=向下
// ============================================================
static void set_xz_speed(float xs, float zs)
{
    // 限位方向限制
    if (sw_forward && xs > 0.0f) xs = 0.0f;
    if (sw_back    && xs < 0.0f) xs = 0.0f;
    if (sw_up      && zs > 0.0f) zs = 0.0f;
    if (sw_down    && zs < 0.0f) zs = 0.0f;

    out.x_speed = xs;
    out.z_speed = zs;
    out.x_speed_mode = true;
    out.z_speed_mode = true;
}

// X/Z位置保持 (只设目标位置, 不发送)
static void set_xz_position(void)
{
    out.x_speed = 0;
    out.z_speed = 0;
    out.x_speed_mode = false;
    out.z_speed_mode = false;
}

// ============================================================
// 校准一步
// ============================================================
static void cali_step(void)
{
    float now = dwt_drv_t::get_timeline_ms();

    // --- X/Z校准 (axis=3): X向前Z向上, 碰到限位即完成 ---
    if (cali_axis == 3) {
        calc_ypr_pid();
        set_xz_speed(X_SPEED_FORWARD, Z_SPEED_UPWARD);
        if ((sw_forward || sw_back) && (sw_up || sw_down)) {
            cali_axis = 4;
            sys_state = SYS_HOLD;
            torque_hold = true;
            cali_done = true;
            g_x_target_pos = g_x_fb_pos;
            g_z_target_pos = g_z_fb_pos;
            g_yaw_target   = g_yaw_fb_pos;
            g_pitch_target = g_pitch_fb_pos;
            g_roll_target  = g_roll_fb_pos;
        }
        return;
    }

    // --- YPR单轴校准 ---
    if (!motor_online[cali_axis]) { cali_axis++; cali_phase = CALI_FIND_NEG; cali_stall_t = 0; return; }

    cybergear_motor_drv_t* motor = (cali_axis==0) ? g_yaw_motor :
                                   (cali_axis==1) ? g_pitch_motor : g_roll_motor;
    float* offset = (cali_axis==0) ? &yaw_offset :
                    (cali_axis==1) ? &pitch_offset : &roll_offset;
    float* dmin = (cali_axis==0) ? &yaw_dmin :
                  (cali_axis==1) ? &pitch_dmin : &roll_dmin;
    float* dmax = (cali_axis==0) ? &yaw_dmax :
                  (cali_axis==1) ? &pitch_dmax : &roll_dmax;

    hold_ypr_except(cali_axis);
    pid_t* rot_pid = (cali_axis==0) ? g_yaw_rot_pid :
                     (cali_axis==1) ? g_pitch_rot_pid : g_roll_rot_pid;
    float trq = 0;

    switch (cali_phase) {
    case CALI_FIND_NEG:
        // 速度环控速: 目标 -CALI_SPEED_TARGET
        trq = rot_pid->calculate(-CALI_SPEED_TARGET, motor->get_current_rotate());
        trq = clampf(trq, -CALI_MAX_TRQ, CALI_MAX_TRQ);
        if (cali_find_t == 0) cali_find_t = now;
        {
            float spd = motor->get_current_rotate();
            // 堵转判定: 力矩大 + 速度低
            bool stalled = (fabsf(trq) > CALI_STALL_TRQ_THRESH) &&
                           (fabsf(spd) < CALI_STALL_SPEED);
            bool timeout = (now - cali_find_t > CALI_MAX_TIME_MS);
            if (stalled || timeout) {
                if (cali_stall_t == 0) cali_stall_t = now;
                else if (now - cali_stall_t > CALI_STALL_TIME_MS) {
                    // 堵转确认, 直接记录位置, 切找正方向
                    cali_neg_pos = motor->get_current_position();
                    cali_stall_t = 0;
                    cali_find_t = 0;
                    cali_phase = CALI_FIND_POS;
                }
            } else cali_stall_t = 0;
        }
        break;

    case CALI_FIND_POS:
        // 速度环控速: 目标 +CALI_SPEED_TARGET
        trq = rot_pid->calculate(CALI_SPEED_TARGET, motor->get_current_rotate());
        trq = clampf(trq, -CALI_MAX_TRQ, CALI_MAX_TRQ);
        if (cali_find_t == 0) cali_find_t = now;
        {
            float spd = motor->get_current_rotate();
            bool stalled = (fabsf(trq) > CALI_STALL_TRQ_THRESH) &&
                           (fabsf(spd) < CALI_STALL_SPEED);
            bool timeout = (now - cali_find_t > CALI_MAX_TIME_MS);
            if (stalled || timeout) {
                if (cali_stall_t == 0) cali_stall_t = now;
                else if (now - cali_stall_t > CALI_STALL_TIME_MS) {
                    // 堵转确认, 记录位置, 算限位范围
                    cali_pos_pos = motor->get_current_position();
                    *offset = (cali_neg_pos + cali_pos_pos) / 2.0f;
                    *dmin = cali_neg_pos - *offset;
                    *dmax = cali_pos_pos - *offset;
                    cali_stall_t = 0;
                    cali_find_t = 0;
                    cali_phase = CALI_CENTER;
                }
            } else cali_stall_t = 0;
        }
        break;

    case CALI_CENTER: {
        float fb = motor->get_current_position() - *offset;
        pid_t* ppos = (cali_axis==0) ? g_yaw_pos_pid : (cali_axis==1) ? g_pitch_pos_pid : g_roll_pos_pid;
        float vel = ppos->calculate(0.0f, fb);
        trq = clampf(rot_pid->calculate(vel, motor->get_current_rotate()), -CALI_CENTER_TORQUE, CALI_CENTER_TORQUE);
        if (fb < 0.1f && fb > -0.1f) {
            cali_axis++;
            cali_phase = CALI_FIND_NEG;
            cali_stall_t = 0;
            cali_find_t = 0;
        }
        break;
    }
    }

    // 写入当前校准轴力矩输出
    if (cali_axis == 0) out.yaw_trq = trq;
    else if (cali_axis == 1) out.pitch_trq = trq;
    else if (cali_axis == 2) out.roll_trq = trq;

    set_xz_position();
}

// ============================================================
// 姿态调整一步
// ============================================================
static void pose_step(void)
{
    float now = dwt_drv_t::get_timeline_ms();
    if (!pose_act) return;

    // --- X移动 ---
    if (pose_state == POSE_X_MOVE) {
        float el = now - pose_start_t;
        float xs = (float)pose_act->x_dir * MOVE_SPEED_RAD;
        bool hit = (xs > 0 && sw_forward) || (xs < 0 && sw_back);
        if (el < pose_act->x_time_ms && !hit)
            set_xz_speed(xs, 0);
        else {
            g_x_target_pos = g_x_fb_pos;
            pose_state = POSE_Z_MOVE;
            pose_start_t = now;
        }
        calc_ypr_pid();
        return;
    }

    // --- Z移动 ---
    if (pose_state == POSE_Z_MOVE) {
        float el = now - pose_start_t;
        float zs = (float)pose_act->z_dir * MOVE_SPEED_RAD;
        bool hit = (zs > 0 && sw_up) || (zs < 0 && sw_down);
        if (el < pose_act->z_time_ms && !hit)
            set_xz_speed(0, zs);
        else {
            g_z_target_pos = g_z_fb_pos;
            // 直接设置YPR目标(无插值,直接跳转)
            g_yaw_target   = clampf(pose_act->yaw_target,   yaw_dmin, yaw_dmax);
            g_pitch_target = clampf(pose_act->pitch_target, pitch_dmin, pitch_dmax);
            g_roll_target  = clampf(pose_act->roll_target,  roll_dmin, roll_dmax);
            pose_state = POSE_DONE;
            sys_state = SYS_HOLD;
            torque_hold = true;
        }
        calc_ypr_pid();
        return;
    }
    set_xz_position();
    calc_ypr_pid();
}

// ============================================================
// 统一发送 (每帧末尾调用)
// ============================================================
static void flush_motors(void)
{
    // X/Z: 根据模式发送
    if (out.x_speed_mode) {
        g_x_axis_ctrl->set_target_speed(out.x_speed);
    } else {
        g_x_axis_ctrl->set_target_pos(g_x_target_pos);
    }
    g_x_axis_ctrl->control();

    if (out.z_speed_mode) {
        g_z_axis_ctrl->set_target_speed(out.z_speed);
    } else {
        g_z_axis_ctrl->set_target_pos(g_z_target_pos);
    }
    g_z_axis_ctrl->control();

    // YPR小米电机: 统一发力矩
    if (motor_online[0]) g_yaw_motor->send_motion_control(out.yaw_trq, 0, 0, 0, 0);
    if (motor_online[1]) g_pitch_motor->send_motion_control(out.pitch_trq, 0, 0, 0, 0);
    if (motor_online[2]) g_roll_motor->send_motion_control(out.roll_trq, 0, 0, 0, 0);

    // 调试记录
    g_yaw_trq_sent   = out.yaw_trq;
    g_pitch_trq_sent = out.pitch_trq;
    g_roll_trq_sent  = out.roll_trq;
    // 通过USART1发送二进制帧数据，供串口绘图 (20ms一次)
    // 帧格式: 0xAA | float yaw_fb | float yaw_target | float yaw_trq | 0x55
    // 共14字节, 小端序
    static float print_t = 0;
    float now_ms = dwt_drv_t::get_timeline_ms();
    if (now_ms - print_t > 20.0f) {
        print_t = now_ms;
        union { float f; uint8_t b[4]; } c1, c2, c3;
        c1.f = g_pitch_trq_sent;//发送的力矩值
        c2.f = g_pitch_target_speed;//速度的期望值
        c3.f = g_pitch_fb_val;//速度的反馈值
        uint8_t tx[14];
        tx[0]  = 0xAA;
        tx[1]  = c1.b[0]; tx[2]  = c1.b[1]; tx[3]  = c1.b[2]; tx[4]  = c1.b[3];
        tx[5]  = c2.b[0]; tx[6]  = c2.b[1]; tx[7]  = c2.b[2]; tx[8]  = c2.b[3];
        tx[9]  = c3.b[0]; tx[10] = c3.b[1]; tx[11] = c3.b[2]; tx[12] = c3.b[3];
        tx[13] = 0x55;
        bsp_uart::get_uart1().write(tx, 14, 10);
    }

}

// ============================================================
// 主任务
// ============================================================
extern "C" void techcore_ctrl_task(void *argument)
{
    (void)argument;
    while (!g_techcore_init_done) vTaskDelay(pdMS_TO_TICKS(10));

    torque_hold = false;
    pose_act = &g_pose_actions[pose_action_i];

    for (;;) {
        float now = dwt_drv_t::get_timeline_ms();

        // 1. 更新开关
        g_move_switch->update();
        sw_up      = g_move_switch->_up_switch_hold      ? 1 : 0;
        sw_down    = g_move_switch->_down_switch_hold    ? 1 : 0;
        sw_forward = g_move_switch->_forawrd_switch_hold ? 1 : 0;
        sw_back    = g_move_switch->_back_switch_hold     ? 1 : 0;

        // 2. 更新反馈
        g_x_axis_ctrl->update();
        g_z_axis_ctrl->update();
        g_x_fb_pos = g_x_axis_ctrl->get_feedback_pos();
        g_z_fb_pos = g_z_axis_ctrl->get_feedback_pos();
        g_yaw_motor->update_feedback();
        g_pitch_motor->update_feedback();
        g_roll_motor->update_feedback();
        g_yaw_fb_pos   = g_yaw_motor->get_current_position()   - yaw_offset;
        g_yaw_vel_fb   = g_yaw_motor->get_current_rotate();
        g_pitch_fb_pos = g_pitch_motor->get_current_position() - pitch_offset;
        g_roll_fb_pos  = g_roll_motor->get_current_position()  - roll_offset;

        // 3. 每帧输出清零
        memset(&out, 0, sizeof(out));
        out.x_speed_mode = false;
        out.z_speed_mode = false;

        // 4. 处理指令
        if (cmd_recv.estop_enable) {
            cmd_recv.estop_enable = 0;
            sys_state = SYS_ESTOP;
            g_yaw_motor->disable();
            g_pitch_motor->disable();
            g_roll_motor->disable();
            g_x_motor->disable();
            g_z_motor->disable();
        } else if (sys_state != SYS_ESTOP) {
            if (cmd_recv.calibrate_enable) {
                cmd_recv.calibrate_enable = 0;
                sys_state = SYS_CALIBRATION;
                cali_axis = 0;
                cali_phase = CALI_FIND_NEG;
                cali_stall_t = 0;
                torque_hold = false;
            } else if (cmd_recv.pose_move_enable) {
                cmd_recv.pose_move_enable = 0;
                if (sys_state != SYS_CALIBRATION) {
                    pose_act = &g_pose_actions[pose_action_i];
                    pose_action_i = (pose_action_i + 1) % g_pose_action_count;
                    sys_state = SYS_POSE_MOVE;
                    pose_state = POSE_X_MOVE;
                    pose_start_t = now;
                    torque_hold = false;
                }
            }
        }

        // 5. 状态计算 (只写out, 不发送)
        switch (sys_state) {
        case SYS_ESTOP:
            out.yaw_trq = 0; out.pitch_trq = 0; out.roll_trq = 0;
            out.x_speed = 0; out.z_speed = 0;
            break;
        case SYS_CALIBRATION:
            cali_step();
            break;
        case SYS_POSE_MOVE:
            pose_step();
            break;
        case SYS_HOLD:
            if (cali_done) {
                set_xz_position();
                calc_ypr_pid();
            } else {
                out.yaw_trq = 0; out.pitch_trq = 0; out.roll_trq = 0;
                out.x_speed = 0; out.z_speed = 0;
            }
            break;
        case SYS_IDLE:
        default:
            if (cali_done) {
                set_xz_position();
                calc_ypr_pid();
            } else {
                out.yaw_trq = 0; out.pitch_trq = 0; out.roll_trq = 0;
                out.x_speed = 0; out.z_speed = 0;
            }
            break;
        }

        // 6. 统一发送
        flush_motors();

        // 7. 调试变量
        g_sys_state_debug   = (uint8_t)sys_state;
        g_cali_axis_debug   = (uint8_t)cali_axis;
        g_cali_phase_debug  = (uint8_t)cali_phase;
        g_pose_sub_debug    = (uint8_t)pose_state;
        g_pose_action_debug = (uint8_t)pose_action_i;

        vTaskDelay(1);
    }
}
