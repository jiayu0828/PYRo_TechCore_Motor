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
#include "pyro_log.h"
#include "FreeRTOS.h"
#include "task.h"

#include "techcore_global.h"
#include "techcore_command.h"

#include <cstdint>
#include <cstring>
#include <cstdio>
#include "pyro_bsp_can.h"
#include "pyro_bsp_uart.h"
using namespace pyro;

// CAN 发送失败统计（定义在 pyro_can_drv.cpp）
extern "C" uint32_t can_get_tx_busy_cnt(void);
extern "C" uint32_t can_get_tx_error_cnt(void);

// ============================================================
// CAN 回环自检
//
// 原理：把该路 CAN 临时切到 LOOPBACK 模式，bxCAN 会把发出去的帧
// 在芯片内部环回给自己，不依赖外部节点回 ACK。
//   PASS -> MCU 侧(外设/驱动/位时序/过滤器)正常，问题在外部接线或电机
//   FAIL -> MCU 侧配置有问题，先查软件
// 自检期间该路电机通信会中断约 50ms。
// ============================================================
static void run_can_selftest(bsp_can::which_can which, const char *label)
{
    can_drv_t *can = bsp_can::get_can(which);
    if (nullptr == can)
    {
        PYRO_LOGE("CAN", "%s selftest: invalid instance", label);
        return;
    }

    PYRO_LOGI("CAN", "%s selftest start (loopback)", label);

    can_selftest_result_t r{};
    const status_t st = can->run_loopback_selftest(20U, &r);

    if (PYRO_OK != st)
    {
        PYRO_LOGE("CAN", "%s selftest FAIL: flow error (ret=%d)", label,
                  (int)st);
        return;
    }

    if (r.tx_ok && r.rx_ok && r.data_ok)
    {
        PYRO_LOGI("CAN",
                  "%s selftest PASS (tx=1 rx=1 data=1, %lums)",
                  label, (unsigned long)r.elapsed_ms);
        PYRO_LOGI("CAN",
                  "%s -> MCU side OK. If the bus still Bus-Off, the fault is "
                  "wiring / transceiver / motor power, not software.",
                  label);
        return;
    }

    PYRO_LOGE("CAN",
              "%s selftest FAIL (tx=%d rx=%d data=%d tx_ret=%d esr=0x%08lX, %lums)",
              label, (int)r.tx_ok, (int)r.rx_ok, (int)r.data_ok,
              r.tx_ret, (unsigned long)r.esr,
              (unsigned long)r.elapsed_ms);
    PYRO_LOGE("CAN",
              "%s -> MCU side problem: check CAN init / bit timing / filter.",
              label);
}

// ============================================================
// 小米电机诊断字段格式化
// ============================================================
static const char *motor_mode_name(uint8_t mode)
{
    switch (mode)
    {
    case 0:  return "RESET";
    case 1:  return "CALI";
    case 2:  return "MOTOR";
    default: return "?";
    }
}

/**
 * @brief 把故障位图转成可读字符串（bit0 欠压 / bit1 过流 / bit2 过温 /
 *        bit3 磁编码 / bit4 HALL / bit5 未标定）
 */
static void motor_fault_str(uint8_t fault, char *buf, size_t buf_size)
{
    if (0U == fault)
    {
        (void)snprintf(buf, buf_size, "ok");
        return;
    }

    size_t used = 0U;
    buf[0]      = '\0';

    auto append = [&](const char *name)
    {
        if (used >= buf_size)
            return;
        const int w = snprintf(buf + used, buf_size - used, "%s%s",
                               (0U == used) ? "" : "|", name);
        if (w > 0)
            used += static_cast<size_t>(w);
    };

    if (0U != (fault & 0x01U)) append("UV");
    if (0U != (fault & 0x02U)) append("OC");
    if (0U != (fault & 0x04U)) append("OT");
    if (0U != (fault & 0x08U)) append("MAG");
    if (0U != (fault & 0x10U)) append("HALL");
    if (0U != (fault & 0x20U)) append("UNCAL");

    if (0U == used)
        (void)snprintf(buf, buf_size, "0x%02X", (unsigned)fault);
}

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
volatile float    g_roll_target_speed = 0.0f;
volatile float    g_roll_fb_val      = 0.0f;

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
        g_roll_fb_val = g_roll_motor->get_current_rotate();//调试相关变量
        g_roll_target_speed = vel;//调试相关变量
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
        // 未校准的轴(i > active)不保持位置, 输出0力矩
        if (i > active) { *ax[i].out = 0.0f; continue; }
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
            PYRO_LOGI("CALI", "all axes done, cali_done=1 -> HOLD");
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
    // 校准用速度环: Yaw用正常rot_pid, Pitch/Roll用cali_pid(大力矩)
    pid_t* cali_rot_pid;
    if (cali_axis == 0)      cali_rot_pid = g_yaw_rot_pid;
    else if (cali_axis == 1) cali_rot_pid = g_pitch_cali_pid;
    else                     cali_rot_pid = g_roll_cali_pid;
    float trq = 0;

    switch (cali_phase) {
    case CALI_FIND_NEG:
        trq = cali_rot_pid->calculate(-CALI_SPEED_TARGET, motor->get_current_rotate());
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
                    PYRO_LOGI("CALI", "axis=%d neg limit pos=%.3f%s",
                              cali_axis, (double)cali_neg_pos,
                              timeout ? " (timeout)" : "");
                }
            } else cali_stall_t = 0;
        }
        break;

    case CALI_FIND_POS:
        trq = cali_rot_pid->calculate(CALI_SPEED_TARGET, motor->get_current_rotate());
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
                    PYRO_LOGI("CALI", "axis=%d pos limit pos=%.3f%s",
                              cali_axis, (double)cali_pos_pos,
                              timeout ? " (timeout)" : "");
                }
            } else cali_stall_t = 0;
        }
        break;

    case CALI_CENTER: {
        float fb = motor->get_current_position() - *offset;
        pid_t* ppos = (cali_axis==0) ? g_yaw_pos_pid : (cali_axis==1) ? g_pitch_pos_pid : g_roll_pos_pid;
        pid_t* prot = (cali_axis==0) ? g_yaw_rot_pid : (cali_axis==1) ? g_pitch_rot_pid : g_roll_rot_pid;
        float vel = ppos->calculate(0.0f, fb);
        trq = clampf(prot->calculate(vel, motor->get_current_rotate()), -CALI_CENTER_TORQUE, CALI_CENTER_TORQUE);
        if (fb < 0.1f && fb > -0.1f) {
            PYRO_LOGI("CALI", "axis=%d centered, next axis=%d",
                      cali_axis, cali_axis + 1);
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

    // 打印三个轴实际下发的力矩 (2s一次)
    static float trq_print_t = 0;
    float now_ms = dwt_drv_t::get_timeline_ms();
    if (now_ms - trq_print_t > 500.0f) {
        trq_print_t = now_ms;
        PYRO_LOGI("CTRL", "torque yaw=%.3f pitch=%.3f roll=%.3f",
                  (double)out.yaw_trq, (double)out.pitch_trq,
                  (double)out.roll_trq);
    }

    // 调试记录
    g_yaw_trq_sent   = out.yaw_trq;
    g_pitch_trq_sent = out.pitch_trq;
    g_roll_trq_sent  = out.roll_trq;
    // 通过USART1发送二进制帧数据，供串口绘图 (20ms一次)
    // 帧格式: 0xAA | float yaw_fb | float yaw_target | float yaw_trq | 0x55
    // 共14字节, 小端序
    static float print_t = 0;
    if (now_ms - print_t > 20.0f) {
        print_t = now_ms;
        union { float f; uint8_t b[4]; } c1, c2, c3;
        c1.f = g_roll_trq_sent;//发送的力矩值
        c2.f = g_roll_target_speed;//速度的期望值
        c3.f = g_roll_fb_val;//速度的反馈值
        uint8_t tx[14];
        tx[0]  = 0xAA;
        tx[1]  = c1.b[0]; tx[2]  = c1.b[1]; tx[3]  = c1.b[2]; tx[4]  = c1.b[3];
        tx[5]  = c2.b[0]; tx[6]  = c2.b[1]; tx[7]  = c2.b[2]; tx[8]  = c2.b[3];
        tx[9]  = c3.b[0]; tx[10] = c3.b[1]; tx[11] = c3.b[2]; tx[12] = c3.b[3];
        tx[13] = 0x55;
        bsp_uart::get_uart1().write(tx, 14, 10);
    }

    // 周期性状态日志 (2s一次)，用于定位"力矩为0"的原因
    static float log_t = 0;
    if (now_ms - log_t > 2000.0f) {
        log_t = now_ms;
        PYRO_LOGD("CTRL",
                  "st=%d cali_done=%d axis=%d ph=%d | trq y=%.3f p=%.3f r=%.3f"
                  " | fb y=%.3f p=%.3f r=%.3f | tgt y=%.3f | txbusy=%lu txerr=%lu",
                  (int)sys_state, (int)cali_done, (int)cali_axis,
                  (int)cali_phase, (double)out.yaw_trq, (double)out.pitch_trq,
                  (double)out.roll_trq, (double)g_yaw_fb_pos,
                  (double)g_pitch_fb_pos, (double)g_roll_fb_pos,
                  (double)g_yaw_target,
                  (unsigned long)can_get_tx_busy_cnt(),
                  (unsigned long)can_get_tx_error_cnt());

        /*
         * 每个小米电机一行诊断信息。
         * on  : 是否收到有效反馈（通信层面）
         * mode: 电机状态机 RESET / CALI / MOTOR
         * flt : 故障位图（UV 欠压 / OC 过流 / OT 过温 / MAG 磁编码 /
         *       HALL / UNCAL 未标定），ok 表示无故障
         * busV: 电机端母线电压，异步刷新，可能是上次读到的值
         *
         * 判读：on=0            -> 完全没通信（接线/供电/波特率）
         *       on=0 且 flt!=ok -> 有通信但电机报故障或未标定
         */
        (void)g_yaw_motor->request_param(
            cybergear_motor_drv_t::param_vbus);
        (void)g_pitch_motor->request_param(
            cybergear_motor_drv_t::param_vbus);
        (void)g_roll_motor->request_param(
            cybergear_motor_drv_t::param_vbus);

        /*
         * 有反馈但不在 MOTOR 态，说明使能帧没有生效（例如开机时总线正在
         * Bus-Off、邮箱满导致使能帧被丢）。这里补发使能；ESTOP 下跳过，
         * 避免把急停的电机重新使能。
         */
        if (sys_state != SYS_ESTOP)
        {
            auto reenable_if_reset = [](cybergear_motor_drv_t *motor)
            {
                if ((motor->get_feedback_count() > 0U) &&
                    (motor->get_mode_state() !=
                     cybergear_motor_drv_t::state_motor))
                {
                    (void)motor->enable();
                }
            };

            reenable_if_reset(g_yaw_motor);
            reenable_if_reset(g_pitch_motor);
            reenable_if_reset(g_roll_motor);
        }

        char fault_buf[32];

        motor_fault_str((uint8_t)g_yaw_motor->get_fault(), fault_buf,
                        sizeof(fault_buf));
        PYRO_LOGD("MOTOR", "yaw   rx=%lu on=%d mode=%-5s flt=%-10s busV=%.1f",
                  (unsigned long)g_yaw_motor->get_feedback_count(),
                  (int)g_yaw_motor->is_online(),
                  motor_mode_name((uint8_t)g_yaw_motor->get_mode_state()),
                  fault_buf, (double)g_yaw_motor->get_bus_voltage());

        motor_fault_str((uint8_t)g_pitch_motor->get_fault(), fault_buf,
                        sizeof(fault_buf));
        PYRO_LOGD("MOTOR", "pitch rx=%lu on=%d mode=%-5s flt=%-10s busV=%.1f",
                  (unsigned long)g_pitch_motor->get_feedback_count(),
                  (int)g_pitch_motor->is_online(),
                  motor_mode_name((uint8_t)g_pitch_motor->get_mode_state()),
                  fault_buf, (double)g_pitch_motor->get_bus_voltage());

        motor_fault_str((uint8_t)g_roll_motor->get_fault(), fault_buf,
                        sizeof(fault_buf));
        PYRO_LOGD("MOTOR", "roll  rx=%lu on=%d mode=%-5s flt=%-10s busV=%.1f",
                  (unsigned long)g_roll_motor->get_feedback_count(),
                  (int)g_roll_motor->is_online(),
                  motor_mode_name((uint8_t)g_roll_motor->get_mode_state()),
                  fault_buf, (double)g_roll_motor->get_bus_voltage());
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

        // 2.1 反馈断流检测（限频打印，避免刷屏）
        {
            static uint32_t last_fb_warn_ms = 0;
            if (!g_yaw_motor->is_online() || !g_pitch_motor->is_online() ||
                !g_roll_motor->is_online())
            {
                if (now - (float)last_fb_warn_ms > 2000.0f)
                {
                    last_fb_warn_ms = (uint32_t)now;
                    PYRO_LOGW("CTRL", "feedback lost: yaw=%d pitch=%d roll=%d",
                              (int)g_yaw_motor->is_online(),
                              (int)g_pitch_motor->is_online(),
                              (int)g_roll_motor->is_online());
                }
            }
        }

        // 3. 每帧输出清零
        memset(&out, 0, sizeof(out));
        out.x_speed_mode = false;
        out.z_speed_mode = false;

        // 4. 处理指令
        // 4.0 指令接收统计（在任务上下文打印，ISR 中不打日志）
        {
            static uint32_t last_frames = 0;
            static uint32_t last_err    = 0;
            uint32_t err_total = g_cmd_rx_err_sof + g_cmd_rx_err_eof +
                                 g_cmd_rx_err_chksum;
            if (g_cmd_rx_frames != last_frames || err_total != last_err)
            {
                last_frames = g_cmd_rx_frames;
                last_err    = err_total;
                PYRO_LOGI("CMD",
                          "rx total=%lu frames=%lu last=%u err(sof=%lu eof=%lu ck=%lu)",
                          (unsigned long)g_cmd_rx_total,
                          (unsigned long)g_cmd_rx_frames,
                          (unsigned)g_cmd_last_cmd,
                          (unsigned long)g_cmd_rx_err_sof,
                          (unsigned long)g_cmd_rx_err_eof,
                          (unsigned long)g_cmd_rx_err_chksum);
            }

        }

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
