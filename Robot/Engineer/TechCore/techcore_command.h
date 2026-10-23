#ifndef TECHCORE_COMMAND_H
#define TECHCORE_COMMAND_H

#include <cstdint>

/**
 * @brief 科技核心简化串口指令协议 (4条指令)
 *
 * 帧格式(固定4字节):
 *   SOF(1B) + CMD(1B) + CHKSUM(1B) + EOF(1B)
 *   SOF    = 0xA5  (帧头)
 *   CMD    = 指令编号 (1~4)
 *   CHKSUM = CMD (单字节校验和)
 *   EOF    = 0x5A  (帧尾)
 *
 * 指令集:
 *   1 = CALIBRATE  : 校准(小米三轴双向找限位+回中, X/Z到最前端最下端), 完成后自动HOLD
 *   2 = POSE_MOVE  : 姿态调整(循环切换动作数组, X/Z移动+YPR回中+旋转), 完成后自动HOLD
 *   4 = ESTOP      : 紧急停止, 无视一切指令, 全部电机失能
 *   5 = CAN1_TEST  : CAN1 回环自检(诊断软件/接线), 会短暂中断该路电机通信
 *   6 = CAN2_TEST  : CAN2 回环自检(诊断软件/接线), 会短暂中断该路电机通信
 *   (指令3已废弃, HOLD模式由校准/姿态调整完成后自动进入, 收到新指令自动退出)
 *
 * 硬件链路: 蓝牙模块 TX → C板 USART1 RX(PB7)
 *           蓝牙模块 RX → C板 USART1 TX(PA9)
 *           波特率 115200 8N1, DMA接收
 */
typedef struct
{
    uint8_t  calibrate_enable;   ///< 1=触发校准(消费后清零) (指令1)
    uint8_t  pose_move_enable;   ///< 1=触发姿态调整(消费后清零) (指令2)
    uint8_t  hold_enable;        ///< 1=切换保持模式(大力矩) (指令3)
    uint8_t  estop_enable;       ///< 1=紧急停止(指令4, 最高优先级)
    uint8_t  can1_selftest_enable; ///< 1=CAN1回环自检(消费后清零) (指令5)
    uint8_t  can2_selftest_enable; ///< 1=CAN2回环自检(消费后清零) (指令6)
    float    x;                   ///< X轴目标位置(mm) (预留)
    float    y;                   ///< Y轴目标位置(预留)
    float    z;                   ///< Z轴目标位置(mm) (预留)
    float    theta;               ///< Yaw目标角度(rad) (预留)
    float    phi;                 ///< Pitch目标角度(rad) (预留)
    float    alpha;               ///< Roll目标角度(rad) (预留)
}
command_recv_t;

/// 全局接收命令(由UART回调写入，控制任务消费)
extern command_recv_t cmd_recv;

// ============================================================
// 接收统计(调试器可查看)
// ============================================================
extern volatile uint32_t g_cmd_rx_total;
extern volatile uint32_t g_cmd_rx_frames;
extern volatile uint32_t g_cmd_rx_err_sof;
extern volatile uint32_t g_cmd_rx_err_eof;
extern volatile uint32_t g_cmd_rx_err_chksum;
extern volatile uint8_t  g_cmd_last_cmd;
extern volatile uint32_t g_cmd_last_tick_ms;

// ============================================================
// 指令编号定义
// ============================================================
#define CMD_CALIBRATE   1   ///< 校准
#define CMD_POSE_MOVE   2   ///< 姿态调整
#define CMD_HOLD        3   ///< 保持姿态+大力矩
#define CMD_ESTOP       4   ///< 紧急停止
#define CMD_CAN1_SELFTEST 5 ///< CAN1 回环自检
#define CMD_CAN2_SELFTEST 6 ///< CAN2 回环自检

// ============================================================
// 小米电机力矩上限 (两档，Yaw/Pitch/Roll三轴统一)
// NORMAL: 所有非Hold状态(校准/IDLE/姿态调整)
// HOLD:   指令3保持姿态时使用
// 大疆电机不做软件力矩限幅，完全依赖驱动层内部上限
// ============================================================
#define TORQUE_LIMIT_NORMAL  0.2f   ///< Normal模式力矩上限(N·m)，三轴统一
#define TORQUE_LIMIT_HOLD    9.0f   ///< Hold模式力矩上限(N·m)，三轴统一

// ============================================================
// 大疆电机(X/Z)移动速度与方向映射
// 速度单位: rad/s (转子转速)
// direction系数在初始化时设置(X=-1, Z=-1)，速度宏已含方向
// ============================================================
#define MOVE_SPEED_RAD       200.0f ///< X/Z轴基础移动速度(rad/s)
#define X_SPEED_FORWARD      ( MOVE_SPEED_RAD)  ///< X轴向前移动速度
#define X_SPEED_BACKWARD     ( -MOVE_SPEED_RAD)  ///< X轴向后移动速度
#define Z_SPEED_UPWARD       ( MOVE_SPEED_RAD)  ///< Z轴向上移动速度
#define Z_SPEED_DOWNWARD     (-MOVE_SPEED_RAD)  ///< Z轴向下移动速度

// ============================================================
// 校准配置 (小米三轴向速控速找限位 + 回中验证)
// ============================================================
#define CALI_SPEED_TARGET     0.5f   ///< 校准目标速度(rad/s)
#define CALI_MAX_TRQ          12.0f   ///< 校准速度环力矩上限(N·m)
#define CALI_STALL_TRQ_THRESH 2.0f   ///< 堵转力矩阈值(N·m)
#define CALI_STALL_SPEED      0.1f  ///< 堵转速度阈值(rad/s)
#define CALI_STALL_TIME_MS    500.0f ///< 堵转判定持续时间(ms)
#define CALI_MAX_TIME_MS      6000.0f ///< 单方向超时时间(ms)
#define CALI_CENTER_TORQUE    12.0f   ///< 回中力矩上限(N·m)
// ============================================================
// 姿态调整预设参数 (指令2使用)
// 指令2触发后, 在动作数组中循环切换, 每次执行一个动作
// ============================================================
#define POSE_YPR_SPEED        0.5f    ///< YPR插值速度(rad/s)

/**
 * @brief 单个姿态动作定义
 *
 * 方向定义(用户视角):
 *   x_dir: +1=向前, -1=向后
 *   z_dir: +1=向上, -1=向下
 *
 * YPR动作流程: 先回中(到0) → 再旋转到 yaw/pitch/roll_target → 保持
 */
typedef struct
{
    int8_t  x_dir;          ///< X方向: +1=向前, -1=向后
    uint16_t x_time_ms;     ///< X移动持续时间(ms)
    int8_t  z_dir;          ///< Z方向: +1=向上, -1=向下
    uint16_t z_time_ms;     ///< Z移动持续时间(ms)
    float   yaw_target;     ///< Yaw目标角度(rad, 回中后旋转到此)
    float   pitch_target;   ///< Pitch目标角度(rad)
    float   roll_target;    ///< Roll目标角度(rad)
}
pose_action_t;

/// 姿态动作循环数组 (每次指令2切换到下一个, 循环往复)
/// 可根据实际需求修改动作参数和数量
extern const pose_action_t g_pose_actions[];
extern const int g_pose_action_count;

// ============================================================
// 堵转保护配置 (仅小米电机使用，大疆电机不加)
// ============================================================
#define STALL_SPEED_THRESHOLD  0.2f   ///< 堵转速度阈值(rad/s)
#define STALL_TORQUE_THRESHOLD 0.1f   ///< 堵转力矩阈值(N·m)
#define STALL_DETECT_TIME_MS   300.0f ///< 堵转判定时间(ms)
#define STALL_TORQUE_REDUCED   0.1f   ///< 堵转后t降矩值(N·m)
#define STALL_RELEASE_SPEED    1.0f   ///< 解除堵转速度阈值(rad/s)

// ============================================================
// 活动范围硬限位 (rad)
// ============================================================
#define PITCH_LIMIT_MIN  -1.57080f
#define PITCH_LIMIT_MAX   1.57080f
#define ROLL_LIMIT_MIN   -1.04720f
#define ROLL_LIMIT_MAX    1.04720f

/**
 * @brief 解析一帧简化协议数据
 */
int command_recv_process(command_recv_t *cmd, const uint8_t *buf, uint16_t len);

#endif // TECHCORE_COMMAND_H
