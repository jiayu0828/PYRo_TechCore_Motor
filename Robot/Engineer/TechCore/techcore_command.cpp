#include "techcore_command.h"
#include "FreeRTOS.h"
#include "task.h"

// ============================================================
// 姿态动作循环数组定义
// 每次收到指令2, 切换到下一个动作(循环往复)
// 可根据实际需求修改动作参数和数量
// ============================================================
const pose_action_t g_pose_actions[] =
{
    // 动作0: X向后1s, Z向下1s, YPR回中保持
    { -1, 0, -1, 0,  1.5f, 0.0f, 0.0f },
    // 动作1: X向后1s, Z向下1s, Yaw+0.3rad (Pitch/Roll暂未接入, 目标为0)
    { -1, 0, -1, 0,  1.5f, 0.0f, 0.0f },
    // 动作2: X向前0.5s, Z向上0.5s, Yaw-0.3rad
    { +1,  0, +1,  0, -1.5f, 0.0f, 0.0f },
    // 动作3: X向后0.5s, Z向下0.5s, Yaw+0.5rad
    { -1,  0, -1,  0,  -1.5f, 0.0f, 0.0f },
};
const int g_pose_action_count = sizeof(g_pose_actions) / sizeof(g_pose_actions[0]);

// ============================================================
// 接收统计变量定义
// ============================================================
volatile uint32_t g_cmd_rx_total      = 0;
volatile uint32_t g_cmd_rx_frames     = 0;
volatile uint32_t g_cmd_rx_err_sof    = 0;
volatile uint32_t g_cmd_rx_err_eof    = 0;
volatile uint32_t g_cmd_rx_err_chksum = 0;
volatile uint8_t  g_cmd_last_cmd      = 0;
volatile uint32_t g_cmd_last_tick_ms  = 0;

// ============================================================
// 简化协议解析 (4条指令)
// 帧格式(固定4字节):
//   SOF(0xA5) + CMD(1B) + CHKSUM(1B) + EOF(0x5A)
//   CHKSUM = CMD
// ============================================================
int command_recv_process(command_recv_t *cmd, const uint8_t *buf, uint16_t len)
{
    g_cmd_rx_total += len;

    if (nullptr == cmd || nullptr == buf)
        return -1;

    if (len < 4)
        return -1;

    // 校验帧头
    if (0xA5 != buf[0])
    {
        g_cmd_rx_err_sof++;
        return -2;
    }

    // 校验帧尾
    if (0x5A != buf[3])
    {
        g_cmd_rx_err_eof++;
        return -3;
    }

    uint8_t cmd_id  = buf[1];
    uint8_t chksum  = buf[2];

    // 校验校验和
    if (chksum != (cmd_id & 0xFF))
    {
        g_cmd_rx_err_chksum++;
        return -4;
    }

    // 按指令编号设置标志(控制任务消费后清零)
    switch (cmd_id)
    {
        case CMD_CALIBRATE:  // 1 = 校准
            cmd->calibrate_enable = 1;
            break;

        case CMD_POSE_MOVE:  // 2 = 姿态调整
            cmd->pose_move_enable = 1;
            break;

        case CMD_ESTOP:  // 4 = 紧急停止 (最高优先级)
            cmd->estop_enable = 1;
            break;

        case CMD_CAN1_SELFTEST:  // 5 = CAN1 回环自检
            cmd->can1_selftest_enable = 1;
            break;

        case CMD_CAN2_SELFTEST:  // 6 = CAN2 回环自检
            cmd->can2_selftest_enable = 1;
            break;

        default:
            break;
    }

    g_cmd_rx_frames++;
    g_cmd_last_cmd = cmd_id;
    g_cmd_last_tick_ms = (uint32_t)xTaskGetTickCount();

    return 0;
}
