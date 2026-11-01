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
    
    { -1, 1500, -1, 2000,  1.52f, 0.32f, -0.72f },
    //先运动到极端位置
    { +1, 400, +1, 500,  -1.53f, 0.38f, 0.72f },
    //往前面，往上面抬  X：-1100 Y: -1500
    { +1, 800, +1, 600,  1.49f, 0.43f, -0.72f },
    //往前面，往上面抬  X：-300 Y: -900
    { -1, 800, -1, 600,  -1.49f, 0.43f, 0.72f },
    //往后面，往下面降  X: -1100 Y:-1500
    { -1, 300, +1, 600,  -1.43f, 0.35f, 0.73f },
    //往后面，往下面降  X: -300 Y:-900
    //极端的来5个
    { -1, 0, -1, 700,  1.46f, 0.2f, -0.7f },
    // X:-1400 Y:-1600
    { +1, 400, +1, 600,  1.54f, 0.15f, -0.73f },
    //X: -1100 Y: -1000
    { +1, 600, +1, 500,  -1.48f, 0.2f, 0.73f },
    { +1, 400, +1, 500,  -1.52f, 0.5f, +0.72f },
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
