#ifndef CYBERGEAR_MOTOR_DRV_H
#define CYBERGEAR_MOTOR_DRV_H

#include "pyro_motor_base.h"

namespace pyro
{

/**
 * @brief 小米Cybergear微电机驱动
 *
 * 基于CAN 2.0B扩展帧协议，支持MIT运动控制模式、位置模式、速度模式、电流模式。
 * 继承自motor_base_t，与PYRo框架中DJI/DM电机驱动保持一致的接口范式。
 */
class cybergear_motor_drv_t : public motor_base_t
{
  public:
    /**
     * @brief 电机运行模式
     */
    enum run_mode_t
    {
        motion_mode   = 0, ///< MIT运动控制模式（扭矩+位置+速度+Kp+Kd单帧控制）
        position_mode = 1, ///< 位置模式
        speed_mode    = 2, ///< 速度模式
        current_mode  = 3  ///< 电流模式（直接控制Iq）
    };

    /**
     * @brief 故障码（反馈帧ID bit16~21）
     */
    enum fault_code_t
    {
        fault_none              = 0x00,
        fault_undervoltage      = (1 << 0), ///< 欠压故障
        fault_overcurrent       = (1 << 1), ///< 过流故障
        fault_overtemperature   = (1 << 2), ///< 过温故障
        fault_magnetic_encoder  = (1 << 3), ///< 磁编码器故障
        fault_hall_encoder      = (1 << 4), ///< HALL编码器故障
        fault_not_calibrated    = (1 << 5), ///< 未标定
    };

    /**
     * @brief 模式状态（反馈帧ID bit22~23）
     */
    enum mode_state_t
    {
        state_reset   = 0, ///< 复位模式
        state_cali    = 1, ///< 标定模式
        state_motor   = 2, ///< 运行模式
    };

    /**
     * @brief 单参数索引表（部分常用）
     */
    enum param_index_t
    {
        param_run_mode        = 0x7005, ///< 运行模式 uint8
        param_iq_ref          = 0x7006, ///< 电流模式Iq指令 float (-23~23A)
        param_spd_ref         = 0x700A, ///< 速度模式速度指令 float (-30~30 rad/s)
        param_imit_torque     = 0x700B, ///< 扭矩限制 float (0~12 Nm)
        param_cur_filt_gain   = 0x7014, ///< 电流滤波增益 float (0~1.0, default 0.1)
        param_loc_ref         = 0x7016, ///< 位置模式角度指令 float (rad)
        param_limit_spd       = 0x7017, ///< 位置模式速度限制 float (0~30 rad/s)
        param_limit_cur       = 0x7018, ///< 速度/位置模式电流限制 float (0~23A)
        param_mech_vel        = 0x701B, ///< 负载端速度 float (R)
        param_vbus            = 0x701C, ///< 母线电压 float (V, R)
        param_spd_kp          = 0x701F, ///< 速度环Kp float (default 1.0)
        param_spd_ki          = 0x7020, ///< 速度环Ki float (default 0.002)
    };

    /**
     * @brief 构造函数
     * @param motor_id 电机CAN ID（默认0x7F，可通过set_motor_id修改）
     * @param master_id 主机CAN ID（用于标识发送方，建议0x00）
     * @param which CAN总线实例
     */
    cybergear_motor_drv_t(uint8_t motor_id, uint8_t master_id,
                           bsp_can::which_can which);
    ~cybergear_motor_drv_t() override;

    // ===== motor_base_t 接口实现 =====
    status_t enable() override;
    status_t disable() override;
    status_t update_feedback() override;
    status_t send_torque(float torque) override;

    // ===== MIT运动控制模式（通信类型1）=====
    /**
     * @brief MIT模式单帧控制
     * @param torque 前馈扭矩 (-12~12 Nm)
     * @param position 目标位置 (-4π~4π rad)
     * @param velocity 目标速度 (-30~30 rad/s)
     * @param kp 位置环比例增益 (0~500)
     * @param kd 位置环微分增益 (0~5)
     */
    status_t send_motion_control(float torque, float position, float velocity,
                                  float kp, float kd);

    // ===== 运行模式切换 =====
    status_t set_run_mode(run_mode_t mode);
    run_mode_t get_run_mode() const { return _run_mode; }

    // ===== 位置模式 =====
    status_t set_position(float position);            ///< 设置目标位置(rad)
    status_t set_position_limit(float speed_limit,    ///< 位置模式限速(rad/s)
                                 float current_limit); ///< 位置模式限流(A)

    // ===== 速度模式 =====
    status_t set_speed(float speed);                   ///< 设置目标速度(rad/s)
    status_t set_speed_limit(float current_limit);     ///< 速度模式限流(A)

    // ===== 电流模式 =====
    status_t set_current(float current);               ///< 设置Iq电流(-23~23A)

    // ===== 参数配置 =====
    status_t set_torque_limit(float torque);           ///< 扭矩限制(0~12 Nm)
    status_t set_speed_pid(float kp, float ki);        ///< 速度环PID参数
    status_t set_current_filter_gain(float gain);      ///< 电流滤波增益(0~1.0)

    // ===== 状态查询 =====
    float get_bus_voltage() const { return _bus_voltage; }
    fault_code_t get_fault() const { return _fault; }
    mode_state_t get_mode_state() const { return _mode_state; }
    uint8_t get_motor_id() const { return _motor_id; }

    // ===== 参数读取（同步/异步）=====
    /**
     * @brief 同步读取单个参数（发送读命令后等待回应）
     * @param index 参数索引（见param_index_t）
     * @param out_value 输出参数值
     * @param timeout_ms 超时时间（默认50ms）
     * @return PYRO_OK=成功, PYRO_TIMEOUT=超时, PYRO_ERROR=发送失败
     */
    status_t read_param_sync(param_index_t index, float &out_value,
                              uint32_t timeout_ms = 50);

    /**
     * @brief 读取母线电压（通过读参数0x701C，更新_bus_voltage）
     * @param timeout_ms 超时时间
     * @return PYRO_OK=成功, 其他=失败
     */
    status_t read_bus_voltage(uint32_t timeout_ms = 50);

    /**
     * @brief 获取最近一次异步读参数的结果
     * @param out_index 输出参数索引
     * @param out_value 输出参数值
     * @return true=有新数据, false=无新数据
     */
    bool get_last_read_param(param_index_t &out_index, float &out_value);

    // ===== 其他操作 =====
    status_t clear_error();           ///< 清除故障
    status_t set_zero_position();     ///< 设置当前位置为机械零位（断电丢失）
    status_t set_motor_id(uint8_t new_id); ///< 修改电机CAN ID（立即生效）

    // ===== 总线扫描（发现未知ID的电机）=====
    /**
     * @brief 扫描CAN总线上的Cybergear电机
     *
     * 逐个ID发送"获取设备ID"命令（通信类型0），有回应的即为在线电机。
     * 扫描期间会临时接管扩展帧回调，扫描结束后恢复。
     *
     * @param which CAN总线实例
     * @param out_ids 输出数组，存放找到的电机ID
     * @param max_count out_ids数组的最大容量
     * @param master_id 主机CAN ID（默认0x00）
     * @param start_id 扫描起始ID（默认0x01）
     * @param end_id 扫描结束ID（默认0x7F，出厂默认ID）
     * @param timeout_ms 每个ID的等待超时（默认10ms）
     * @return 找到的电机数量（不超过max_count）
     */
    static uint8_t scan_bus(bsp_can::which_can which,
                             uint8_t *out_ids,
                             uint8_t max_count,
                             uint8_t master_id = 0x00,
                             uint8_t start_id = 0x01,
                             uint8_t end_id = 0x7F,
                             uint32_t timeout_ms = 10);

    // ===== 扩展帧接收回调（由CAN驱动调用）=====
    static void extended_rx_callback(uint32_t id, const uint8_t *data);

  private:
    uint8_t _motor_id;
    uint8_t _master_id;
    run_mode_t _run_mode;
    fault_code_t _fault;
    mode_state_t _mode_state;
    float _bus_voltage;

    // MIT模式运行时参数
    float _runtime_kp;
    float _runtime_kd;

    // 反馈新鲜度标志
    volatile bool _feedback_fresh;

    // 异步读参数状态（类型18回应帧解析后保存）
    volatile param_index_t _read_param_index;  ///< 最近一次读取的参数索引
    volatile float         _read_param_value;   ///< 最近一次读取的参数值
    volatile bool          _read_param_ready;   ///< 读参数回应已就绪标志

    // 实例注册表（用于扩展帧回调分发）—— 固定数组，避免动态内存分配
    static constexpr uint8_t MAX_INSTANCES = 8;
    static cybergear_motor_drv_t *_instance_list[MAX_INSTANCES];
    static uint8_t _instance_count;

    // 扫描用的临时接收状态
    static volatile bool _scan_rx_flag;
    static volatile uint32_t _scan_rx_id;
    static volatile uint8_t _scan_rx_data[8];
    static void scan_rx_callback(uint32_t id, const uint8_t *data);

    // ===== 物理范围常量 =====
    static constexpr float P_MIN  = -4.0f * PI;
    static constexpr float P_MAX  =  4.0f * PI;
    static constexpr float V_MIN  = -30.0f;
    static constexpr float V_MAX  =  30.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;
    static constexpr float T_MIN  = -12.0f;
    static constexpr float T_MAX  =  12.0f;
    static constexpr float I_MIN  = -23.0f;
    static constexpr float I_MAX  =  23.0f;

    // ===== 通信类型定义 =====
    static constexpr uint8_t CMD_GET_DEVICE_ID   = 0;
    static constexpr uint8_t CMD_MOTION_CONTROL  = 1;
    static constexpr uint8_t CMD_MOTOR_FEEDBACK  = 2;
    static constexpr uint8_t CMD_MOTOR_ENABLE    = 3;
    static constexpr uint8_t CMD_MOTOR_STOP      = 4;
    static constexpr uint8_t CMD_SET_ZERO        = 6;
    static constexpr uint8_t CMD_SET_CAN_ID      = 7;
    static constexpr uint8_t CMD_READ_PARAM      = 17;
    static constexpr uint8_t CMD_WRITE_PARAM     = 18;
    static constexpr uint8_t CMD_FAULT_FEEDBACK  = 21;
    static constexpr uint8_t CMD_SET_BAUDRATE    = 22;

    // ===== 辅助函数 =====
    static uint32_t build_ext_id(uint8_t type, uint16_t data, uint8_t target_id);

    status_t write_param_float(param_index_t index, float value);
    status_t write_param_u8(param_index_t index, uint8_t value);
    status_t read_param(param_index_t index);

    void parse_feedback(uint32_t id, const uint8_t *data);

    static int float_to_uint(float x, float x_min, float x_max, int bits);
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

} // namespace pyro

#endif // CYBERGEAR_MOTOR_DRV_H
