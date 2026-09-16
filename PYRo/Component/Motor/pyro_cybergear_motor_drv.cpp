#include "pyro_cybergear_motor_drv.h"
#include <cstring>

namespace pyro
{

// ===== 静态成员初始化 =====
cybergear_motor_drv_t *cybergear_motor_drv_t::_instance_list[MAX_INSTANCES] = {nullptr};
uint8_t cybergear_motor_drv_t::_instance_count = 0;
volatile bool cybergear_motor_drv_t::_scan_rx_flag = false;
volatile uint32_t cybergear_motor_drv_t::_scan_rx_id = 0;
volatile uint8_t cybergear_motor_drv_t::_scan_rx_data[8] = {0};

// ============================================================
// 构造与析构
// ============================================================
cybergear_motor_drv_t::cybergear_motor_drv_t(uint8_t motor_id,
                                                 uint8_t master_id,
                                                 bsp_can::which_can which)
    : motor_base_t(which),
      _motor_id(motor_id),
      _master_id(master_id),
      _run_mode(motion_mode),
      _fault(fault_none),
      _mode_state(state_reset),
      _bus_voltage(0.0f),
      _runtime_kp(0.0f),
      _runtime_kd(0.0f),
      _feedback_fresh(false),
      _read_param_index(param_run_mode),
      _read_param_value(0.0f),
      _read_param_ready(false)
{
    // 注册到实例列表，用于扩展帧回调分发（固定数组，避免动态内存）
    if (_instance_count < MAX_INSTANCES)
    {
        _instance_list[_instance_count++] = this;
    }
}

cybergear_motor_drv_t::~cybergear_motor_drv_t()
{
    // 从实例列表移除（固定数组，用最后一个元素填补空缺）
    for (uint8_t i = 0; i < _instance_count; i++)
    {
        if (_instance_list[i] == this)
        {
            _instance_list[i] = _instance_list[_instance_count - 1];
            _instance_list[_instance_count - 1] = nullptr;
            _instance_count--;
            break;
        }
    }
}

// ============================================================
// motor_base_t 接口实现
// ============================================================
status_t cybergear_motor_drv_t::enable()
{
    // 通信类型3：电机使能
    // ID结构: type=3, data=master_id<<8, target=motor_id
    uint32_t ext_id = build_ext_id(CMD_MOTOR_ENABLE,
                                    static_cast<uint16_t>(_master_id << 8),
                                    _motor_id);
    std::array<uint8_t, 8> data{};
    data.fill(0);

    if (PYRO_OK != _can_drv->send_extended_msg(ext_id, data.data()))
        return PYRO_ERROR;

    _enable = true;
    return PYRO_OK;
}

status_t cybergear_motor_drv_t::disable()
{
    // 通信类型4：电机停止
    uint32_t ext_id = build_ext_id(CMD_MOTOR_STOP,
                                    static_cast<uint16_t>(_master_id << 8),
                                    _motor_id);
    std::array<uint8_t, 8> data{};
    data.fill(0);

    if (PYRO_OK != _can_drv->send_extended_msg(ext_id, data.data()))
        return PYRO_ERROR;

    _enable = false;
    return PYRO_OK;
}

status_t cybergear_motor_drv_t::update_feedback()
{
    // Cybergear反馈帧由扩展帧回调自动解析并更新成员变量
    // 此处检查反馈新鲜度和在线状态
    if (!_feedback_fresh)
    {
        _online = false;
        return PYRO_ERROR;
    }

    _feedback_fresh = false;
    _online = (_mode_state == state_motor);
    return PYRO_OK;
}

status_t cybergear_motor_drv_t::send_torque(float torque)
{
    // 基类接口：以MIT模式发送纯扭矩控制（位置/速度为0，Kp/Kd为0）
    return send_motion_control(torque, 0.0f, 0.0f, 0.0f, 0.0f);
}

// ============================================================
// MIT运动控制模式（通信类型1）
// ============================================================
status_t cybergear_motor_drv_t::send_motion_control(float torque, float position,
                                                      float velocity, float kp,
                                                      float kd)
{
    // 扭矩编码到ID的data区（16bit, -12~12 Nm）
    uint16_t torque_int =
        static_cast<uint16_t>(float_to_uint(torque, T_MIN, T_MAX, 16));

    // 构建扩展帧ID: type=1, data=torque_int, target=motor_id
    uint32_t ext_id = build_ext_id(CMD_MOTION_CONTROL, torque_int, _motor_id);

    // 8字节数据区：位置(16bit) + 速度(16bit) + Kp(16bit) + Kd(16bit)
    uint16_t pos_int =
        static_cast<uint16_t>(float_to_uint(position, P_MIN, P_MAX, 16));
    uint16_t vel_int =
        static_cast<uint16_t>(float_to_uint(velocity, V_MIN, V_MAX, 16));
    uint16_t kp_int =
        static_cast<uint16_t>(float_to_uint(kp, KP_MIN, KP_MAX, 16));
    uint16_t kd_int =
        static_cast<uint16_t>(float_to_uint(kd, KD_MIN, KD_MAX, 16));

    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(pos_int >> 8);
    data[1] = static_cast<uint8_t>(pos_int & 0xFF);
    data[2] = static_cast<uint8_t>(vel_int >> 8);
    data[3] = static_cast<uint8_t>(vel_int & 0xFF);
    data[4] = static_cast<uint8_t>(kp_int >> 8);
    data[5] = static_cast<uint8_t>(kp_int & 0xFF);
    data[6] = static_cast<uint8_t>(kd_int >> 8);
    data[7] = static_cast<uint8_t>(kd_int & 0xFF);

    if (PYRO_OK != _can_drv->send_extended_msg(ext_id, data.data()))
        return PYRO_ERROR;

    return PYRO_OK;
}

// ============================================================
// 运行模式切换
// ============================================================
status_t cybergear_motor_drv_t::set_run_mode(run_mode_t mode)
{
    status_t ret = write_param_u8(param_run_mode, static_cast<uint8_t>(mode));
    if (PYRO_OK == ret)
    {
        _run_mode = mode;
    }
    return ret;
}

// ============================================================
// 位置模式
// ============================================================
status_t cybergear_motor_drv_t::set_position(float position)
{
    return write_param_float(param_loc_ref, position);
}

status_t cybergear_motor_drv_t::set_position_limit(float speed_limit,
                                                     float current_limit)
{
    status_t ret = write_param_float(param_limit_spd, speed_limit);
    if (PYRO_OK != ret)
        return ret;
    return write_param_float(param_limit_cur, current_limit);
}

// ============================================================
// 速度模式
// ============================================================
status_t cybergear_motor_drv_t::set_speed(float speed)
{
    return write_param_float(param_spd_ref, speed);
}

status_t cybergear_motor_drv_t::set_speed_limit(float current_limit)
{
    return write_param_float(param_limit_cur, current_limit);
}

// ============================================================
// 电流模式
// ============================================================
status_t cybergear_motor_drv_t::set_current(float current)
{
    return write_param_float(param_iq_ref, current);
}

// ============================================================
// 参数配置
// ============================================================
status_t cybergear_motor_drv_t::set_torque_limit(float torque)
{
    return write_param_float(param_imit_torque, torque);
}

status_t cybergear_motor_drv_t::set_speed_pid(float kp, float ki)
{
    status_t ret = write_param_float(param_spd_kp, kp);
    if (PYRO_OK != ret)
        return ret;
    return write_param_float(param_spd_ki, ki);
}

status_t cybergear_motor_drv_t::set_current_filter_gain(float gain)
{
    return write_param_float(param_cur_filt_gain, gain);
}

// ============================================================
// 其他操作
// ============================================================
status_t cybergear_motor_drv_t::clear_error()
{
    // 通信类型4，数据区Byte[0]=1表示清除故障
    uint32_t ext_id = build_ext_id(CMD_MOTOR_STOP,
                                    static_cast<uint16_t>(_master_id << 8),
                                    _motor_id);
    std::array<uint8_t, 8> data{};
    data.fill(0);
    data[0] = 1; // 清除故障标志

    return _can_drv->send_extended_msg(ext_id, data.data());
}

status_t cybergear_motor_drv_t::set_zero_position()
{
    // 通信类型6：设置机械零位
    uint32_t ext_id = build_ext_id(CMD_SET_ZERO,
                                    static_cast<uint16_t>(_master_id << 8),
                                    _motor_id);
    std::array<uint8_t, 8> data{};
    data.fill(0);
    data[0] = 1;

    return _can_drv->send_extended_msg(ext_id, data.data());
}

status_t cybergear_motor_drv_t::set_motor_id(uint8_t new_id)
{
    // 通信类型7：修改电机CAN ID
    // ID结构: type=7, data=(new_id<<8 | master_id), target=current_motor_id
    uint16_t data_field =
        static_cast<uint16_t>((new_id << 8) | _master_id);
    uint32_t ext_id = build_ext_id(CMD_SET_CAN_ID, data_field, _motor_id);

    std::array<uint8_t, 8> data{};
    data.fill(0);

    status_t ret = _can_drv->send_extended_msg(ext_id, data.data());
    if (PYRO_OK == ret)
    {
        _motor_id = new_id;
    }
    return ret;
}

// ============================================================
// 扩展帧接收回调（由CAN驱动调用）
// ============================================================
void cybergear_motor_drv_t::extended_rx_callback(uint32_t id,
                                                    const uint8_t *data)
{
    // 解析29位扩展帧ID
    uint8_t type = static_cast<uint8_t>((id >> 24) & 0x1F);
    uint8_t motor_id = static_cast<uint8_t>(id & 0xFF);

    // 只处理反馈帧（类型2）、故障帧（类型21）、参数读写回应（类型18）
    if (type != CMD_MOTOR_FEEDBACK &&
        type != CMD_FAULT_FEEDBACK &&
        type != CMD_WRITE_PARAM)
        return;

    // 根据电机ID查找对应实例（固定数组，按实际数量遍历）
    for (uint8_t i = 0; i < _instance_count; i++)
    {
        cybergear_motor_drv_t *motor = _instance_list[i];
        if (motor != nullptr && motor->_motor_id == motor_id)
        {
            if (type == CMD_MOTOR_FEEDBACK)
            {
                motor->parse_feedback(id, data);
            }
            else if (type == CMD_WRITE_PARAM)
            {
                // ===== 参数读写回应帧（类型18）=====
                // 扩展帧ID: type=18, data=参数索引(16bit), target=电机ID
                // 数据区: Byte0~1=参数索引, Byte4~7=参数值(float, 小端)
                uint16_t param_idx =
                    static_cast<uint16_t>((data[0] << 8) | data[1]);
                uint32_t float_bits =
                    static_cast<uint32_t>(data[4]) |
                    (static_cast<uint32_t>(data[5]) << 8) |
                    (static_cast<uint32_t>(data[6]) << 16) |
                    (static_cast<uint32_t>(data[7]) << 24);
                float param_value;
                std::memcpy(&param_value, &float_bits, sizeof(float));

                motor->_read_param_index =
                    static_cast<param_index_t>(param_idx);
                motor->_read_param_value = param_value;
                motor->_read_param_ready = true;

                // 如果读取的是母线电压，同步更新_bus_voltage
                if (param_idx == param_vbus)
                {
                    motor->_bus_voltage = param_value;
                }
            }
            // 故障帧(类型21)可在此扩展处理
            break;
        }
    }
}

// ============================================================
// 私有辅助函数
// ============================================================
uint32_t cybergear_motor_drv_t::build_ext_id(uint8_t type, uint16_t data,
                                               uint8_t target_id)
{
    // 29位扩展帧ID:
    // bit28~24: type (5bit)
    // bit23~8:  data (16bit)
    // bit7~0:   target_id (8bit)
    return (static_cast<uint32_t>(type & 0x1F) << 24) |
           (static_cast<uint32_t>(data & 0xFFFF) << 8) |
           (static_cast<uint32_t>(target_id & 0xFF));
}

void cybergear_motor_drv_t::parse_feedback(uint32_t id, const uint8_t *data)
{
    // ===== 解析29位扩展帧ID中的状态信息 =====
    // ID结构: bit28~24=type, bit23~8=data_area, bit7~0=motor_id
    // data_area (bit23~8):
    //   bit8~15:  电机CAN_ID
    //   bit16~21: 故障信息 (bit21=未标定, bit20=HALL故障, bit19=磁编码故障,
    //                        bit18=过温, bit17=过流, bit16=欠压)
    //   bit22~23: 模式状态 (0=复位, 1=标定, 2=运行)

    uint16_t data_area = static_cast<uint16_t>((id >> 8) & 0xFFFF);
    uint8_t fault_bits = static_cast<uint8_t>((data_area >> 8) & 0x3F); // bit16~21
    uint8_t mode_bits  = static_cast<uint8_t>((data_area >> 14) & 0x03); // bit22~23

    _fault      = static_cast<fault_code_t>(fault_bits);
    _mode_state = static_cast<mode_state_t>(mode_bits);

    // ===== 解析8字节数据区 =====
    // Byte0~1: 当前角度 (0~65535 → -4π~4π rad)
    // Byte2~3: 当前角速度 (0~65535 → -30~30 rad/s)
    // Byte4~5: 当前扭矩 (0~65535 → -12~12 Nm)
    // Byte6~7: 当前温度 (°C × 10)

    uint16_t pos_int =
        static_cast<uint16_t>((data[0] << 8) | data[1]);
    uint16_t vel_int =
        static_cast<uint16_t>((data[2] << 8) | data[3]);
    uint16_t tor_int =
        static_cast<uint16_t>((data[4] << 8) | data[5]);
    uint16_t temp_int =
        static_cast<uint16_t>((data[6] << 8) | data[7]);

    _current_position = uint_to_float(pos_int, P_MIN, P_MAX, 16);
    _current_rotate   = uint_to_float(vel_int, V_MIN, V_MAX, 16);
    _current_torque   = uint_to_float(tor_int, T_MIN, T_MAX, 16);
    _temperature      = static_cast<int8_t>(temp_int / 10);

    // 标记反馈新鲜
    _feedback_fresh = true;
}

status_t cybergear_motor_drv_t::write_param_float(param_index_t index,
                                                    float value)
{
    // 通信类型18：单参数写入
    // 数据区: Byte0~1=index, Byte2~3=0, Byte4~7=float(小端)
    uint32_t ext_id = build_ext_id(CMD_WRITE_PARAM,
                                    static_cast<uint16_t>(_master_id << 8),
                                    _motor_id);

    std::array<uint8_t, 8> data{};
    data.fill(0);
    data[0] = static_cast<uint8_t>(index >> 8);
    data[1] = static_cast<uint8_t>(index & 0xFF);

    // float按小端写入Byte4~7
    uint32_t float_bits;
    std::memcpy(&float_bits, &value, sizeof(float));
    data[4] = static_cast<uint8_t>(float_bits & 0xFF);
    data[5] = static_cast<uint8_t>((float_bits >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>((float_bits >> 16) & 0xFF);
    data[7] = static_cast<uint8_t>((float_bits >> 24) & 0xFF);

    return _can_drv->send_extended_msg(ext_id, data.data());
}

status_t cybergear_motor_drv_t::write_param_u8(param_index_t index,
                                                 uint8_t value)
{
    // 通信类型18：单参数写入（1字节数据在Byte4）
    uint32_t ext_id = build_ext_id(CMD_WRITE_PARAM,
                                    static_cast<uint16_t>(_master_id << 8),
                                    _motor_id);

    std::array<uint8_t, 8> data{};
    data.fill(0);
    data[0] = static_cast<uint8_t>(index >> 8);
    data[1] = static_cast<uint8_t>(index & 0xFF);
    data[4] = value;

    return _can_drv->send_extended_msg(ext_id, data.data());
}

status_t cybergear_motor_drv_t::read_param(param_index_t index)
{
    // 通信类型17：单参数读取
    uint32_t ext_id = build_ext_id(CMD_READ_PARAM,
                                    static_cast<uint16_t>(_master_id << 8),
                                    _motor_id);

    std::array<uint8_t, 8> data{};
    data.fill(0);
    data[0] = static_cast<uint8_t>(index >> 8);
    data[1] = static_cast<uint8_t>(index & 0xFF);

    return _can_drv->send_extended_msg(ext_id, data.data());
}

// ============================================================
// 静态工具函数
// ============================================================
int cybergear_motor_drv_t::float_to_uint(float x, float x_min, float x_max,
                                           int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return static_cast<int>((x - offset) *
                            (static_cast<float>((1 << bits) - 1)) / span);
}

float cybergear_motor_drv_t::uint_to_float(int x_int, float x_min,
                                             float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return static_cast<float>(x_int) * span /
               static_cast<float>((1 << bits) - 1) + offset;
}

// ============================================================
// 总线扫描（发现未知ID的电机）
// ============================================================
void cybergear_motor_drv_t::scan_rx_callback(uint32_t id, const uint8_t *data)
{
    _scan_rx_id = id;
    std::memcpy((void *)_scan_rx_data, data, 8);
    _scan_rx_flag = true;
}

uint8_t
cybergear_motor_drv_t::scan_bus(bsp_can::which_can which,
                                  uint8_t *out_ids, uint8_t max_count,
                                  uint8_t master_id, uint8_t start_id,
                                  uint8_t end_id, uint32_t timeout_ms)
{
    uint8_t found_count = 0;

    if (nullptr == out_ids || 0 == max_count)
        return 0;

    can_drv_t *can = bsp_can::get_can(which);
    if (nullptr == can)
        return 0;

    // 注册扫描用的临时回调（会覆盖已注册的回调，扫描结束后需恢复）
    can->register_extended_rx_callback(scan_rx_callback);

    uint8_t tx_data[8] = {0};

    // 逐个ID发送"获取设备ID"命令（通信类型0）
    for (uint16_t id = start_id; id <= end_id; id++)
    {
        _scan_rx_flag = false;

        // 构建扩展帧ID: type=0, data=master_id<<8, target=id
        uint32_t ext_id = build_ext_id(CMD_GET_DEVICE_ID,
                                        static_cast<uint16_t>(master_id << 8),
                                        static_cast<uint8_t>(id));
        // 发送失败则跳过该ID，避免总线异常时卡死
        if (PYRO_OK != can->send_extended_msg(ext_id, tx_data))
        {
            continue;
        }

        // 等待回应（超时timeout_ms）
        const TickType_t start_tick = xTaskGetTickCount();
        while (xTaskGetTickCount() - start_tick < pdMS_TO_TICKS(timeout_ms))
        {
            if (_scan_rx_flag)
            {
                // 验证回应帧：
                //   bit28~24 = 通信类型（必须=0，获取设备ID）
                //   bit15~8  = 回应的电机ID（必须==当前扫描的id，防止延迟帧/广播帧误判）
                //   bit7~0   = 目标地址（必须=0xFE，广播回应）
                uint8_t resp_type     = static_cast<uint8_t>((_scan_rx_id >> 24) & 0x1F);
                uint8_t resp_motor_id = static_cast<uint8_t>((_scan_rx_id >> 8) & 0xFF);
                uint8_t resp_target   = static_cast<uint8_t>(_scan_rx_id & 0xFF);
                if (resp_type == CMD_GET_DEVICE_ID &&
                    resp_target == 0xFE &&
                    resp_motor_id == static_cast<uint8_t>(id))
                {
                    // 去重：检查该ID是否已经被记录过
                    bool already_found = false;
                    for (uint8_t i = 0; i < found_count; i++)
                    {
                        if (out_ids[i] == static_cast<uint8_t>(id))
                        {
                            already_found = true;
                            break;
                        }
                    }
                    if (!already_found && found_count < max_count)
                    {
                        out_ids[found_count] = static_cast<uint8_t>(id);
                        found_count++;
                    }
                }
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // 扫描结束，恢复正常的扩展帧回调（用于电机反馈分发）
    can->register_extended_rx_callback(extended_rx_callback);

    return found_count;
}

// ============================================================
// 同步参数读取（新增功能）
// ============================================================
status_t cybergear_motor_drv_t::read_param_sync(param_index_t index,
                                                   float &out_value,
                                                   uint32_t timeout_ms)
{
    // 清除上一次的读参数就绪标志
    _read_param_ready = false;

    // 发送读参数命令（类型17）
    status_t ret = read_param(index);
    if (PYRO_OK != ret)
        return ret;

    // 等待回应（类型18帧），超时机制
    const TickType_t start_tick = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_tick < pdMS_TO_TICKS(timeout_ms))
    {
        if (_read_param_ready && _read_param_index == index)
        {
            out_value = _read_param_value;
            _read_param_ready = false;  // 消费掉，避免重复读取
            return PYRO_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return PYRO_TIMEOUT;
}

status_t cybergear_motor_drv_t::read_bus_voltage(uint32_t timeout_ms)
{
    float voltage = 0.0f;
    status_t ret = read_param_sync(param_vbus, voltage, timeout_ms);
    if (PYRO_OK == ret)
    {
        _bus_voltage = voltage;
    }
    return ret;
}

bool cybergear_motor_drv_t::get_last_read_param(param_index_t &out_index,
                                                   float &out_value)
{
    if (!_read_param_ready)
        return false;

    out_index = _read_param_index;
    out_value = _read_param_value;
    _read_param_ready = false;  // 消费掉
    return true;
}

} // namespace pyro
