#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H

#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "pyro_core_def.h"

/**
 * @brief 平移轴电机控制类（双环PID + 多圈位置计数）
 *
 * 适用于大疆M3508等带减速器的旋转电机驱动平移轴。
 * 将电机编码器的多圈角度转换为线位移，支持位置控制和速度控制。
 *
 * 控制架构:
 *   目标位置 → [位置环PID] → 目标速度 → [速度环PID] → 目标力矩 → 电机
 *
 * 移植自原H723主控板，适配F407 C板。
 */
class move_control_t
{
public:
    typedef enum
    {
        POSITION_CONTROL,
        VELOCITY_CONTROL
    }
    move_mode_e;

private:
    pyro::motor_base_t* _motor;
    pyro::pid_t* _pos_pid;
    pyro::pid_t* _rot_pid;

    float _perimeter;       ///< 丝杆/轮子周长(mm)，电机转一圈对应的线位移
    move_mode_e _mode;
    float _target_pos;
    float _target_rot;

    float _feedback_pos;
    float _feedback_rot;
    float _direction;       ///< 方向系数: 1.0或-1.0

    float _pos_offset;      ///< 位置零点偏移(电机原始角度)
    int32_t _cycle_counter; ///< 多圈计数器
    float _last_pos;        ///< 上一次电机位置(用于圈数判断)
    float _gravity_offset;  ///< 重力补偿力矩(垂直轴使用)

public:

    move_control_t(pyro::motor_base_t* motor, pyro::pid_t* pos_pid, pyro::pid_t* rot_pid)
    {
        _motor = motor;
        _pos_pid = pos_pid;
        _rot_pid = rot_pid;
        _cycle_counter = 0;
        _perimeter = pyro::PI;   // 默认周长=π(需根据实际机械结构设置)
        _mode = VELOCITY_CONTROL;
        _direction = 1.0f;
        _gravity_offset = 0.0f;
        _last_pos = 0.0f;
        _pos_offset = 0.0f;
    }

    /**
     * @brief 设置周长(电机转一圈对应的线位移，单位mm)
     */
    void set_perimeter(float perimeter)
    {
        _perimeter = perimeter;
    }

    /**
     * @brief 更新电机反馈，计算多圈位置和线位移
     * 应在每个控制周期调用，在control()之前调用
     */
    void update()
    {
        float cur_pos;

        _motor->update_feedback();
        cur_pos = _motor->get_current_position();

        // 多圈计数: 检测位置跨越±π边界
        if (cur_pos - _last_pos < -pyro::PI)
        {
            _cycle_counter++;
        }
        else if (cur_pos - _last_pos > pyro::PI)
        {
            _cycle_counter--;
        }
        _last_pos = cur_pos;

        // 计算累计角度 → 转换为线位移
        _feedback_pos = cur_pos - _pos_offset + 2 * pyro::PI * _cycle_counter;
        _feedback_pos = _feedback_pos / 2.0f / pyro::PI * _perimeter * _direction;
        _feedback_rot = _motor->get_current_rotate() * _direction;
    }

    /**
     * @brief 设置目标速度(进入速度控制模式，校准时用)
     * @param speed 电机转子转速(rad/s)，与get_current_rotate()单位一致
     */
    void set_target_speed(float speed)
    {
        _mode = VELOCITY_CONTROL;
        _target_pos = _feedback_pos;
        _target_rot = speed;
    }

    /**
     * @brief 设置目标位置(进入位置控制模式)
     */
    void set_target_pos(float pos)
    {
        _mode = POSITION_CONTROL;
        _target_pos = pos;
        _target_rot = _feedback_rot;
    }

    /**
     * @brief 执行PID计算并发送力矩命令
     * 应在update()之后调用
     */
    void control()
    {
        float target_torque = 0.0f;
        float target_rot = 0.0f;
        switch (_mode)
        {
            case POSITION_CONTROL:
                target_rot = _pos_pid->calculate(_target_pos, _feedback_pos);
                target_torque = _rot_pid->calculate(target_rot, _feedback_rot) + _gravity_offset;
                break;
            case VELOCITY_CONTROL:
                target_torque = _rot_pid->calculate(_target_rot, _feedback_rot) + _gravity_offset;
                break;
            default:
                target_torque = 0.0f;
                break;
        }
        _motor->send_torque(target_torque * _direction);
    }

    /**
     * @brief 设置位置零点偏移(校准时调用，将当前电机原始位置设为零点)
     */
    void set_pos_offset(float offset)
    {
        _pos_offset = offset;
        _cycle_counter = 0;
    }

    float get_motor_raw_pos()
    {
        return _motor->get_current_position();
    }

    float get_feedback_pos()
    {
        return _feedback_pos;
    }

    float get_feedback_rot()
    {
        return _feedback_rot;
    }

    void set_direction(float direction)
    {
        _direction = direction < 0 ? -1.0f : 1.0f;
    }

    void set_gravity_offset(float offset)
    {
        _gravity_offset = offset;
    }
};

#endif // MOVE_CONTROL_H
