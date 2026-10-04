#ifndef MOVE_SWITCH_H
#define MOVE_SWITCH_H

#include "main.h"

/**
 * @brief 平移轴限位开关检测类
 *
 * 适配 RoboMaster 开发板C型(STM32F407) 的GPIO引脚。
 * CubeMX中已配置为输入模式(无上拉)，共4个限位开关。
 *
 * 引脚映射(可根据实际接线修改):
 *   up      -> GPIOI, GPIO_PIN_6
 *   down    -> GPIOI, GPIO_PIN_7
 *   forward -> GPIOC, GPIO_PIN_6
 *   back    -> GPIOE, GPIO_PIN_14
 *
 * 注意: 如果限位开关是常闭型(按下时为低电平)，需要将判断条件
 *       从 GPIO_PIN_SET 改为 GPIO_PIN_RESET。
 */
class move_switch_t
{
public:
    move_switch_t()
    {
        // 构造时不操作GPIO，CubeMX已完成初始化
        _up_switch_hold = false;
        _down_switch_hold = false;
        _forawrd_switch_hold = false;
        _back_switch_hold = false;
    }

    ~move_switch_t() {}

    /**
     * @brief 更新所有限位开关状态，应在控制循环中周期调用
     */
    void update(void)
    {
        // up: GPIOI_PIN_6
        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6))
        {
            _up_switch_hold = true;
        }
        else
        {
            _up_switch_hold = false;
        }

        // down: GPIOI_PIN_7
        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_7))
        {
            _down_switch_hold = true;
        }
        else
        {
            _down_switch_hold = false;
        }

        // forward: GPIOC_PIN_6
        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
        {
            _forawrd_switch_hold = true;
        }
        else
        {
            _forawrd_switch_hold = false;
        }

        // back: GPIOE_PIN_14
        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14))
        {
            _back_switch_hold = true;
        }
        else
        {
            _back_switch_hold = false;
        }
    }

    // 限位开关状态(public方便访问)
    bool _up_switch_hold;
    bool _down_switch_hold;
    bool _forawrd_switch_hold;
    bool _back_switch_hold;
};

#endif // MOVE_SWITCH_H
