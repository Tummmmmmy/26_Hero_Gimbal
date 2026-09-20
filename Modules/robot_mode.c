/**
 * @file robot_mode.c
 * @brief 机器人工作模式选择模块
 *
 * 根据遥控来源(DR16/VTM)与底盘上坎状态决定当前工作模式,
 * 并维护模式对应的 LED 指示。只负责“选模式”, 不执行控制与 CAN 发送。
 */

#include "robot_mode.h"
#include "remote_control.h"
#include "bsp_DR16.h"
#include "chassis_control.h"

// 当前工作模式
static Robot_Mode_e mode = ROBOT_MODE_OFF;

// 模式对应的LED指示(无控制源时保持上一次)
static LED_State_e mode_led = LED_STATE_BOOTING;

/**
 * @brief 更新机器人工作模式
 * @param rc 遥控数据
 * @param control_source 当前控制源(VTM/DR16)
 */
void Robot_Mode_Update(const RC_Data_t* rc, ControlSource_e control_source)
{
    //VT03 拨杆选择
    if (control_source == CONTROL_SOURCE_VTM)
    {
        switch (rc->rc.s_left)
        {
            case 1: //S
                mode = ROBOT_MODE_BATTLE;
                mode_led = LED_STATE_ENABLED;
                break;

            case 3: //N
                mode = ROBOT_MODE_FOLLOW;
                mode_led = LED_STATE_ENABLED;
                break;

            case 2: //C
            default:
                mode = ROBOT_MODE_OFF;
                mode_led = LED_STATE_DISABLED;
                break;
        }
    }
    //DR16 拨杆组合
    else if (control_source == CONTROL_SOURCE_DR16)
    {
        //上坎状态优先: 上坎待命/执行中固定为爬坡模式(不改变LED)
        if (Chassis_Get_Climb_Mode() == CLIMB_MODE_STANDBY || Chassis_Get_Climb_Mode() == CLIMB_MODE_ACTIVE)
        {
            mode = ROBOT_MODE_CLIMB;
        }
        else if (rc->rc.s_left == DR16_SW_DOWN && rc->rc.s_right == DR16_SW_DOWN)
        {
            mode = ROBOT_MODE_OFF;
            mode_led = LED_STATE_DISABLED;
        }
        else if (rc->rc.s_left == DR16_SW_DOWN && rc->rc.s_right == DR16_SW_UP)
        {
            mode = ROBOT_MODE_GIMBAL_ONLY;
            mode_led = LED_STATE_ENABLED;
        }
        else if (rc->rc.s_left == DR16_SW_DOWN && rc->rc.s_right == DR16_SW_MID)
        {
            mode = ROBOT_MODE_CHASSIS_ONLY;
            mode_led = LED_STATE_DISABLED;
        }
        else if (rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_MID)
        {
            mode = ROBOT_MODE_FOLLOW;
            mode_led = LED_STATE_ENABLED;
        }
        else if (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_MID)
        {
            mode = ROBOT_MODE_SHOOT;
            mode_led = LED_STATE_ENABLED;
        }
        else if (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_DOWN)
        {
            mode = ROBOT_MODE_SHOOT_LOW;
            mode_led = LED_STATE_ENABLED;
        }
        else if (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_UP)
        {
            mode = ROBOT_MODE_AIMING;
            mode_led = LED_STATE_ENABLED;
        }
        else if (rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_UP)
        {
            mode = ROBOT_MODE_M_AIMING;
            mode_led = LED_STATE_ENABLED;
        }
        else if (rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_DOWN)
        {
            mode = ROBOT_MODE_CLIMB;
            mode_led = LED_STATE_ENABLED;
        }
    }
    //无控制源: 强制失能, LED保持
    else
    {
        mode = ROBOT_MODE_OFF;
    }
}

// 获取当前工作模式
Robot_Mode_e Robot_Mode_Get(void)
{
    return mode;
}

// 获取模式对应的LED指示
LED_State_e Robot_Mode_Get_Led_State(void)
{
    return mode_led;
}
