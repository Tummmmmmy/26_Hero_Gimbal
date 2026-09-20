/**
 * @file robot_types.h
 * @brief 机器人整机级公共类型
 *
 * 由 robot_control / robot_mode 等整机层模块共用的类型定义。
 */
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

//机器人LED状态枚举
typedef enum {
    LED_STATE_BOOTING,        //启动中 (紫色)
    LED_STATE_DISABLED,       //失能 (红色)
    LED_STATE_ENABLED,        //使能(蓝色)
    LED_STATE_CLIMB_FRONT,    //上坡-前伸(黄色)
    LED_STATE_CLIMB_BACK,     //上坡-后伸(青色)
	  LED_STATE_BASE_HIT_READY,
	  LED_STATE_RESET
} LED_State_e;

#endif
