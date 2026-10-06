/**
 * @file servo.h
 * @brief 舵机 PWM 驱动接口与物理参数
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"


//图传舵机物理参数
#define SERVO_MIN_PWM       500.0f   // 0度对应的PWM脉宽(us)
#define SERVO_MAX_PWM       2500.0f  // 最大角度对应的PWM脉宽(us)

#define SERVO_MAX_ANGLE     180.0f   //舵机的最大物理角度

#define PITCH_HORIZON_DEGREE   121.1f 


//初始角度
#define VTM_SERVO_CENTER 6.0f

#define VTM_SERVO_PWM_MIN  500
#define VTM_SERVO_PWM_MAX  1800


//传动比例
#define VTM_PITCH_RATIO 1.58f //1.58

//方向反转
#define VTM_DIR_INVERT 1        


//lens


//倍镜舵机专用配置
#define MAG_ANGLE_OFF 42.0f  //收起时的角度
#define MAG_ANGLE_ON  46.0f  //展开时的角度

#define MAG_OFF 0  //关倍镜
#define MAG_ON  1  //开倍镜


/** @brief 初始化舵机 */
void Bsp_Servo_Init(void);

/** @brief 设置舵机角度 */
void Servo_Set_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle);

/** @brief 图传反向增稳控制 */
void VTM_Servo_Control_Loop(float gun_pitch_angle);

/** @brief 倍镜舵机控制 */
void Magnifier_Servo_Control(uint8_t state);

#endif
