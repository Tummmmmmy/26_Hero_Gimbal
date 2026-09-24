/**
 * @file servo.c
 * @brief 舵机 PWM 驱动
 *
 * 把目标角度换算成 PWM 脉宽, 驱动图传低头舵机与磁铁舵机。
 */

#include "servo.h"
#include "tim.h"


#define VTM_TIM_HANDLE  (&htim2)
#define VTM_TIM_CHANNEL TIM_CHANNEL_3


#define MAG_TIM_HANDLE  (&htim1)
#define MAG_TIM_CHANNEL TIM_CHANNEL_1


//初始化舵机
void Bsp_Servo_Init(void)
{
    HAL_TIM_PWM_Start(VTM_TIM_HANDLE, VTM_TIM_CHANNEL);

    //初始化舵机角度
    Servo_Set_Angle(VTM_TIM_HANDLE, VTM_TIM_CHANNEL, VTM_SERVO_CENTER);
}


void Servo_Set_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle)
{
    //角度安全限幅
    if(angle < 0.0f) angle = 0.0f;
    if(angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    //角度映射到PWM
    float pwm_val = SERVO_MIN_PWM + (angle / SERVO_MAX_ANGLE) * (SERVO_MAX_PWM - SERVO_MIN_PWM);

	  //PWM限幅
    if(pwm_val > VTM_SERVO_PWM_MAX) pwm_val = VTM_SERVO_PWM_MAX;
    if(pwm_val < VTM_SERVO_PWM_MIN) pwm_val = VTM_SERVO_PWM_MIN;

    __HAL_TIM_SetCompare(htim, Channel, (uint32_t)pwm_val);
}


void VTM_Servo_Control_Loop(float gun_pitch_angle)
{
	  float diff = PITCH_HORIZON_DEGREE  - gun_pitch_angle;

    //计算补偿后的目标角度
    float target_angle = VTM_SERVO_CENTER + (diff  * VTM_PITCH_RATIO * VTM_DIR_INVERT);

    //发送给舵机执行
    Servo_Set_Angle(VTM_TIM_HANDLE, VTM_TIM_CHANNEL, target_angle);
}


void Magnifier_Servo_Control(uint8_t state)
{
    if(state == MAG_ON)
    {
        Servo_Set_Angle(MAG_TIM_HANDLE, MAG_TIM_CHANNEL, MAG_ANGLE_ON);
    }
    else
    {
        Servo_Set_Angle(MAG_TIM_HANDLE, MAG_TIM_CHANNEL, MAG_ANGLE_OFF);
    }
}
