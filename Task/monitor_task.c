#include "monitor_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"


extern Motor_Controller gimbal_yaw_motor;
extern Motor_Controller gimbal_pitch_motor;

extern Motor_Controller dial_motor;

extern Motor_Controller friction_motor_inner_left;
extern Motor_Controller friction_motor_inner_right;
extern Motor_Controller friction_motor_inner_up;
extern Motor_Controller friction_motor_outer_left;
extern Motor_Controller friction_motor_outer_right;
extern Motor_Controller friction_motor_outer_up;



/**
  * @brief  检测电机是否在线
  * @param  motor: 电机结构体指针
  * @param  current_tick: 当前系统时间戳 (ms)
  * @param  timeout_ms: 超时阈值 (ms)
  */
static inline void Check_Motor_Online(Motor_Controller* motor, uint32_t current_tick, uint32_t timeout_ms)
{
    if(current_tick - motor->last_update_tick > timeout_ms) 
    {
        motor->is_online = false;
    } 
    else 
    {
        motor->is_online = true;
    }
}



//离线检测任务
void Monitor_Task(void const * argument)
{
    osDelay(1000); 
    
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		const TickType_t TimeIncrement = pdMS_TO_TICKS(10);

    for(;;)
    {
        uint32_t now = osKernelGetTickCount();

        //云台
        Check_Motor_Online(&gimbal_yaw_motor, now, 200);
        Check_Motor_Online(&gimbal_pitch_motor, now, 200);
        
        //拨盘
        Check_Motor_Online(&dial_motor, now, 200);
        
        //一级摩擦轮
        Check_Motor_Online(&friction_motor_inner_left, now, 200);
        Check_Motor_Online(&friction_motor_inner_right, now, 200);
        Check_Motor_Online(&friction_motor_inner_up, now, 200);
        
        //二级摩擦轮
        Check_Motor_Online(&friction_motor_outer_left, now, 200);
        Check_Motor_Online(&friction_motor_outer_right, now, 200);
        Check_Motor_Online(&friction_motor_outer_up, now, 200);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
