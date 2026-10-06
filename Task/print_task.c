/**
 * @file print_task.c
 * @brief 调试打印任务
 *
 */

#include "print_task.h"
#include "cmsis_os.h"
#include "bsp_ano_protocol.h"
#include "shooter_control.h"
#include "gimbal_control.h"

void Print_Entry_Task(void const * argument)
{
	osDelay(2000);

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(10);

	for(;;)
	{
		const Shooter_Debug_Data_t *shooter_dbg = Shooter_Get_Debug_Data();
		const Gimbal_Debug_Data_t  *gimbal_dbg  = Gimbal_Get_Debug_Data();

//		//自瞄
//		VOFA_Print_Wave(4,
//		gimbal_dbg->yaw_ref_pos,
//		gimbal_dbg->yaw_fdb_pos,
//		gimbal_dbg->pitch_ref_pos,
//		gimbal_dbg->pitch_fdb_pos);

//		//吊射
//		VOFA_Print_Wave(10,
//		shooter_dbg->referee_bullet_speed,   //弹速
//		shooter_dbg->bullet_speed_avg,       //平均弹速
//		shooter_dbg->bullet_speed_variance,  //方差
//		shooter_dbg->bullet_speed_range,     //极差
//		shooter_dbg->friction_outer_left_speed,
//		-shooter_dbg->friction_outer_right_speed,
//		shooter_dbg->friction_outer_up_speed,
//		shooter_dbg->friction_inner_left_speed,
//		-shooter_dbg->friction_inner_right_speed,
//		shooter_dbg->friction_inner_up_speed);


//		VOFA_Print_Wave(4,
//		gimbal_dbg->yaw_motor_angle,
//		gimbal_dbg->pitch_motor_angle,
//		shooter_dbg->shot_count,         //发射数
//		shooter_dbg->predict_heat);      //预测热量

//    //拨盘
//		VOFA_Print_Wave(8,
//		shooter_dbg->dial_total_angle,
//		shooter_dbg->dial_target_rad,
//		shooter_dbg->dial_angle_target,
//		shooter_dbg->dial_angle_measured,
//		shooter_dbg->dial_speed_target,
//		shooter_dbg->dial_speed_measured,
//		shooter_dbg->dial_torque,
//		shooter_dbg->dial_velocity);

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
