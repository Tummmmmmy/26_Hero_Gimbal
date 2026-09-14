/**
 * @file print_task.c
 * @brief 10ms 调试打印任务(Print_Entry_Task)
 *
 * 输出射击弹速统计/摩擦轮转速等到 VOFA 上位机。
 */

#include "print_task.h"
#include "cmsis_os.h"
#include "bsp_ano_protocol.h"
#include "shooter_control.h"

void Print_Entry_Task(void const * argument)
{
	osDelay(2000); 
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(10);
	
	for(;;)
	{
		
Shooter_Debug_Tick();
const Shooter_Debug_Data_t *shooter_dbg = Shooter_Get_Debug_Data();
		
		
////自瞄
//			            VOFA_Print_Wave(4,
//(float)gimbal_yaw_smc_imu.ref_pos,
//(float)gimbal_yaw_smc_imu.fdb_pos,
//(float)gimbal_pitch_smc_imu.ref_pos,
//(float)gimbal_pitch_smc_imu.fdb_pos);
//		

					            VOFA_Print_Wave(10,
//吊射
(float)shooter_dbg->referee_bullet_speed,  //弹速
(float)shooter_dbg->bullet_speed_avg,     //平均弹速
(float)shooter_dbg->bullet_speed_variance,//方差
(float)shooter_dbg->bullet_speed_range, //极差
(float)shooter_dbg->friction_outer_left_speed,
-(float)shooter_dbg->friction_outer_right_speed,
(float)shooter_dbg->friction_outer_up_speed,
(float)shooter_dbg->friction_inner_left_speed,
-(float)shooter_dbg->friction_inner_right_speed,
(float)shooter_dbg->friction_inner_up_speed);



//(float)gimbal_yaw_motor.feedback.angle,
//(float)gimbal_pitch_motor.feedback.angle,
//(float)total_shot_count,     //发射数
//热量
//(float)shooter_predict_heat);


////拨盘
//			            VOFA_Print_Wave(8,
//(float)dial_motor.current_total_angle_rad,
//(float)shooter_controller.dial_target_rad,
//(float)dial_motor.pid_angle.target,
//(float)dial_motor.pid_angle.measured,
//(float)dial_motor.pid_speed.target,
//(float)dial_motor.pid_speed.measured,
//(float)dial_motor.feedback.torque,
//(float)dial_motor.feedback.velocity);

			vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
