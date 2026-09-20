/**
 * @file print_task.c
 * @brief 10ms 调试打印任务(Print_Entry_Task)
 *
 * 发射器弹速统计/摩擦轮转速等吊射 VOFA 上位机数据
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
		// 通过模块对外接口取本帧快照, 不再直接访问模块内部变量
		const Shooter_Debug_Data_t *shooter_dbg = Shooter_Get_Debug_Data();
		const Gimbal_Debug_Data_t  *gimbal_dbg  = Gimbal_Get_Debug_Data();

		// ===== 方案1: 云台自瞄 SMC (4路) =====
		//VOFA_Print_Wave(4,
		//gimbal_dbg->yaw_ref_pos,      //Yaw 目标角度
		//gimbal_dbg->yaw_fdb_pos,      //Yaw 实际角度
		//gimbal_dbg->pitch_ref_pos,    //Pitch 目标角度
		//gimbal_dbg->pitch_fdb_pos);   //Pitch 实际角度

		// ===== 方案2: 吊射弹速 + 6路摩擦轮转速 (10路, 当前启用) =====
		VOFA_Print_Wave(10,
		shooter_dbg->referee_bullet_speed,   //弹速
		shooter_dbg->bullet_speed_avg,       //平均弹速
		shooter_dbg->bullet_speed_variance,  //方差
		shooter_dbg->bullet_speed_range,     //极差
		shooter_dbg->friction_outer_left_speed,
		-shooter_dbg->friction_outer_right_speed,
		shooter_dbg->friction_outer_up_speed,
		shooter_dbg->friction_inner_left_speed,
		-shooter_dbg->friction_inner_right_speed,
		shooter_dbg->friction_inner_up_speed);

		// ===== 方案3: 云台电机编码器 + 发射数/预测热量 (4路) =====
		//VOFA_Print_Wave(4,
		//gimbal_dbg->yaw_motor_angle,     //Yaw 电机编码器原始值
		//gimbal_dbg->pitch_motor_angle,   //Pitch 电机编码器原始值
		//shooter_dbg->shot_count,         //发射数
		//shooter_dbg->predict_heat);      //预测热量

		// ===== 方案4: 拨盘调试 (8路) =====
		//VOFA_Print_Wave(8,
		//shooter_dbg->dial_total_angle,      //拨盘当前总角度(rad)
		//shooter_dbg->dial_target_rad,       //拨盘目标角度(rad)
		//shooter_dbg->dial_angle_target,     //拨盘角度环目标
		//shooter_dbg->dial_angle_measured,   //拨盘角度环反馈
		//shooter_dbg->dial_speed_target,     //拨盘速度环目标
		//shooter_dbg->dial_speed_measured,   //拨盘速度环反馈
		//shooter_dbg->dial_torque,           //拨盘力矩反馈
		//shooter_dbg->dial_velocity);        //拨盘速度反馈

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
