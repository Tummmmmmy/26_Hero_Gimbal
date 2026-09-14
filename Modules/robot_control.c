/**
 * @file robot_control.c
 * @brief 机器人总调度模块(Robot_Control_Loop)
 *
 * 1ms 控制循环的编排层:
 *   更新输入(遥控/IMU) -> 安全检查 -> 模式更新 -> 复位处理
 *   -> 云台/底盘/射击控制 -> 统一 CAN 输出
 * 具体业务逻辑在 gimbal_control / chassis_control / shooter_control 模块中,
 * 本文件只负责调度、模式相关状态与整机级流程。
 */

#include "robot_control.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdbool.h>
#include "remote_control.h"
#include "INS_task.h"
#include "bsp_DR16.h"
#include "Control_Vision.h"
#include "Servo_control.h"
#include "tim.h"
#include "fdcan.h"
#include "can_transmit.h"
#include "robot_mode.h"
#include "gimbal_control.h"
#include "chassis_control.h"
#include "shooter_control.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))

//==================== 整机状态(仅本模块持有) ====================

// 系统LED状态(device_task 通过 Robot_Get_Led_State 读取)
static volatile LED_State_e s_led_state = LED_STATE_BOOTING;

// 复位触发计时(遥控拨轮长按)
static uint32_t s_reset_trigger_timer = 0;

// 射击摩擦轮档位(F/G 键切换) 0:关, 1:12m/s, 2:16m/s
static uint8_t s_fric_level = 0;

//==================== 瞄准/视觉中间状态 ====================

// 是否处于瞄准中(自瞄或手动自瞄)
static uint8_t is_aiming_now = 0;

// 视觉目标角度中间量
static float vision_yaw_deg = 0.0f;
static float vision_pitch_deg = 0.0f;
static float vision_target_yaw_angle = 0.0f;
static float vision_target_pitch_angle = 0.0f;

// 基地打击
static float vision_base_yaw_deg = 0.0f;
static float vision_base_pitch_deg = 0.0f;

static float vision_target_base_pitch_angle = 0.0f;

//==================== 控制参数 ====================

static const float RAD_TO_DEG = 180.0f / PI;
static const float DEG_TO_ENCODER = 8192.0f / 360.0f;

// 云台遥控控制参数
static float ky_follow = 0.2f;
static float kpit_follow = 0.03f;

// 射击(编码器闭环)
static float ky_shoot = 0.01f;
static float kpit_shoot = 0.0015f;

// 键鼠(战斗模式)
static float ky_mouse = 5.0f;
static float kpit_mouse = 3.3f;

// 吊射
static float lob_yaw_step = 0.075f;
static float lob_pitch_step = 0.009f;

// 基地打击步进参数
static float kyaw = 0.002f;
static float yaw_step_max = 0.015f;
static float kpitch = 0.015f;
static float pitch_step_max = 0.1f;

// VTM 上坎低头舵机角度
static const float VTM_SERVO_CLIMB = 100.0f;

// 遥控数据指针(每周期刷新)
static const RC_Data_t* rc;

//==================== 对外接口 ====================

// 初始化所有控制模块(All_Init 任务中调用)
void Robot_Param_Init(void)
{
    // 云台
    Gimbal_Control_Init();

    // 射击
    Shooter_Control_Init();

    // 底盘
    Chassis_Control_Init();
}

// 获取系统LED状态(device_task 读取)
LED_State_e Robot_Get_Led_State(void)
{
    return s_led_state;
}

void Robot_Control_Loop(const IMU_Data_t* imu_data)
{
    Robot_Mode_e robot_mode = ROBOT_MODE_OFF;
    static Robot_Mode_e last_robot_mode = ROBOT_MODE_OFF;
	

  	uint8_t is_base_hitting_now = 0;
	
	  uint8_t is_base_locked = 0;//已锁定基地
	
	  //更新获取遥控输入数据
	  RC_Update();
    rc = RC_Get_Data();
	
	
	  //IMU数据就绪
	  if(imu_ready_flag == 0)
    {
        robot_mode = ROBOT_MODE_OFF; 
        last_robot_mode = ROBOT_MODE_OFF; 
			
		  	LK_Send_Torque(&hfdcan1, 0);
			  DJI_Motor_Send_Command(&hfdcan2, 0x1FF, 0, 0, 0, 0);
        DJI_Motor_Send_Command(&hfdcan2, 0x200, 0, 0, 0, 0);
		  	DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);
        Dial_Send_Speed(&hfdcan1, 0);
        return; 
    }
									

				//模式选择: 由 robot_mode 模块根据遥控来源/上坎状态决定, 同时维护模式LED指示
		Robot_Mode_Update(rc, control_source);
		robot_mode = Robot_Mode_Get();
		s_led_state = Robot_Mode_Get_Led_State();
static bool key_V_last_state = false;
		
		//RESET
    if(robot_mode == ROBOT_MODE_OFF)
    {
        if(rc->rc.wheel < -500)
        {
            s_reset_trigger_timer++;

            //持续足够长时间
            if(s_reset_trigger_timer >= 2000)
            {
						    s_reset_trigger_timer=0;
							  s_led_state = LED_STATE_RESET;
							
								LK_Send_Torque(&hfdcan1, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x1FF, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x200, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);
								Dial_Send_Speed(&hfdcan1, 0);
                osDelay(500);

								Reset_All_IMU_Yaw_Only();
							  Shooter_Reset_Stats();
								Gimbal_Reset_Initialized();
            }
        }
				else if(rc->key.map.V && !key_V_last_state)
				{			
						s_led_state = LED_STATE_RESET;
					
						LK_Send_Torque(&hfdcan1, 0);
						DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);

						Reset_All_IMU_Yaw_Only();
					
						Gimbal_Set_World_Yaw(180.0f);

						Gimbal_Reset_Yaw_SMC();
				}
        else
        {
            s_reset_trigger_timer = 0;
        }
    }
    else s_reset_trigger_timer = 0;
		
		
    if(robot_mode == ROBOT_MODE_BATTLE)
    {
				if(rc->key.map.V && !key_V_last_state)
				{
						s_led_state = LED_STATE_RESET;
					
						LK_Send_Torque(&hfdcan1, 0);
						DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);

						Reset_All_IMU_Yaw_Only();
					
						Gimbal_Set_World_Yaw(180.0f);

						Gimbal_Reset_Yaw_SMC();
				}
			}

		key_V_last_state = rc->key.map.V; //更新状态
		
		
		
		//重新上电后对齐
    Gimbal_Handle_Motor_Reconnect(imu_data);
			

		
    //视觉离线
    if(!Vision_IsOnline()) 
    {
			  Clear_Vision_Target(); 
    }

		

		is_aiming_now = 0;
    if(Vision_IsOnline() && VisionData.is_tracking == 1)
    {
        if(control_source == CONTROL_SOURCE_DR16)
        {
            if(robot_mode == ROBOT_MODE_AIMING || robot_mode == ROBOT_MODE_M_AIMING || robot_mode == ROBOT_MODE_GIMBAL_ONLY)
            {
                is_aiming_now = 1;
            }
        }
        else if(control_source == CONTROL_SOURCE_VTM)
        {
            if(rc->mouse.press_r == 1)
            {
                is_aiming_now = 1;
            }
        }
    }
		
		
		//陀螺仪首次对其
		Gimbal_Sync_On_First_Ready(imu_data);
		
		
		//云台模式切换
		static bool is_base_shooting = false;
		static bool key_B_last_state = false;
		
		if(robot_mode == ROBOT_MODE_BATTLE)
		{
			if(rc->key.map.B && !key_B_last_state)
			{
				is_base_shooting = !is_base_shooting; 
			}
		}
		else is_base_shooting = false;
		key_B_last_state = rc->key.map.B;//更新按键状态
		
		
		//由机器人模式翻译出目标云台模式(重初始化在模块内部完成)
		Gimbal_Mode_e gimbal_mode = GIMBAL_MODE_IMU;
		if(robot_mode == ROBOT_MODE_SHOOT || robot_mode == ROBOT_MODE_SHOOT_LOW || is_base_shooting)
		{
			gimbal_mode = GIMBAL_MODE_ENCODER;
		}
		else
		{
			gimbal_mode = GIMBAL_MODE_IMU;
		}
		
		Gimbal_Set_Mode(gimbal_mode, imu_data);
		

			
		
		
		//重新使能
		if(robot_mode != ROBOT_MODE_OFF && last_robot_mode == ROBOT_MODE_OFF)
		{
			Gimbal_On_Robot_Start(imu_data);
			
			Shooter_On_Robot_Start();
			
			Chassis_On_Robot_Start();
		}
		
		
    //自瞄参数切换(内部做边沿检测)
    Gimbal_Set_Aiming(is_aiming_now);
		
		
		
		
		static bool Key_E_last_state = false;

		static bool Key_Q_last_state = false;
    static bool vtm_look_down = false;

		//E键刚按下瞬间
		bool e_key_just_pressed = (rc->key.map.E && !Key_E_last_state);
		//Q键刚松开瞬间
		bool q_key_just_released = (!rc->key.map.Q && Key_Q_last_state);

		if(q_key_just_released)
		{
				vtm_look_down = false;//收回图传
		}

		//上坎状态机与上坎/LED指令(模块内部维护状态)
		Chassis_Update_Climb(rc, robot_mode, e_key_just_pressed);

		//上坎图传低头(VTM)
    if(robot_mode == ROBOT_MODE_BATTLE)
    {
				if(!rc->key.map.Q && e_key_just_pressed)
				{
						vtm_look_down = !vtm_look_down;
				}

				if(vtm_look_down)
				{
						Servo_Set_Angle(&htim2, TIM_CHANNEL_3, VTM_SERVO_CLIMB);  // 低头看底盘
				}
				else
				{
						Servo_Set_Angle(&htim2, TIM_CHANNEL_3, VTM_SERVO_CENTER); // 抬起看前方
				}

				Key_E_last_state = rc->key.map.E;
				Key_Q_last_state = rc->key.map.Q;
    }

		//云台Yaw编码器偏差(供底盘跟随/坐标变换)
		float yaw_encoder_diff = Gimbal_Get_Yaw_Encoder_Diff();


switch(robot_mode)
{
        case ROBOT_MODE_OFF://失能
        {
						if(last_robot_mode != ROBOT_MODE_OFF)
						{	
								Gimbal_On_Robot_Off();
								
								Shooter_On_Robot_Off();
								
								Chassis_On_Robot_Off();
							
                LK_Send_Torque(&hfdcan1, 0);
								DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x1FF, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x200, 0, 0, 0, 0);
								
								Dial_Send_Speed(&hfdcan1, 0);
							

								s_fric_level = 0;
						}
					
            if(Gimbal_Is_Initialized() && last_robot_mode == ROBOT_MODE_OFF)
            {
                Gimbal_Sync_World_Target_To_IMU(imu_data);
            }
						
						Chassis_Set_Battle_Mode(BATTLE_OFF);
            break;
        }
		 case ROBOT_MODE_BATTLE://键鼠模式
        {
            static bool key_W_last_state = false;
            static bool key_S_last_state = false;

            //底盘目标速度/斜坡滤波/坐标变换/输出快照
            Chassis_Update_Battle(rc, yaw_encoder_diff, is_aiming_now, is_base_shooting);

					  //吊射模式云台按键
            if(is_base_shooting)
            {
                if(rc->key.map.A) Gimbal_Add_Encoder_Target(-lob_yaw_step, 0.0f);
                if(rc->key.map.D) Gimbal_Add_Encoder_Target(lob_yaw_step, 0.0f);

                if(rc->key.map.W) Gimbal_Add_Encoder_Target(0.0f, -lob_pitch_step);
                if(rc->key.map.S) Gimbal_Add_Encoder_Target(0.0f, lob_pitch_step);

                //Pitch限位
                Gimbal_Clamp_Encoder_Pitch(1750.0f, 2900.0f);
            }
            else
            {
								//云台控制
								//雷达
						    if(rc->key.map.R && (VisionData.target_yaw_base_hit != 0 || VisionData.target_pitch_base_hit != 0))
				 			  {
										is_base_hitting_now = 1;

										vision_base_yaw_deg = VisionData.target_yaw_base_hit * RAD_TO_DEG;
										vision_base_pitch_deg = VisionData.target_pitch_base_hit * RAD_TO_DEG;

										if(fabsf(vision_base_yaw_deg) < 180 && fabsf(vision_base_pitch_deg) < 90)
										{
												float yaw_step = vision_base_yaw_deg * kyaw;
												if(yaw_step > yaw_step_max)  yaw_step = yaw_step_max;
												if(yaw_step < -yaw_step_max) yaw_step = -yaw_step_max;

												Gimbal_Add_World_Target(yaw_step, 0.0f);

												vision_target_base_pitch_angle = 180.0f - vision_base_pitch_deg;

												float pitch_diff = vision_target_base_pitch_angle - Gimbal_Get_World_Pitch();

												float pitch_step = pitch_diff * kpitch;
												if(pitch_step > pitch_step_max)  pitch_step = pitch_step_max;
												if(pitch_step < -pitch_step_max) pitch_step = -pitch_step_max;

												Gimbal_Add_World_Target(0.0f, pitch_step);

												float yaw_error = fabsf(vision_base_yaw_deg);
												float actual_pitch_err = fabsf(vision_target_base_pitch_angle - imu_data->pitch_angle);

												if(yaw_error <= 0.1f && actual_pitch_err <= 0.1f)
												{
													   is_base_locked = 1;
														 s_led_state = LED_STATE_BASE_HIT_READY;
												}
												else
												{
														 s_led_state = LED_STATE_ENABLED;
												}
										}
							  }
								else if(Vision_IsOnline() &&VisionData.is_tracking == 1 && rc->mouse.press_r )//右键自瞄   //&&
								{
											vision_yaw_deg = VisionData.target_yaw_rad * RAD_TO_DEG;
											vision_pitch_deg = VisionData.target_pitch_rad * RAD_TO_DEG;

											vision_target_yaw_angle = 180.0f + vision_yaw_deg;
											vision_target_pitch_angle = 180.0f + vision_pitch_deg;

											//最短路径
											while(vision_target_yaw_angle - imu_data->yaw_angle > 180.0f)  vision_target_yaw_angle -= 360.0f;
											while(vision_target_yaw_angle - imu_data->yaw_angle < -180.0f) vision_target_yaw_angle += 360.0f;

											if(fabs(vision_target_yaw_angle - Gimbal_Get_World_Yaw()) < 90.0f && fabs(vision_target_pitch_angle - Gimbal_Get_World_Pitch()) < 90.0f)
											{
													Gimbal_Set_World_Target(vision_target_yaw_angle, vision_target_pitch_angle);
											}
								}
								else
								{
										float yaw_increment = -(float)rc->mouse.x * RC_TO_ONE * ky_mouse;
										float pitch_increment = -(float)rc->mouse.y * RC_TO_ONE * kpit_mouse;

										Gimbal_Add_World_Target(yaw_increment, pitch_increment);

										VisionData.target_yaw_rad = 0.0f;
										VisionData.target_pitch_rad = 0.0f;
										vision_yaw_deg = 0.0f;
										vision_pitch_deg = 0.0f;
										vision_target_yaw_angle = Gimbal_Get_World_Yaw();
										vision_target_pitch_angle = Gimbal_Get_World_Pitch();
								}
					}

						//Pitch限位
						if(Chassis_Get_Climb_Mode() == CLIMB_MODE_ACTIVE && Chassis_Get_Climb_Cmd() == 1)
						{
								Gimbal_Clamp_World_Pitch(135.5f, 166.0f);
						}
						else
						{
								Gimbal_Clamp_World_Pitch(135.5f, 194.1f);
						}

						//重连视觉
            static bool key_X_last_state = false;
            if(rc->key.map.X && !key_X_last_state)
			  		{
						     Vision_Reconnect_Task_Async();
            }
            key_X_last_state = rc->key.map.X; //更新状态

						//上坎图传
            static bool key_E_last_state = false;
            if(rc->key.map.E && !key_E_last_state)
			  		{
						  	Servo_Set_Angle(&htim2, TIM_CHANNEL_3, VTM_SERVO_CLIMB);

							  Servo_Set_Angle(&htim2, TIM_CHANNEL_3, VTM_SERVO_CENTER);
            }
            key_E_last_state = rc->key.map.E; //更新状态

						//调整摩擦轮转速
						float offset_step = 10;//每次点按补偿10
						if(rc->key.map.G && rc->key.map.W && !key_W_last_state) Shooter_Tune_High_Speed((int32_t)offset_step);
						if(rc->key.map.G && rc->key.map.S && !key_S_last_state) Shooter_Tune_High_Speed(-(int32_t)offset_step);

						if(rc->key.map.F && rc->key.map.W && !key_W_last_state) Shooter_Tune_Low_Speed((int32_t)offset_step);
						if(rc->key.map.F && rc->key.map.S && !key_S_last_state) Shooter_Tune_Low_Speed(-(int32_t)offset_step);

            key_W_last_state = rc->key.map.W;
            key_S_last_state = rc->key.map.S;

            break;
        }

				case ROBOT_MODE_GIMBAL_ONLY:
        {
					   if(ABS(rc->rc.wheel) > 500 && (VisionData.target_yaw_base_hit != 0 || VisionData.target_pitch_base_hit != 0))
						 {
							    is_base_hitting_now = 1;

								  vision_base_yaw_deg = VisionData.target_yaw_base_hit * RAD_TO_DEG;
						 		vision_base_pitch_deg = VisionData.target_pitch_base_hit * RAD_TO_DEG;

							    if(fabsf(vision_base_yaw_deg) < 180 && fabsf(vision_base_pitch_deg) < 90)
								{
										float yaw_step = vision_base_yaw_deg * kyaw;
										if(yaw_step > yaw_step_max)  yaw_step = yaw_step_max;
										if(yaw_step < -yaw_step_max) yaw_step = -yaw_step_max;

										Gimbal_Add_World_Target(yaw_step, 0.0f);

										vision_target_base_pitch_angle = 180.0f - vision_base_pitch_deg;

										float pitch_diff = vision_target_base_pitch_angle - Gimbal_Get_World_Pitch();

										float pitch_step = pitch_diff * kpitch;
										if(pitch_step > pitch_step_max)  pitch_step = pitch_step_max;
										if(pitch_step < -pitch_step_max) pitch_step = -pitch_step_max;

										Gimbal_Add_World_Target(0.0f, pitch_step);

										float yaw_error = fabsf(vision_base_yaw_deg);
										float actual_pitch_err = fabsf(vision_target_base_pitch_angle - imu_data->pitch_angle);

										static uint16_t lock_confirm_timer = 0;

										if(yaw_error <= 0.1f && actual_pitch_err <= 0.1f)
										{
												lock_confirm_timer++;
												if(lock_confirm_timer > 200)
												{
														s_led_state = LED_STATE_BASE_HIT_READY;
														lock_confirm_timer = 200;
												}
										}
										else if (yaw_error > 0.1f || actual_pitch_err > 0.1f)
										{
												lock_confirm_timer = 0;
												s_led_state = LED_STATE_ENABLED;
										}
								}
						 }
						 else if(Vision_IsOnline() && VisionData.is_tracking == 1)
              {
								  vision_yaw_deg = VisionData.target_yaw_rad * RAD_TO_DEG;
									vision_pitch_deg = VisionData.target_pitch_rad * RAD_TO_DEG;

									vision_target_yaw_angle = 180.0f + vision_yaw_deg;
									vision_target_pitch_angle = 180.0f + vision_pitch_deg;

									//最短路径
									while(vision_target_yaw_angle - imu_data->yaw_angle > 180.0f)  vision_target_yaw_angle -= 360.0f;
									while(vision_target_yaw_angle - imu_data->yaw_angle < -180.0f) vision_target_yaw_angle += 360.0f;

									if(fabs(vision_target_yaw_angle - Gimbal_Get_World_Yaw()) < 60.0f && fabs(vision_target_pitch_angle - Gimbal_Get_World_Pitch()) < 60.0f)
									{
											Gimbal_Set_World_Target(vision_target_yaw_angle, vision_target_pitch_angle);
									}
              }
						else
						{
									//更新目标角度
									float yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
									float pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;

									Gimbal_Add_World_Target(yaw_angle_increment, pitch_angle_increment);

									VisionData.target_yaw_rad = 0.0f;
									VisionData.target_pitch_rad = 0.0f;
									vision_yaw_deg = 0.0f;
									vision_pitch_deg = 0.0f;
									vision_target_yaw_angle = Gimbal_Get_World_Yaw();
									vision_target_pitch_angle = Gimbal_Get_World_Pitch();
						}

						//Pitch限位
						Gimbal_Clamp_World_Pitch(135.5f, 194.1f);

						//底盘(坐标变换, 不改自旋, 不请求超级电容)
						Chassis_Update_Move(rc, yaw_encoder_diff, CHASSIS_MOVE_GIMBAL_ONLY);

						Shooter_Control_Disable();

          break;
        }

				case ROBOT_MODE_FOLLOW:
        {
						//更新目标角度
						float yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
						float pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;

						Gimbal_Add_World_Target(yaw_angle_increment, pitch_angle_increment);

						//Pitch限位
						Gimbal_Clamp_World_Pitch(135.5f, 194.5f);

						//底盘(坐标变换 + 拨轮小陀螺/底盘跟随PID)
						Chassis_Update_Move(rc, yaw_encoder_diff, CHASSIS_MOVE_FOLLOW);

						if(control_source != CONTROL_SOURCE_VTM)
						{
						Shooter_Control_Disable();
						}

						Chassis_Set_Battle_Mode(BATTLE_FOLLOW);
          break;
        }

        case ROBOT_MODE_SHOOT:
				case ROBOT_MODE_SHOOT_LOW:
        {
						//更新目标角度
						float yaw_angle_increment = (float)rc->rc.right_x * RC_TO_ONE * ky_shoot ;
						float pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_shoot;

						float yaw_encoder_increment = yaw_angle_increment * DEG_TO_ENCODER;
						float pitch_encoder_increment = pitch_angle_increment * DEG_TO_ENCODER;

						Gimbal_Add_Encoder_Target(yaw_encoder_increment, pitch_encoder_increment);

						//Pitch限位
						Gimbal_Clamp_Encoder_Pitch(1750.0f, 2900.0f);

						//底盘停机
						Chassis_Stop_Movement();

            break;
        }

				case ROBOT_MODE_AIMING:
				{
						 if(Vision_IsOnline() && VisionData.is_tracking == 1)
              {
								  vision_yaw_deg = VisionData.target_yaw_rad * RAD_TO_DEG;
									vision_pitch_deg = VisionData.target_pitch_rad * RAD_TO_DEG;

									vision_target_yaw_angle = 180.0f + vision_yaw_deg;
									vision_target_pitch_angle = 180.0f + vision_pitch_deg;

									//最短路径
									while(vision_target_yaw_angle - imu_data->yaw_angle > 180.0f)  vision_target_yaw_angle -= 360.0f;
									while(vision_target_yaw_angle - imu_data->yaw_angle < -180.0f) vision_target_yaw_angle += 360.0f;

									if(fabs(vision_target_yaw_angle - Gimbal_Get_World_Yaw()) < 60.0f && fabs(vision_target_pitch_angle - Gimbal_Get_World_Pitch()) < 60.0f)
									{
											Gimbal_Set_World_Target(vision_target_yaw_angle, vision_target_pitch_angle);
									}
              }
						else
						{
									//更新目标角度
									float yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
									float pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;

									Gimbal_Add_World_Target(yaw_angle_increment, pitch_angle_increment);

									VisionData.target_yaw_rad = 0.0f;
									VisionData.target_pitch_rad = 0.0f;
									vision_yaw_deg = 0.0f;
									vision_pitch_deg = 0.0f;
									vision_target_yaw_angle = Gimbal_Get_World_Yaw();
									vision_target_pitch_angle = Gimbal_Get_World_Pitch();
						}

						//Pitch限位
						Gimbal_Clamp_World_Pitch(135.5f, 194.1f);

						//底盘(坐标变换 + 拨轮小陀螺1.5倍速)
						Chassis_Update_Move(rc, yaw_encoder_diff, CHASSIS_MOVE_AIMING);
            break;
				}

				case ROBOT_MODE_M_AIMING:
				{
						 if(Vision_IsOnline() && VisionData.is_tracking == 1)
              {
								  vision_yaw_deg = VisionData.target_yaw_rad * RAD_TO_DEG;
									vision_pitch_deg = VisionData.target_pitch_rad * RAD_TO_DEG;

									vision_target_yaw_angle = 180.0f + vision_yaw_deg;
									vision_target_pitch_angle = 180.0f + vision_pitch_deg;

									//最短路径
									while(vision_target_yaw_angle - imu_data->yaw_angle > 180.0f)  vision_target_yaw_angle -= 360.0f;
									while(vision_target_yaw_angle - imu_data->yaw_angle < -180.0f) vision_target_yaw_angle += 360.0f;

									if(fabs(vision_target_yaw_angle - Gimbal_Get_World_Yaw()) < 60.0f && fabs(vision_target_pitch_angle - Gimbal_Get_World_Pitch()) < 60.0f)
									{
											Gimbal_Set_World_Target(vision_target_yaw_angle, vision_target_pitch_angle);
									}
              }
						else
						{
									//更新目标角度
									float yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
									float pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;

									Gimbal_Add_World_Target(yaw_angle_increment, pitch_angle_increment);

									VisionData.target_yaw_rad = 0.0f;
									VisionData.target_pitch_rad = 0.0f;
									vision_yaw_deg = 0.0f;
									vision_pitch_deg = 0.0f;
									vision_target_yaw_angle = Gimbal_Get_World_Yaw();
									vision_target_pitch_angle = Gimbal_Get_World_Pitch();
						}

						//Pitch限位
						Gimbal_Clamp_World_Pitch(135.5f, 194.1f);

						//底盘(坐标变换, 不改自旋)
						Chassis_Update_Move(rc, yaw_encoder_diff, CHASSIS_MOVE_M_AIMING);
            break;
				}

        case ROBOT_MODE_CHASSIS_ONLY:
        {
            //裸底盘(不做坐标变换, 左杆前推超阈值请求超级电容)
            Chassis_Set_Direct(rc, CHASSIS_DIRECT_AUTO_SUPERCAP);

						Shooter_Control_Disable();

				    Gimbal_Sync_World_Target_To_IMU(imu_data);
            break;
        }

				case ROBOT_MODE_CLIMB:
        {
					 //更新目标角度
					 float pitch_angle_increment = -(float)rc->rc.left_x * RC_TO_ONE * kpit_follow;
					 Gimbal_Add_World_Target(0.0f, pitch_angle_increment);

					if(Chassis_Get_Climb_Mode() == CLIMB_MODE_ACTIVE && Chassis_Get_Climb_Cmd() == 1)
					{
							//Pitch限位
							Gimbal_Clamp_World_Pitch(135.5f, 166.0f);
					}
					else
					{
							//Pitch限位
							Gimbal_Clamp_World_Pitch(135.5f, 194.1f);
					}

            //裸底盘(不做坐标变换, 不请求超级电容)
            Chassis_Set_Direct(rc, CHASSIS_DIRECT_NO_SUPERCAP);

					if(last_robot_mode  != ROBOT_MODE_CLIMB)
					{
							Gimbal_Set_World_Yaw(imu_data->yaw_angle);
					}

            Shooter_Control_Disable();
            break;
        }
 }
		 if(Chassis_Get_Climb_Cmd() != 0)
		 {
		 	Chassis_Zero_Turn();
		 	Gimbal_Set_World_Yaw(imu_data->yaw_angle);
		 }
		 
		 
		//雷达辅吊云台参数切换(内部做边沿检测)
		Gimbal_Set_Base_Hitting(is_base_hitting_now);
		 
 
    //云台闭环计算(目标角已由上方各模式写入模块)
    Gimbal_Set_Active(robot_mode != ROBOT_MODE_CHASSIS_ONLY && robot_mode != ROBOT_MODE_OFF);
    Gimbal_Control_Update(imu_data);
		
        Shooter_Set_System_State(robot_mode == ROBOT_MODE_OFF,
                                 robot_mode == ROBOT_MODE_AIMING,
                                 (robot_mode == ROBOT_MODE_BATTLE && is_aiming_now == 1),
                                 ABS(rc->rc.wheel) < 100);

        static bool f_key_last = false;
        if (rc->key.map.F && !f_key_last) s_fric_level = 1;
        f_key_last = rc->key.map.F;

        static bool g_key_last = false;
        if (rc->key.map.G && !g_key_last) s_fric_level = 2;
        g_key_last = rc->key.map.G;

        Shooter_Cmd_t shoot_cmd = {0};
        shoot_cmd.use_referee_heat_limit = (control_source == CONTROL_SOURCE_VTM);

        if (control_source == CONTROL_SOURCE_VTM)
        {
            if (s_fric_level == 1) shoot_cmd.fric_mode = SHOOTER_MODE_LOW_SPEED;
            else if (s_fric_level == 2) shoot_cmd.fric_mode = SHOOTER_MODE_HIGH_SPEED;
        }
        else
        {
            if (robot_mode == ROBOT_MODE_SHOOT || (robot_mode == ROBOT_MODE_BATTLE && gimbal_mode == GIMBAL_MODE_ENCODER))
                shoot_cmd.fric_mode = SHOOTER_MODE_HIGH_SPEED;
            else if (robot_mode == ROBOT_MODE_SHOOT_LOW || robot_mode == ROBOT_MODE_AIMING || robot_mode == ROBOT_MODE_M_AIMING || (robot_mode == ROBOT_MODE_BATTLE && gimbal_mode == GIMBAL_MODE_IMU))
                shoot_cmd.fric_mode = SHOOTER_MODE_LOW_SPEED;
        }

        uint8_t shooter_armed =
            (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_MID) ||
            (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_UP) ||
            (rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_UP) ||
            (robot_mode == ROBOT_MODE_BATTLE && (s_fric_level == 1 || s_fric_level == 2)) ||
            (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_DOWN);
        if (!shooter_armed) shoot_cmd.fric_mode = SHOOTER_MODE_OFF;

        static bool mouse_l_last_state = false;
        bool mouse_l_just_pressed = (rc->mouse.press_l && !mouse_l_last_state);
        mouse_l_last_state = rc->mouse.press_l;

        static bool mouse_m_last_state = false;
        bool mouse_m_just_pressed = (rc->mouse.press_middle && !mouse_m_last_state);
        mouse_m_last_state = rc->mouse.press_middle;

        if (rc->rc.wheel > 250 && robot_mode != ROBOT_MODE_AIMING)
        {
            shoot_cmd.fire_command = 1;
        }
        if (robot_mode == ROBOT_MODE_AIMING && Vision_IsOnline() && VisionData.is_tracking == 1 && VisionData.cmd_mode == 2)
        {
            shoot_cmd.fire_command = 1;
        }
        if (mouse_m_just_pressed) shoot_cmd.fire_command = 1;
        if (is_aiming_now)
        {
            if (rc->mouse.press_l && VisionData.cmd_mode == 2) shoot_cmd.fire_command = 1;
        }
        else if (mouse_l_just_pressed)
        {
            shoot_cmd.fire_command = 1;
        }

        Shooter_Control_Update(&shoot_cmd);
        const Shooter_Output_t *shooter_out = Shooter_Get_Output();
        if (shooter_out->dial_mode_cmd)
        {
            Dial_Send_Mode_Command(&hfdcan1, shooter_out->dial_mode_cmd);
        }

	    	const Gimbal_Output_t *gimbal_out = Gimbal_Get_Output();
	    	LK_Send_Torque(&hfdcan1, gimbal_out->yaw_current);
		    DJI_Motor_Send_Command(&hfdcan3, 0x1FF, gimbal_out->pitch_current, 0, 0, 0);


        DJI_Motor_Send_Command(&hfdcan2, 0x200, shooter_out->friction_current_outer_left, shooter_out->friction_current_outer_right, shooter_out->friction_current_outer_up, 0);
        DJI_Motor_Send_Command(&hfdcan2, 0x1FF, shooter_out->friction_current_inner_left, shooter_out->friction_current_inner_right, shooter_out->friction_current_inner_up, 0);
		
        uint8_t friction_report = 0;
        if (shooter_out->friction_ready)
        {
            if (control_source == CONTROL_SOURCE_VTM)
            {
                friction_report = (shoot_cmd.fric_mode == SHOOTER_MODE_LOW_SPEED) ? 1 : 2;
            }
            else
            {
                friction_report = (shoot_cmd.fric_mode == SHOOTER_MODE_HIGH_SPEED) ? 1 : 2;
            }
        }
        const Chassis_Output_t *chassis_out = Chassis_Get_Output();
        Send_Chassis_Command(&hfdcan1, chassis_out->vx, chassis_out->vy, chassis_out->wz, chassis_out->mode, chassis_out->supercap, chassis_out->climb, chassis_out->led, chassis_out->shift, friction_report, is_base_locked, chassis_out->battle_mode);

		    Dial_Send_Speed(&hfdcan1, shooter_out->dial_target_speed);
		
				static uint32_t ui_send_cnt = 0;
				if (++ui_send_cnt >= 50) 
				{
						ui_send_cnt = 0;
						
						float yaw_deg = gimbal_out->yaw_degree; 
						float pit_deg = gimbal_out->pitch_degree;

						Send_Chassis_UI_Data(&hfdcan1, yaw_deg, pit_deg, shooter_out->friction_target_speed);
				}
        last_robot_mode = robot_mode;
}



