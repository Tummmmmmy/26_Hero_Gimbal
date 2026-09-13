/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "arm_math.h"

#include "bsp_buzzer.h"
#include "INS_task.h"
#include "bsp_DR16.h"
#include "bsp_VT03.h"
#include "remote_control.h"
#include "bsp_ano_protocol.h"

#include "Control_Vision.h"
#include "vision_can.h"
#include "smc.h"
#include "Ramp.h"
#include "bsp_dwt.h"
#include "Servo_control.h"
#include "pid_controller.h"
#include "robot_mode.h"
#include "shooter_control.h"
#include "gimbal_control.h"
#include "chassis_control.h"
#include "bsp_can.h"
#include "can_transmit.h"
/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ABS(x) ((x) > 0 ? (x) : -(x))

volatile Robot_Mode_e robot_mode = ROBOT_MODE_OFF;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */











const float RAD_TO_DEG = 180.0f / PI;
const float DEG_TO_ENCODER = 8192.0f / 360.0f;











//视觉传入的目标



//云台控制参数
//跟随
float ky_follow = 0.2f;
float kpit_follow = 0.03f;
//发射
float ky_shoot = 0.01f;
float kpit_shoot = 0.0015f;


//键鼠
float ky_mouse = 5.0f;
float kpit_mouse = 3.3f;

//吊射
float lob_yaw_step = 0.075f; 
float lob_pitch_step = 0.009f; 







//发射
#define DM_DIAL_SEND_ID 0x07 
#define DM_DIAL_RECEIVE_ID 0x17

//二级摩擦轮id
#define FRICTION_OUTER_LEFT_ID  1
#define FRICTION_OUTER_RIGHT_ID 2
#define FRICTION_OUTER_UP_ID    3

//一级摩擦轮id
#define FRICTION_INNER_LEFT_ID  5
#define FRICTION_INNER_RIGHT_ID 6
#define FRICTION_INNER_UP_ID    7



//目标转速
static uint8_t fric_level = 0; //键鼠摩擦轮档位 0:关,   1:12m/s,   2:16m/s



//摩擦轮半径

//拨盘
#define CONTINUOUS_FIRE_RATE 2000 //4200    //连发频率: 100ms/发(10Hz 100, 15Hz 66 , 20Hz 50, 25Hz 40)


//卡弹参数




volatile LED_State_e led_state = LED_STATE_BOOTING;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//初始化电机
void Motor_Init(Motor_Controller* motor, uint16_t id, FDCAN_HandleTypeDef* hfdcan);

//初始化达秒电机
void DM_Motor_Init(Motor_Controller* motor, uint8_t id, FDCAN_HandleTypeDef* hfdcan);


//初始化电机参数
void Robot_Param_Init(void);







//控制循环
void Robot_Control_Loop(const IMU_Data_t* imu_data);

//发射控制

//检测是否发射弹丸

//热量模拟

//是否允许开火


//CAN接收解码任务
void Can_Decode_Task(void const *argument);

//控制任务
void Gimbal_Control_task(void const * argument);

//重置弹道统计数据



//更新拨盘电机

//更新摩擦轮电机

//更新底盘发来的信息

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define DM4310_DIAL_T_MIN -7.0f
#define DM4310_DIAL_T_MAX 7.0f

#define DM4340_DIAL_T_MIN -12.0f
#define DM4340_DIAL_T_MAX 12.0f

#define DM6006_T_MIN -12.0f
#define DM6006_T_MAX 12.0f

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}




//初始化电机
void Motor_Init(Motor_Controller* motor, uint16_t id, FDCAN_HandleTypeDef* hfdcan)
{
    memset(&motor->feedback, 0, sizeof(Motor_Feedback));
    motor->id = id;
    motor->hfdcan = hfdcan;
    motor->target_world_angle = 0.0f;
    motor->mechanical_degree = 0.0f;
    motor->target_total_angle = 0.0f;
    motor->round_count = 0;
    motor->last_feedback_angle = 0;
    motor->current_total_angle = 0.0f;
    motor->target_speed = 0.0f;
    motor->target_current = 0;
}



//初始化电机参数
void Robot_Param_Init(void)
{
    //云台
    Gimbal_Control_Init();

    //拨盘
    Shooter_Control_Init();

    //底盘
    Chassis_Control_Init();
}





#define RX_DIAL_FDB_ID    0x302





















float VTM_SERVO_CLIMB = 100.0f;


static uint32_t reset_trigger_timer = 0;


float kyaw = 0.002f;
float yaw_step_max = 0.015f;

float kpitch = 0.015f;//0.015f
float pitch_step_max = 0.1f;//0.05f

//底盘参数




uint8_t is_aiming_now = 0;


float vision_yaw_deg = 0.0f;
float vision_pitch_deg = 0.0f;
float vision_target_yaw_angle = 0.0f;
float vision_target_pitch_angle = 0.0f;

//吊射
float vision_base_yaw_deg = 0.0f;
float vision_base_pitch_deg = 0.0f;


float vision_target_base_pitch_angle = 0.0f;








		





const RC_Data_t* rc;

void Robot_Control_Loop(const IMU_Data_t* imu_data)
{
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
									

		//VT03
    if(control_source == CONTROL_SOURCE_VTM) 
    {
        switch(rc->rc.s_left) 
        {
            case 1: //S
                robot_mode = ROBOT_MODE_BATTLE;
						    led_state = LED_STATE_ENABLED;
                break;
                
            case 3: //N
                robot_mode = ROBOT_MODE_FOLLOW;
					    	led_state = LED_STATE_ENABLED;
                break;
                
            case 2: //C
            default:
                robot_mode = ROBOT_MODE_OFF;
						    led_state = LED_STATE_DISABLED;
                break;
        }
    } 
		//DR16
    else if(control_source == CONTROL_SOURCE_DR16) 
    {
					//模式切换
					if(Chassis_Get_Climb_Mode() == CLIMB_MODE_STANDBY || Chassis_Get_Climb_Mode() == CLIMB_MODE_ACTIVE)
					{
							robot_mode = ROBOT_MODE_CLIMB;
					}
					else
					{
							if(rc->rc.s_left == DR16_SW_DOWN && rc->rc.s_right == DR16_SW_DOWN)
							{
									robot_mode = ROBOT_MODE_OFF;
									led_state = LED_STATE_DISABLED;
							}
							else if(rc->rc.s_left == DR16_SW_DOWN && rc->rc.s_right == DR16_SW_UP)
							{
									robot_mode = ROBOT_MODE_GIMBAL_ONLY;
									led_state = LED_STATE_ENABLED;
							}
							else if(rc->rc.s_left == DR16_SW_DOWN && rc->rc.s_right == DR16_SW_MID)
							{
									robot_mode = ROBOT_MODE_CHASSIS_ONLY;
									led_state = LED_STATE_DISABLED;
							}
							else if(rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_MID)
							{
									robot_mode = ROBOT_MODE_FOLLOW;
									led_state = LED_STATE_ENABLED;
							}
							else if(rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_MID)
							{
									robot_mode = ROBOT_MODE_SHOOT;
									led_state = LED_STATE_ENABLED;
							}
							else if(rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_DOWN)
							{
								  robot_mode = ROBOT_MODE_SHOOT_LOW;
									led_state = LED_STATE_ENABLED;
							}
							else if(rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_UP)
							{
									robot_mode = ROBOT_MODE_AIMING;
									led_state = LED_STATE_ENABLED;
							}
							else if(rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_UP)
							{
									robot_mode = ROBOT_MODE_M_AIMING;
									led_state = LED_STATE_ENABLED;
							}
							else if(rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_DOWN)
							{
									robot_mode = ROBOT_MODE_CLIMB;
									led_state = LED_STATE_ENABLED;
							}
					 }
    }
    else 
    { 
        //离线失能
        robot_mode = ROBOT_MODE_OFF;
    }
		
		
		
		
		static bool key_V_last_state = false;
		
		//RESET
    if(robot_mode == ROBOT_MODE_OFF)
    {
        if(rc->rc.wheel < -500)
        {
            reset_trigger_timer++;

            //持续足够长时间
            if(reset_trigger_timer >= 2000)
            {
						    reset_trigger_timer=0;
							  led_state = LED_STATE_RESET;
							
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
						led_state = LED_STATE_RESET;
					
						LK_Send_Torque(&hfdcan1, 0);
						DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);

						Reset_All_IMU_Yaw_Only();
					
						Gimbal_Set_World_Yaw(180.0f);

						Gimbal_Reset_Yaw_SMC();
				}
        else
        {
            reset_trigger_timer = 0;
        }
    }
    else reset_trigger_timer = 0;
		
		
    if(robot_mode == ROBOT_MODE_BATTLE)
    {
				if(rc->key.map.V && !key_V_last_state)
				{
						led_state = LED_STATE_RESET;
					
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
							

								fric_level = 0;
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
														 led_state = LED_STATE_BASE_HIT_READY;
												}
												else
												{
														 led_state = LED_STATE_ENABLED;
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
														led_state = LED_STATE_BASE_HIT_READY;
														lock_confirm_timer = 200;
												}
										}
										else if (yaw_error > 0.1f || actual_pitch_err > 0.1f)
										{
												lock_confirm_timer = 0;
												led_state = LED_STATE_ENABLED;
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
        if (rc->key.map.F && !f_key_last) fric_level = 1;
        f_key_last = rc->key.map.F;

        static bool g_key_last = false;
        if (rc->key.map.G && !g_key_last) fric_level = 2;
        g_key_last = rc->key.map.G;

        Shooter_Cmd_t shoot_cmd = {0};
        shoot_cmd.use_referee_heat_limit = (control_source == CONTROL_SOURCE_VTM);

        if (control_source == CONTROL_SOURCE_VTM)
        {
            if (fric_level == 1) shoot_cmd.fric_mode = SHOOTER_MODE_LOW_SPEED;
            else if (fric_level == 2) shoot_cmd.fric_mode = SHOOTER_MODE_HIGH_SPEED;
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
            (robot_mode == ROBOT_MODE_BATTLE && (fric_level == 1 || fric_level == 2)) ||
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



















//CAN接收解码任务
void Can_Decode_Task(void const *argument)
{
    CAN_Rx_Message_t rx_msg;

    for(;;)
    {
        if(osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK)
        {
            if(rx_msg.hfdcan->Instance == FDCAN1)
            {
								//云台Yaw电机反馈
								if(rx_msg.can_id == LK_YAW_ID)
                {
                    Gimbal_Update_Yaw(rx_msg.data);
                }
                //拨盘电机反馈
                else if(rx_msg.can_id == RX_DIAL_FDB_ID)
                {
                    Shooter_CAN_Feedback(rx_msg.can_id, rx_msg.data);
                }
								//双板通信
                else if(rx_msg.can_id == RX_CHASSIS_DATA_ID) 
                {
									  Chassis_Decode_Feedback(rx_msg.data);
                }
            }
            else if(rx_msg.hfdcan->Instance == FDCAN2) 
            {
							  //摩擦轮电机
                if(rx_msg.can_id >= 0x201 && rx_msg.can_id <= 0x207)
                {
                    Shooter_CAN_Feedback(rx_msg.can_id, rx_msg.data);
                }
            }
             else if(rx_msg.hfdcan->Instance == FDCAN3)
            {
                //云台Pitch电机反馈
                if(rx_msg.can_id == (0x200 + GIMBAL_PITCH_MOTOR_ID))
                {
                    Gimbal_Update_Pitch(rx_msg.data);
                }
//						   	//自瞄
//							  else if (rx_msg.can_id == VISION_RX_CAN_ID)
//                {
//                    Vision_CAN_Decode(rx_msg.can_id, rx_msg.data);
//                }
            }
        }
    }
}



//控制任务
void Gimbal_Control_task(void const * argument)
{
	
	IMU_Data_t imu_data_pack = {0};
 
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
	
  for(;;)
  {
		
				imu_data_pack.yaw_angle = total_yaw; 
				imu_data_pack.yaw_gyro  = gyro[2]; 
				imu_data_pack.pitch_angle = pitch;
				imu_data_pack.pitch_gyro = gyro[0];

		
				Robot_Control_Loop(&imu_data_pack);
		
	    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}
}




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







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    SCB_EnableICache();
    SCB_EnableDCache();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_SPI2_Init();
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART10_UART_Init();
  MX_SPI6_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
	DWT_Init();
	
	VT03_Init();
	
	Bsp_Servo_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB2;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

