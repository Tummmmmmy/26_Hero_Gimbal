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
#include "bsp_can.h"
#include "can_transmit.h"
/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ABS(x) ((x) > 0 ? (x) : -(x))

volatile Robot_Mode_e robot_mode = ROBOT_MODE_OFF;
volatile Gimbal_Mode_e gimbal_mode = GIMBAL_MODE_IMU;
static Climb_Mode_e climb_mode = CLIMB_MODE_OFF;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


//云台电机物理ID
#define GIMBAL_PITCH_MOTOR_ID 5


//双板通信CAN ID
#define RX_CHASSIS_DATA_ID 0x304

volatile uint8_t color = 2;//颜色 (0=蓝, 1=红)
uint8_t is_referee_online = 1;


uint8_t battle_mode = 0;

float Max_x = 80.0f;
float Max_y = 80.0f;
float Max_z = 80.0f;
float Max_Top_z = 163.0f;

float KEY_Top_z = 163.0f;

const float RPS_TO_RPM = 60.0f / (2.0f * PI);
const float DEG_TO_RAD = PI / 180.0f;
const float RAD_TO_DEG = 180.0f / PI;
const float ENCODER_TO_RAD = 2.0f * PI / 8192.0f;//编码值与弧度的转换系数
const float ENCODER_TO_DEG = 360.0f / 8192.0f;
const float DEG_TO_ENCODER = 8192.0f / 360.0f;
const float YAW_ENCODER_DIFF_TO_DEG = 360.0f / 65536.0f; 
const float YAW_ENCODER_DIFF_TO_RAD = 2.0f * PI / 65536.0f;

#define LK_RAW_TO_CURRENT  (16.5f / 2048.0f)
#define LK_RAW_TO_DPS      1.0f               // 1 LSB = 1 degree/second
#define LK_DPS_TO_RPM      (1.0f / 6.0f)      // dps -> rpm


#define GIMBAL_YAW_CENTER_ENCODER 30002

#define PITCH_HORIZON_DEGREE  121.1f  //枪管水平时的机械角度

//状态标志位
volatile uint8_t gimbal_control_initialized = 0;

Motor_Controller gimbal_yaw_motor;
Motor_Controller gimbal_pitch_motor;



SMC_Handle_t gimbal_yaw_smc_imu;
SMC_Handle_t gimbal_pitch_smc_imu;


//遥控器目标角度
//IMU
float rc_target_world_yaw_angle = 0.0f;
float rc_target_world_pitch_angle = 0.0f;
//Encoder
float rc_target_yaw_encoder = 0.0f;
float rc_target_pitch_encoder = 0.0f;



//视觉传入的目标
volatile float vision_target_world_yaw_angle = 0.0f;
volatile float vision_target_world_pitch_angle = 0.0f;
volatile uint8_t vision_is_tracking = 0;//视觉是否正在跟踪目标


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


const float RC_TO_ONE = 1.0f / 660.0f;


//Yaw
//速度环
const float gimbal_yaw_speed_kp_encoder = 0.25f;//0.45
const float gimbal_yaw_speed_ki_encoder = 0.0f;
const float gimbal_yaw_speed_kd_encoder = 3.25f;//4.25
const float gimbal_yaw_speed_Integral_Limit_encoder = 1.0f;
const float gimbal_yaw_speed_MaxOutput_encoder = 10.0f; //最大输出电流16.5A(max16.5A)
//位置环
const float gimbal_yaw_angle_kp_encoder = 0.195f;//之前同步带是0.1f
const float gimbal_yaw_angle_ki_encoder = 0.0f;
const float gimbal_yaw_angle_kd_encoder = 0.0f;
const float gimbal_yaw_angle_Integral_Limit_encoder = 0.0f;
const float gimbal_yaw_angle_MaxOutput_encoder = 30.0f; //最大目标速度RPM


//SMC
const float gimbal_yaw_c = 0.2f;          // c: 滑模面系数 ，决定回中快慢           0.35
const float gimbal_yaw_k_exp = 10.0f;      // k_exp: 指数趋近 ，决定出力大小          7.5
const float gimbal_yaw_epsilon = 0.0f;    // epsilon: 等速趋近 (克服静摩擦)，给小点防止震动
const float gimbal_yaw_beta = 0.0f;       // beta: 干扰观测器增益 ，不够大挂不住
const float gimbal_yaw_boundary = 0.5f;   // boundary: 边界层，防抖 
const float gimbal_yaw_out_max = 10.0f;    // out_max: 最大输出                       10
 
//SMC_Vision
float gimbal_yaw_c_vision = 3.5f;          // c: 滑模面系数 ，决定回中快慢        //1
float gimbal_yaw_k_exp_vision = 0.75f;      // k_exp: 指数趋近 ，决定出力大小      //0.3
const float gimbal_yaw_epsilon_vision = 0.0f;    // epsilon: 等速趋近 (克服静摩擦)，给小点防止震动
const float gimbal_yaw_beta_vision = 0.0f;       // beta: 干扰观测器增益 ，不够大挂不住
const float gimbal_yaw_boundary_vision = 0.5f;   // boundary: 边界层，防抖
const float gimbal_yaw_out_max_vision = 5.0f;    // out_max: 最大输出


//SMC_Rader
float gimbal_yaw_c_rader = 0.0f;          // c: 滑模面系数 ，决定回中快慢       
float gimbal_yaw_k_exp_rader = 0.0f;      // k_exp: 指数趋近 ，决定出力大小     
float gimbal_yaw_epsilon_rader = 0.0f;    // epsilon: 等速趋近 (克服静摩擦)




//Pitch

//速度环
const float gimbal_pitch_speed_kp_encoder = 105.0f;//450
const float gimbal_pitch_speed_ki_encoder = 0.1f;//15
const float gimbal_pitch_speed_kd_encoder = 0.0f;
const float gimbal_pitch_speed_Integral_Limit_encoder = 9000.0f;
const float gimbal_pitch_speed_MaxOutput_encoder = 25000.0f;//最大输出电流
//位置环
const float gimbal_pitch_angle_kp_encoder = 8.5f;//0.8
const float gimbal_pitch_angle_ki_encoder = 0.0f;
const float gimbal_pitch_angle_kd_encoder = 0.0f;
const float gimbal_pitch_angle_Integral_Limit_encoder = 50.0f;
const float gimbal_pitch_angle_MaxOutput_encoder = 400.0f;//最大目标速度RPM



//SMC
const float gimbal_pitch_c = 5.0f;          // c: 滑模面系数 ，决定回中快慢           0.35
const float gimbal_pitch_k_exp = 500.0f;      // k_exp: 指数趋近 ，决定出力大小          7.5
const float gimbal_pitch_epsilon = 0.0f;    // epsilon: 等速趋近(克服静摩擦)
const float gimbal_pitch_beta = 40.0f;       // beta: 干扰观测器增益
const float gimbal_pitch_boundary = 0.5f;   // boundary: 边界层，防抖 
const float gimbal_pitch_out_max = 25000.0f;    // out_max: 最大输出    

//SMC_Vision
float gimbal_pitch_c_vision = 7.5f;//7          // c: 滑模面系数 ，决定回中快慢    //前哨8.0f   
float gimbal_pitch_k_exp_vision = 700.0f;//600      // k_exp: 指数趋近 ，决定出力大小      //前哨550.0f
float gimbal_pitch_epsilon_vision = 0.0f;    // epsilon: 等速趋近 (克服静摩擦)
float gimbal_pitch_beta_vision = 40.0f;       // beta: 干扰观测器增益
float gimbal_pitch_boundary_vision = 0.5f;   // boundary: 边界层，防抖
float gimbal_pitch_out_max_vision = 25000.0f;    // out_max: 最大输出


//SMC_Rader
float gimbal_pitch_c_rader = 5.0f;          // c: 滑模面系数 ，决定回中快慢           //7
float gimbal_pitch_k_exp_rader = 200.0f;      // k_exp: 指数趋近 ，决定出力大小       //400
float gimbal_pitch_epsilon_rader = 3000.0f;    // epsilon: 等速趋近 (克服静摩擦)      //3000
float gimbal_pitch_beta_rader = 40.0f;       // beta: 干扰观测器增益                  //40


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


float chassis_follow_kp = 0.2f;//350.0f
float chassis_follow_ki = 0.0f;
float chassis_follow_kd = 0.0f;
float chassis_follow_Integral_Limit = 300.0f;
float chassis_follow_MaxOutput = 100.0f;//最大Wz指令值     //50.0f
PID_Controller chassis_follow_pid;


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

//初始化发射
void Gimbal_Angle_Update(Motor_Controller *motor, float encoder_range);





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

//更新云台Pitch电机
void update_gimbal_pitch(const uint8_t* data);

//更新云台Yaw电机
void update_gimbal_yaw(const uint8_t* data);

//更新拨盘电机

//更新摩擦轮电机

//更新底盘发来的信息
void update_chassis_data(const uint8_t *data);

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
    //Yaw
    Motor_Init(&gimbal_yaw_motor, 1 , &hfdcan1);

    //Pitch
		Motor_Init(&gimbal_pitch_motor, GIMBAL_PITCH_MOTOR_ID, &hfdcan3);

	  //拨盘
        Shooter_Control_Init();

  //Yaw
		SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
	  //Pitch
		SMC_Init(&gimbal_pitch_smc_imu, gimbal_pitch_c, gimbal_pitch_k_exp, gimbal_pitch_epsilon, gimbal_pitch_beta, gimbal_pitch_boundary, gimbal_pitch_out_max, 0.02f, 0.001f);		 

		//底盘跟随
	  PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, chassis_follow_Integral_Limit, chassis_follow_MaxOutput);
	
	  //初始化发射
}




//初始化发射
void Gimbal_Angle_Update(Motor_Controller *motor, float encoder_range)
{
    float half_range = encoder_range * 0.5f;//半圈值

    if(motor->last_feedback_angle == 0 && motor->round_count == 0 && motor->feedback.angle != 0)
    {
        motor->last_feedback_angle = motor->feedback.angle;
        motor->current_total_angle = (float)motor->feedback.angle;
        motor->mechanical_degree = (float)motor->feedback.angle * (360.0f / encoder_range);
        return;
    }
    
    int32_t angle_diff = (int32_t)motor->feedback.angle - (int32_t)motor->last_feedback_angle;

    //过零处理
    if(angle_diff > half_range) motor->round_count--;
    else if (angle_diff < -half_range) motor->round_count++;


    motor->current_total_angle = (float)(motor->round_count * encoder_range) + (float)motor->feedback.angle;//总角度 = 圈数 * 每圈编码值 + 当前单圈编码值
    motor->mechanical_degree = (float)motor->feedback.angle * (360.0f / encoder_range);//编码器值转换为机械角度度数
    motor->last_feedback_angle = motor->feedback.angle;
}
































#define RX_DIAL_FDB_ID    0x302



















int16_t friction_offset = 1500;

float VTM_SERVO_CLIMB = 100.0f;


static uint32_t reset_trigger_timer = 0;


float kyaw = 0.002f;
float yaw_step_max = 0.015f;

float kpitch = 0.015f;//0.015f
float pitch_step_max = 0.1f;//0.05f

//底盘参数
float Vx_RAMP_K_ACCEL = 200.0f;
float Vx_RAMP_K_DECEL = 1000.0f;
float Vy_RAMP_K_ACCEL = 800.0f;
float Vy_RAMP_K_DECEL = 1000.0f;
float Wz_RAMP_K_ACCEL = 300.0f;
float Wz_RAMP_K_DECEL = 500.0f;

float CLIMB_RAMP_K_ACCEL = 200.0f;
float CLIMB_RAMP_K_DECEL = 1000.0f;
float CLIMB_MAX_SPEED = 60.0f;

float climb_target_vx = 60.0f;
float climb_target_vy = 60.0f;


uint8_t is_aiming_now = 0;


float vision_yaw_deg = 0.0f;
float vision_pitch_deg = 0.0f;
float vision_target_yaw_angle = 0.0f;
float vision_target_pitch_angle = 0.0f;

//吊射
float vision_base_yaw_deg = 0.0f;
float vision_base_pitch_deg = 0.0f;


float vision_target_base_yaw_angle = 0.0f;
float vision_target_base_pitch_angle = 0.0f;


int16_t send_vx = 0, send_vy = 0, send_wz = 0;
uint8_t send_mode = 0; //0:Disable, 1:Enable
uint8_t supercap_cmd = 0;		uint8_t climb_cmd = 0; uint8_t led_cmd = 1;

float yaw_angle_diff =0.0f;

float yaw_angle_increment = 0;
float pitch_angle_increment = 0;
float yaw_encoder_increment = 0.0f;
float pitch_encoder_increment = 0.0f;


float target_yaw_speed = 0.0f;
float target_pitch_speed = 0.0f;
		

float yaw_encoder_diff = 0.0f;

float raw_vx = 0, raw_vy = 0;
float yaw_rad_diff = 0.0f;

float kf = 0.07f;//0.5
float Max = 2000.0f;
float Min = 150.0f;//150


const RC_Data_t* rc;

void Robot_Control_Loop(const IMU_Data_t* imu_data)
{
    static Robot_Mode_e last_robot_mode = ROBOT_MODE_OFF;
	
	  static uint8_t is_base_hitting_last_frame = 0;
  	uint8_t is_base_hitting_now = 0;
	
	  uint8_t is_base_locked = 0;//已锁定基地
    uint8_t key_shift = 0;//是否按下shift
	
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
					if(climb_mode == CLIMB_MODE_STANDBY || climb_mode == CLIMB_MODE_ACTIVE)
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
								gimbal_control_initialized = 0; 
            }
        }
				else if(rc->key.map.V && !key_V_last_state)
				{			
						led_state = LED_STATE_RESET;
					
						LK_Send_Torque(&hfdcan1, 0);
						DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);

						Reset_All_IMU_Yaw_Only();
					
						rc_target_world_yaw_angle = 180.0f;

						SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
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
					
						rc_target_world_yaw_angle = 180.0f;

						SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
				}
			}

		key_V_last_state = rc->key.map.V; //更新状态
		
		
		
		//重新上电后对齐
    static bool yaw_was_offline = false;
    if(gimbal_yaw_motor.is_online && yaw_was_offline)
		{
        rc_target_world_yaw_angle = imu_data->yaw_angle;
    }
    yaw_was_offline = !gimbal_yaw_motor.is_online;
		
		
    static bool pitch_was_offline = false;
    if(gimbal_pitch_motor.is_online && pitch_was_offline)
		{
        rc_target_world_pitch_angle = imu_data->pitch_angle;
    }
    pitch_was_offline = !gimbal_pitch_motor.is_online;
			

		
    //视觉离线
    if(!Vision_IsOnline()) 
    {
			  Clear_Vision_Target(); 
    }

		
		static uint8_t is_aiming_last_frame = 0;
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
		if(gimbal_control_initialized == 0 && (imu_data->yaw_angle != 0.0f && imu_data->pitch_angle != 0.0f))//陀螺仪第一次就绪
    {
        rc_target_world_yaw_angle = imu_data->yaw_angle;
        rc_target_world_pitch_angle = imu_data->pitch_angle;
        gimbal_control_initialized = 1;
    }
		
		
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
		
		
		
		static Gimbal_Mode_e last_gimbal_mode = GIMBAL_MODE_IMU;
    if(robot_mode == ROBOT_MODE_SHOOT || robot_mode == ROBOT_MODE_SHOOT_LOW || is_base_shooting)
	  {
        gimbal_mode = GIMBAL_MODE_ENCODER;
    }
	  else
	  {
        gimbal_mode = GIMBAL_MODE_IMU;
    }

    if(gimbal_mode != last_gimbal_mode)
    {
        if(gimbal_mode == GIMBAL_MODE_ENCODER )//发射模式
        {
						PID_Init(&gimbal_yaw_motor.pid_angle, gimbal_yaw_angle_kp_encoder, gimbal_yaw_angle_ki_encoder, gimbal_yaw_angle_kd_encoder, gimbal_yaw_angle_Integral_Limit_encoder, gimbal_yaw_angle_MaxOutput_encoder);
	         	PID_Init(&gimbal_yaw_motor.pid_speed, gimbal_yaw_speed_kp_encoder, gimbal_yaw_speed_ki_encoder, gimbal_yaw_speed_kd_encoder, gimbal_yaw_speed_Integral_Limit_encoder, gimbal_yaw_speed_MaxOutput_encoder);

  					gimbal_pitch_motor.pid_angle.Kp = gimbal_pitch_angle_kp_encoder;
            gimbal_pitch_motor.pid_angle.Ki = gimbal_pitch_angle_ki_encoder;
            gimbal_pitch_motor.pid_angle.Kd = gimbal_pitch_angle_kd_encoder;
            gimbal_pitch_motor.pid_angle.Integral_Limit = gimbal_pitch_angle_Integral_Limit_encoder;
            gimbal_pitch_motor.pid_angle.MaxOutput = gimbal_pitch_angle_MaxOutput_encoder;

            gimbal_pitch_motor.pid_speed.Kp = gimbal_pitch_speed_kp_encoder;
            gimbal_pitch_motor.pid_speed.Ki = gimbal_pitch_speed_ki_encoder;
            gimbal_pitch_motor.pid_speed.Kd = gimbal_pitch_speed_kd_encoder;
            gimbal_pitch_motor.pid_speed.Integral_Limit = gimbal_pitch_speed_Integral_Limit_encoder;
            gimbal_pitch_motor.pid_speed.MaxOutput = gimbal_pitch_speed_MaxOutput_encoder;


					  //目标更新为跟随模式结束时的位置
            rc_target_yaw_encoder = gimbal_yaw_motor.current_total_angle;
            rc_target_pitch_encoder = gimbal_pitch_motor.current_total_angle;
        }
        else//IMU
        {
						//Yaw
						SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);

					  rc_target_world_yaw_angle = imu_data->yaw_angle;
            rc_target_world_pitch_angle = imu_data->pitch_angle;
        }
    }
    last_gimbal_mode = gimbal_mode;
		

			
		
		
		//重新使能
		if(robot_mode != ROBOT_MODE_OFF && last_robot_mode == ROBOT_MODE_OFF)
    {		
		    SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
				SMC_Init(&gimbal_pitch_smc_imu, gimbal_pitch_c, gimbal_pitch_k_exp, gimbal_pitch_epsilon, gimbal_pitch_beta, gimbal_pitch_boundary, gimbal_pitch_out_max, 0.02f, 0.001f);		 

				Shooter_On_Robot_Start();
			
			  PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, chassis_follow_Integral_Limit, chassis_follow_MaxOutput);
			
        rc_target_world_yaw_angle = imu_data->yaw_angle;
        rc_target_world_pitch_angle = imu_data->pitch_angle;
    }
		
		
    if(is_aiming_now != is_aiming_last_frame)
    {
        if(is_aiming_now)//自瞄
        {
							gimbal_yaw_smc_imu.c = gimbal_yaw_c_vision;
							gimbal_yaw_smc_imu.k_exp = gimbal_yaw_k_exp_vision;
							gimbal_yaw_smc_imu.epsilon = gimbal_yaw_epsilon_vision;
							gimbal_yaw_smc_imu.beta = gimbal_yaw_beta_vision;
							gimbal_yaw_smc_imu.boundary = gimbal_yaw_boundary_vision;
							gimbal_yaw_smc_imu.out_max = gimbal_yaw_out_max_vision;
					
							gimbal_pitch_smc_imu.c = gimbal_pitch_c_vision;
							gimbal_pitch_smc_imu.k_exp = gimbal_pitch_k_exp_vision;
							gimbal_pitch_smc_imu.epsilon = gimbal_pitch_epsilon_vision;
							gimbal_pitch_smc_imu.beta = gimbal_pitch_beta_vision;
							gimbal_pitch_smc_imu.boundary = gimbal_pitch_boundary_vision;
							gimbal_pitch_smc_imu.out_max = gimbal_pitch_out_max_vision;
        }
        else
        {
							gimbal_yaw_smc_imu.c = gimbal_yaw_c;
							gimbal_yaw_smc_imu.k_exp = gimbal_yaw_k_exp;
							gimbal_yaw_smc_imu.epsilon = gimbal_yaw_epsilon;
							gimbal_yaw_smc_imu.beta = gimbal_yaw_beta;
							gimbal_yaw_smc_imu.boundary = gimbal_yaw_boundary;
							gimbal_yaw_smc_imu.out_max = gimbal_yaw_out_max;

							gimbal_pitch_smc_imu.c = gimbal_pitch_c_vision;
							gimbal_pitch_smc_imu.k_exp = gimbal_pitch_k_exp_vision;
							gimbal_pitch_smc_imu.epsilon = gimbal_pitch_epsilon_vision;
							gimbal_pitch_smc_imu.beta = gimbal_pitch_beta_vision;
							gimbal_pitch_smc_imu.boundary = gimbal_pitch_boundary_vision;
							gimbal_pitch_smc_imu.out_max = gimbal_pitch_out_max_vision;
        }	
    }
    is_aiming_last_frame = is_aiming_now;
		
		
		
		
		static uint8_t keyboard_climb_cmd = 0; 
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
		
		
    if(robot_mode == ROBOT_MODE_BATTLE)
    {
				if(rc->key.map.Q)
				{
						if(e_key_just_pressed)
						{
								keyboard_climb_cmd = 2; //Q+E: 前收后伸
						}
						else if(keyboard_climb_cmd != 2)
						{
								keyboard_climb_cmd = 1; //仅Q: 前伸
						}
						led_cmd = 0;
				}
				else
				{
						keyboard_climb_cmd = 0;//全收
						led_cmd = 1;
					
						if(e_key_just_pressed)
						{
								vtm_look_down = !vtm_look_down; 
						}
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
				climb_cmd = keyboard_climb_cmd;
    }
    else 
    {
					//上坎模式切换
		    	keyboard_climb_cmd = 0; 
					climb_cmd = 0;
					led_cmd = 1;
					
					if(robot_mode == ROBOT_MODE_OFF)
					{
							climb_mode = CLIMB_MODE_OFF;
					} 
					else
					{
							switch (climb_mode)
							{
									case CLIMB_MODE_OFF:
											
											if(rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_DOWN)
											{
													climb_mode = CLIMB_MODE_STANDBY;//待命
											}
											break;

									case CLIMB_MODE_STANDBY:
											if(rc->rc.wheel < -150)
											{
													climb_mode = CLIMB_MODE_ACTIVE;//激活
											}
											else if(robot_mode == ROBOT_MODE_OFF || rc->rc.s_left != DR16_SW_MID || rc->rc.s_right != DR16_SW_DOWN || rc->rc.wheel > 150)
											{
													climb_mode = CLIMB_MODE_OFF;//关闭
											}
											break;

									case CLIMB_MODE_ACTIVE:
											if(robot_mode == ROBOT_MODE_OFF || rc->rc.wheel > 150 || rc->rc.s_left != DR16_SW_MID)
											{
													climb_mode = CLIMB_MODE_OFF;//关闭
											}
											break;
							}
					}

					if(climb_mode == CLIMB_MODE_ACTIVE)
					{
							led_cmd = 0;
							if(rc->rc.s_right == DR16_SW_UP)        climb_cmd = 2;//前收后伸
							else if (rc->rc.s_right == DR16_SW_MID) climb_cmd = 1;//前伸
							else climb_cmd = 0;//全收
					}
    }
		
		
		
switch(robot_mode)
{
        case ROBOT_MODE_OFF://失能
        {
						if(last_robot_mode != ROBOT_MODE_OFF)
						{	
								if(gimbal_mode == GIMBAL_MODE_IMU)
								{
		                SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, 0, 0.0f, 0.001f);
										SMC_Init(&gimbal_pitch_smc_imu, gimbal_pitch_c, gimbal_pitch_k_exp, gimbal_pitch_epsilon, gimbal_pitch_beta, gimbal_pitch_boundary, 0, 0.02f, 0.001f);		 
								}
								else
								{
										PID_Init(&gimbal_yaw_motor.pid_angle, gimbal_yaw_angle_kp_encoder, gimbal_yaw_angle_ki_encoder, gimbal_yaw_angle_kd_encoder, 0, 0);
										PID_Init(&gimbal_yaw_motor.pid_speed, gimbal_yaw_speed_kp_encoder, gimbal_yaw_speed_ki_encoder, gimbal_yaw_speed_kd_encoder, 0, 0);
										PID_Init(&gimbal_pitch_motor.pid_angle, gimbal_pitch_angle_kp_encoder, gimbal_pitch_angle_ki_encoder, gimbal_pitch_angle_kd_encoder, 0, 0);
										PID_Init(&gimbal_pitch_motor.pid_speed, gimbal_pitch_speed_kp_encoder, gimbal_pitch_speed_ki_encoder, gimbal_pitch_speed_kd_encoder, 0, 0);
								}
							
								Shooter_On_Robot_Off();
								
							  PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, 0, 0);
							
								gimbal_yaw_motor.target_current = 0;
								gimbal_pitch_motor.target_current = 0;
							
                LK_Send_Torque(&hfdcan1, 0);
								DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x1FF, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x200, 0, 0, 0, 0);
								
								Dial_Send_Speed(&hfdcan1, 0);
							
								send_vx = 0; send_vy = 0; send_wz = 0;
                send_mode = 0;  supercap_cmd = 0;

								fric_level = 0;
						}
					
            if(gimbal_control_initialized && last_robot_mode == ROBOT_MODE_OFF)
            {
                rc_target_world_yaw_angle = imu_data->yaw_angle;
                rc_target_world_pitch_angle = imu_data->pitch_angle; 
            }
						
						battle_mode = BATTLE_OFF;
            break;
        }
				 case ROBOT_MODE_BATTLE://键鼠模式
        {
            static bool key_W_last_state = false;
            static bool key_S_last_state = false;

            static float chassis_vx_f = 0.0f;
            static float chassis_vy_f = 0.0f;
            static float chassis_wz_f = 0.0f;
					
            //底盘
            float target_vx = 0.0f;
            float target_vy = 0.0f;
			  		float target_wz = 0.0f;
            
					  static bool is_chassis_locked = false;  //底盘是否锁死
            static bool key_Z_last_state = false;
					
            if(rc->key.map.Z && !key_Z_last_state)
			  		{
                is_chassis_locked = !is_chassis_locked;
            }
            key_Z_last_state = rc->key.map.Z; //更新状态
					

					  //吊射模式
            if(is_base_shooting)
            {
                target_vx = 0.0f;
                target_vy = 0.0f;
                target_wz = 0.0f;
							
								chassis_vx_f = 0.0f;
								chassis_vy_f = 0.0f;
								chassis_wz_f = 0.0f;
                
                if(rc->key.map.A) rc_target_yaw_encoder -= lob_yaw_step;
                if(rc->key.map.D) rc_target_yaw_encoder += lob_yaw_step;
                
                if(rc->key.map.W) rc_target_pitch_encoder -= lob_pitch_step;
                if(rc->key.map.S) rc_target_pitch_encoder += lob_pitch_step;
							
								//Pitch限位
								if(rc_target_pitch_encoder > 2900.0f)
								{
										rc_target_pitch_encoder = 2900.0f;
								}
								else if(rc_target_pitch_encoder < 1750.0f)
								{
										rc_target_pitch_encoder = 1750.0f;
								}
								
								battle_mode = BATTLE_LOB;
            }
            else 
            {
									if(is_chassis_locked)//底盘锁定
									{
												target_vx = 0.0f;
												target_vy = 0.0f;
												target_wz = 0.0f;
											
												chassis_vx_f = 0.0f;
												chassis_vy_f = 0.0f;
												chassis_wz_f = 0.0f;
										
										    battle_mode = BATTLE_GIMBAL;
									}
									else
									{
										    battle_mode = BATTLE_FOLLOW;
									
												if(rc->key.map.W && !rc->key.map.G && !rc->key.map.F && !rc->key.map.R) target_vx = Max_x;
												else if(rc->key.map.S && !rc->key.map.G && !rc->key.map.F && !rc->key.map.R) target_vx = -Max_x;
												
												if(rc->key.map.A) target_vy = -Max_y;
												else if (rc->key.map.D) target_vy = Max_y;

												if(rc->key.map.W && (rc->key.map.A || rc->key.map.D))
												{
														target_vx *= 0.6f;
														target_vy *= 1.3f;
												}
												
												yaw_encoder_diff = (float)gimbal_yaw_motor.feedback.angle - GIMBAL_YAW_CENTER_ENCODER;
												while(yaw_encoder_diff > 32768) yaw_encoder_diff -= 65536.0f;
												while(yaw_encoder_diff < -32768) yaw_encoder_diff += 65536.0f;
												
												
												if(rc->key.map.SHIFT) //小陀螺
												{ 
														target_wz = KEY_Top_z;
														key_shift = 1;
													
														target_vx *= 1.1f;
														target_vy *= 1.7f;
													
												}
						//            if(rc->key.map.SHIFT)//变速小陀螺
						//            {
						//							   key_shift = 1;
						//						
						//                float time_s = (float)xTaskGetTickCount() * 0.001f; 
						//                
						//                //0.3Hz主浪，0.7Hz副浪
						//                float factor = 1.0f 
						//                             + 0.35f * sinf(2.0f * PI * 0.4f * time_s) 
						//                             + 0.15f * cosf(2.0f * PI * 0.93f * time_s);
						//                
						//                target_wz = KEY_Top_z * factor;
						//                
						//                target_vx *= 1.1f;
						//                target_vy *= 1.7f;
						//            }
												else//底盘跟随
												{
														if(is_aiming_now == 1 && yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD < 45.0f)
														{
															
														}
														else
														{
																float follow_Wz = yaw_encoder_diff * -kf;
															
																float calc_Wz = PID_Calculate(&chassis_follow_pid, 0.0f, follow_Wz);
															
																target_wz = calc_Wz;
															
																if(fabsf(target_wz) < Min * 0.01f)
																{
																		target_wz = 0.0f;
																}
																
																
																//转向优先
																float angle_error_deg = fabs(yaw_encoder_diff) * YAW_ENCODER_DIFF_TO_DEG;
																if(angle_error_deg > 8.0f) 
																{
																		float reduce_factor = 1.0f - ((angle_error_deg - 8.0f) / 37.0f);
																		if(reduce_factor < 0.2f) reduce_factor = 0.2f;
																		
																		target_vx *= reduce_factor;
																		target_vy *= reduce_factor;
																}
														}
												 }
								 	}
								 
								//坐标系转换				
								yaw_rad_diff = -yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD;
								float cos_yaw_diff = cosf(yaw_rad_diff);
								float sin_yaw_diff = sinf(yaw_rad_diff);
								 
								float final_target_vx  = target_vx * cos_yaw_diff + target_vy * sin_yaw_diff;
								float final_target_vy  = -target_vx * sin_yaw_diff + target_vy * cos_yaw_diff;

								
								
								float ramp_k_vx = 0.0f;;
								if(climb_cmd == 1)//上坎
								{
										if(rc->key.map.W) final_target_vx = climb_target_vx;
										else if(rc->key.map.S) final_target_vx = -climb_target_vx;
										 
										//速度限幅
										if(final_target_vx > CLIMB_MAX_SPEED) final_target_vx = CLIMB_MAX_SPEED;
										else if (final_target_vx < -CLIMB_MAX_SPEED) final_target_vx = -CLIMB_MAX_SPEED;
									 
										ramp_k_vx = (fabs(final_target_vx) > fabs(chassis_vx_f)) ? CLIMB_RAMP_K_ACCEL : CLIMB_RAMP_K_DECEL;
								}
								else
								{
										ramp_k_vx = (fabs(final_target_vx) > fabs(chassis_vx_f)) ? Vx_RAMP_K_ACCEL : Vx_RAMP_K_DECEL;
								}
								float ramp_k_vy = (fabs(final_target_vy) > fabs(chassis_vy_f)) ? Vy_RAMP_K_ACCEL : Vy_RAMP_K_DECEL;
								float ramp_k_wz = (fabs(target_wz) > fabs(chassis_wz_f)) ? Wz_RAMP_K_ACCEL : Wz_RAMP_K_DECEL;
								
								
								chassis_vx_f = RAMP_float(final_target_vx, chassis_vx_f, ramp_k_vx * 0.001f);
								chassis_vy_f = RAMP_float(final_target_vy, chassis_vy_f, ramp_k_vy * 0.001f);
								chassis_wz_f = RAMP_float(target_wz, chassis_wz_f, ramp_k_wz * 0.001f);
								
								
								
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
										 
												rc_target_world_yaw_angle += yaw_step;
										 

												vision_target_base_pitch_angle = 180.0f - vision_base_pitch_deg;
										 
												float pitch_diff = vision_target_base_pitch_angle - rc_target_world_pitch_angle;
										 
												float pitch_step = pitch_diff * kpitch;
												if(pitch_step > pitch_step_max)  pitch_step = pitch_step_max;
												if(pitch_step < -pitch_step_max) pitch_step = -pitch_step_max;
												
												rc_target_world_pitch_angle += pitch_step;
														
																
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
									
											if(fabs(vision_target_yaw_angle - rc_target_world_yaw_angle) < 90.0f && fabs(vision_target_pitch_angle - rc_target_world_pitch_angle) < 90.0f)
											{
													rc_target_world_yaw_angle = vision_target_yaw_angle;
													rc_target_world_pitch_angle = vision_target_pitch_angle;
											}
									
								} 
								else 
								{
										float yaw_increment = -(float)rc->mouse.x * RC_TO_ONE * ky_mouse;
										float pitch_increment = -(float)rc->mouse.y * RC_TO_ONE * kpit_mouse;
									
										rc_target_world_yaw_angle += yaw_increment;
										rc_target_world_pitch_angle += pitch_increment;
									
										VisionData.target_yaw_rad = 0.0f;
										VisionData.target_pitch_rad = 0.0f;
										vision_yaw_deg = 0.0f;
										vision_pitch_deg = 0.0f;
										vision_target_yaw_angle = rc_target_world_yaw_angle;
										vision_target_pitch_angle = rc_target_world_pitch_angle;
								}
					}
						
						send_vx = (int16_t)(chassis_vx_f * 100.0f);
						send_vy = (int16_t)(chassis_vy_f * 100.0f);
						send_wz = (int16_t)(chassis_wz_f * 100.0f);
						send_mode = 1;
						supercap_cmd = rc->request_super_cap ? 1 : 0;
					
						
						if(climb_mode == CLIMB_MODE_ACTIVE && climb_cmd == 1)
						{
								//Pitch限位
								if(rc_target_world_pitch_angle > 166.0f)
								{
										rc_target_world_pitch_angle = 166.0f;
								}
								else if(rc_target_world_pitch_angle < 135.5f)
								{
										rc_target_world_pitch_angle = 135.5f;
								}
						}
						else
						{
								//Pitch限位
								if(rc_target_world_pitch_angle > 194.1f)
								{
										rc_target_world_pitch_angle = 194.1f;
								}
								else if(rc_target_world_pitch_angle < 135.5f)
								{
										rc_target_world_pitch_angle = 135.5f;
								}
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
									 
											rc_target_world_yaw_angle += yaw_step;
									 
											vision_target_base_pitch_angle = 180.0f - vision_base_pitch_deg;
									 
											float pitch_diff = vision_target_base_pitch_angle - rc_target_world_pitch_angle;
									 
											float pitch_step = pitch_diff * kpitch;
											if(pitch_step > pitch_step_max)  pitch_step = pitch_step_max;
											if(pitch_step < -pitch_step_max) pitch_step = -pitch_step_max;
											
											rc_target_world_pitch_angle += pitch_step;
													
															
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

									if(fabs(vision_target_yaw_angle - rc_target_world_yaw_angle) < 60.0f && fabs(vision_target_pitch_angle - rc_target_world_pitch_angle) < 60.0f)
									{
											rc_target_world_yaw_angle = vision_target_yaw_angle;
											rc_target_world_pitch_angle = vision_target_pitch_angle;
									}
            }
						else
						{
									//更新目标角度
									yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
									pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;
									
									rc_target_world_yaw_angle += yaw_angle_increment;
									rc_target_world_pitch_angle += pitch_angle_increment;
							
									VisionData.target_yaw_rad = 0.0f;
									VisionData.target_pitch_rad = 0.0f;
									vision_yaw_deg = 0.0f;
									vision_pitch_deg = 0.0f;
									vision_target_yaw_angle = rc_target_world_yaw_angle;
									vision_target_pitch_angle = rc_target_world_pitch_angle;
						}
						
						//Pitch限位
						if(rc_target_world_pitch_angle > 194.1f)
						{
								rc_target_world_pitch_angle = 194.1f;
						}
						else if(rc_target_world_pitch_angle < 135.5f)
						{
								rc_target_world_pitch_angle = 135.5f;
						}
		
						raw_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
            raw_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;
						
						yaw_encoder_diff = (float)gimbal_yaw_motor.feedback.angle - GIMBAL_YAW_CENTER_ENCODER;
            while(yaw_encoder_diff > 32768) yaw_encoder_diff -= 65536.0f;
            while(yaw_encoder_diff < -32768) yaw_encoder_diff += 65536.0f;
						
						
						//坐标系转换				
						yaw_rad_diff = -yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD;
						float cos_yaw_diff = cosf(yaw_rad_diff);
						float sin_yaw_diff = sinf(yaw_rad_diff);
						
						float final_vx = raw_vx * cos_yaw_diff + raw_vy * sin_yaw_diff;
						float final_vy = -raw_vx * sin_yaw_diff + raw_vy * cos_yaw_diff;
				
						 
            send_vx = (int16_t)(final_vx * 100.0f);
            send_vy = (int16_t)(final_vy * 100.0f);
            send_mode = 1;
						supercap_cmd = 0;
						
						Shooter_Control_Disable();
					
         break;
        }
				
				case ROBOT_MODE_FOLLOW:
        {
						//更新目标角度
						yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
						pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;
						
						rc_target_world_yaw_angle += yaw_angle_increment;
						rc_target_world_pitch_angle += pitch_angle_increment;

						
						//Pitch限位
						if(rc_target_world_pitch_angle > 194.5f)
						{
								rc_target_world_pitch_angle = 194.5f;
						}
						else if(rc_target_world_pitch_angle < 135.5f)
						{
								rc_target_world_pitch_angle = 135.5f;
						}
						
						raw_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
            raw_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;
						
						yaw_encoder_diff = (float)gimbal_yaw_motor.feedback.angle - GIMBAL_YAW_CENTER_ENCODER;
            while(yaw_encoder_diff > 32768) yaw_encoder_diff -= 65536.0f;
            while(yaw_encoder_diff < -32768) yaw_encoder_diff += 65536.0f;
						
            if(ABS(rc->rc.wheel) > 2 && rc->rc.s_left == DR16_SW_MID) //小陀螺
            {
                send_wz  = (int16_t)(rc->rc.wheel * RC_TO_ONE  * Max_Top_z * 100.0f);//放大发送保留精度，底盘接收后需要/100
            }
            else//底盘跟随
            {
									float follow_Wz = yaw_encoder_diff * -kf;
								
									float calc_Wz = PID_Calculate(&chassis_follow_pid, 0.0f, follow_Wz);
								
									send_wz = (int16_t)(calc_Wz * 100.0f); // 放大发送
								
									if(ABS(send_wz) < Min)
									{
											send_wz = 0;
									}
            }
						
						//坐标系转换				
						yaw_rad_diff = -yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD;
						float cos_yaw_diff = cosf(yaw_rad_diff);
						float sin_yaw_diff = sinf(yaw_rad_diff);
						
						float final_vx = raw_vx * cos_yaw_diff + raw_vy * sin_yaw_diff;
						float final_vy = -raw_vx * sin_yaw_diff + raw_vy * cos_yaw_diff;
				
						 
            send_vx = (int16_t)(final_vx * 100.0f);
            send_vy = (int16_t)(final_vy * 100.0f);
            send_mode = 1;
						
						supercap_cmd = rc->request_super_cap ? 1 : 0;
						
						
						if(control_source != CONTROL_SOURCE_VTM) 
						{
						Shooter_Control_Disable();
						}
					
						battle_mode = BATTLE_FOLLOW;
         break;
        }
        
        case ROBOT_MODE_SHOOT:
				case ROBOT_MODE_SHOOT_LOW:
        {
						//更新目标角度
						yaw_angle_increment = (float)rc->rc.right_x * RC_TO_ONE * ky_shoot ;
						pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_shoot;
						
						yaw_encoder_increment = yaw_angle_increment * DEG_TO_ENCODER;
						pitch_encoder_increment = pitch_angle_increment * DEG_TO_ENCODER;
					
						rc_target_yaw_encoder += yaw_encoder_increment;
						rc_target_pitch_encoder += pitch_encoder_increment;
					
						//Pitch限位
						if(rc_target_pitch_encoder > 2900.0f)
						{
								rc_target_pitch_encoder = 2900.0f;
						}
						else if(rc_target_pitch_encoder < 1750.0f)
						{
								rc_target_pitch_encoder = 1750.0f;
						}
						
 						send_vx = 0; send_vy = 0; send_wz = 0;
						send_mode = 0	; supercap_cmd = 0;
            
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

							 
									float align_yaw_diff = vision_target_yaw_angle - imu_data->yaw_angle;
									while (align_yaw_diff > 180.0f)  align_yaw_diff -= 360.0f;
									while (align_yaw_diff < -180.0f) align_yaw_diff += 360.0f;
											
									if(fabs(vision_target_yaw_angle - rc_target_world_yaw_angle) < 60.0f && fabs(vision_target_pitch_angle - rc_target_world_pitch_angle) < 60.0f)
									{
											rc_target_world_yaw_angle = vision_target_yaw_angle;
											rc_target_world_pitch_angle = vision_target_pitch_angle;
									}
            }
						else
						{
									//更新目标角度
									yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
									pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;
									
									rc_target_world_yaw_angle += yaw_angle_increment;
									rc_target_world_pitch_angle += pitch_angle_increment;
							
									VisionData.target_yaw_rad = 0.0f;
									VisionData.target_pitch_rad = 0.0f;
									vision_yaw_deg = 0.0f;
									vision_pitch_deg = 0.0f;
									vision_target_yaw_angle = rc_target_world_yaw_angle;
									vision_target_pitch_angle = rc_target_world_pitch_angle;
						}
						
						//Pitch限位
						if(rc_target_world_pitch_angle > 194.1f)
						{
								rc_target_world_pitch_angle = 194.1f;
						}
						else if(rc_target_world_pitch_angle < 135.5f)
						{
								rc_target_world_pitch_angle = 135.5f;
						}
						
						raw_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
            raw_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;
						
						yaw_encoder_diff = (float)gimbal_yaw_motor.feedback.angle - GIMBAL_YAW_CENTER_ENCODER;
            while(yaw_encoder_diff > 32768) yaw_encoder_diff -= 65536.0f;
            while(yaw_encoder_diff < -32768) yaw_encoder_diff += 65536.0f;
						
            if(ABS(rc->rc.wheel) > 2 )//&& rc->rc.s_left == RC_SW_MID) //小陀螺
            {
                send_wz = (int16_t)(rc->rc.wheel * RC_TO_ONE  * Max_Top_z * 1.5f * 100.0f);//放大发送保留精度，底盘接收后需要/100
            }
						else
						{
							send_wz = 0;
						}
						
						//坐标系转换				
						yaw_rad_diff = -yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD;
						float cos_yaw_diff = cosf(yaw_rad_diff);
						float sin_yaw_diff = sinf(yaw_rad_diff);
						
						float final_vx = raw_vx * cos_yaw_diff + raw_vy * sin_yaw_diff;
						float final_vy = -raw_vx * sin_yaw_diff + raw_vy * cos_yaw_diff;
				
						 
            send_vx = (int16_t)(final_vx * 100.0f);
            send_vy = (int16_t)(final_vy * 100.0f);
            send_mode = 1;
						
						supercap_cmd = rc->request_super_cap ? 1 : 0;
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
									
									if(fabs(vision_target_yaw_angle - rc_target_world_yaw_angle) < 60.0f && fabs(vision_target_pitch_angle - rc_target_world_pitch_angle) < 60.0f)
									{
											rc_target_world_yaw_angle = vision_target_yaw_angle;
											rc_target_world_pitch_angle = vision_target_pitch_angle;
									}
            }
						else
						{
									//更新目标角度
									yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
									pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;
									
									rc_target_world_yaw_angle += yaw_angle_increment;
									rc_target_world_pitch_angle += pitch_angle_increment;
							
									VisionData.target_yaw_rad = 0.0f;
									VisionData.target_pitch_rad = 0.0f;
									vision_yaw_deg = 0.0f;
									vision_pitch_deg = 0.0f;
									vision_target_yaw_angle = rc_target_world_yaw_angle;
									vision_target_pitch_angle = rc_target_world_pitch_angle;
						}
						
						//Pitch限位
						if(rc_target_world_pitch_angle > 194.1f)
						{
								rc_target_world_pitch_angle = 194.1f;
						}
						else if(rc_target_world_pitch_angle < 135.5f)
						{
								rc_target_world_pitch_angle = 135.5f;
						}

						raw_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
            raw_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;
						
						yaw_encoder_diff = (float)gimbal_yaw_motor.feedback.angle - GIMBAL_YAW_CENTER_ENCODER;
            while(yaw_encoder_diff > 32768) yaw_encoder_diff -= 65536.0f;
            while(yaw_encoder_diff < -32768) yaw_encoder_diff += 65536.0f;
						
						
						//坐标系转换				
						yaw_rad_diff = -yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD;
						float cos_yaw_diff = cosf(yaw_rad_diff);
						float sin_yaw_diff = sinf(yaw_rad_diff);
						
						float final_vx = raw_vx * cos_yaw_diff + raw_vy * sin_yaw_diff;
						float final_vy = -raw_vx * sin_yaw_diff + raw_vy * cos_yaw_diff;
				
						 
            send_vx = (int16_t)(final_vx * 100.0f);
            send_vy = (int16_t)(final_vy * 100.0f);
            send_mode = 1;
						
						supercap_cmd = rc->request_super_cap ? 1 : 0;
            break;
				}

        case ROBOT_MODE_CHASSIS_ONLY:
        {
            float chassis_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
            float chassis_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;
            float chassis_wz = (float)rc->rc.right_x * RC_TO_ONE * Max_z;
            
            send_vx = (int16_t)(chassis_vx * 100.0f);
            send_vy = (int16_t)(chassis_vy * 100.0f);
            send_wz = (int16_t)(chassis_wz * 100.0f);
            send_mode = 1;
            if(ABS(rc->rc.left_y) > 600) supercap_cmd = 1;
            else supercap_cmd = 0;

						Shooter_Control_Disable();
					
				    rc_target_world_yaw_angle = imu_data->yaw_angle;
            rc_target_world_pitch_angle = imu_data->pitch_angle;
            break;
        }
				case ROBOT_MODE_CLIMB:
        {
					 pitch_angle_increment = -(float)rc->rc.left_x * RC_TO_ONE * kpit_follow;	
					 rc_target_world_pitch_angle += pitch_angle_increment;
					
					
					if(climb_mode == CLIMB_MODE_ACTIVE && climb_cmd == 1)
					{
							//Pitch限位
							if(rc_target_world_pitch_angle > 166.0f)
							{
									rc_target_world_pitch_angle = 166.0f;
							}
							else if(rc_target_world_pitch_angle < 135.5f)
							{
									rc_target_world_pitch_angle = 135.5f;
							}
					}
					else
					{
							//Pitch限位
							if(rc_target_world_pitch_angle > 194.1f)
							{
									rc_target_world_pitch_angle = 194.1f;
							}
							else if(rc_target_world_pitch_angle < 135.5f)
							{
									rc_target_world_pitch_angle = 135.5f;
							}
					}
					
            float chassis_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
            float chassis_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;
            float chassis_wz = (float)rc->rc.right_x * RC_TO_ONE * Max_z;
            
            send_vx = (int16_t)(chassis_vx * 100.0f);
            send_vy = (int16_t)(chassis_vy * 100.0f);
            send_wz = (int16_t)(chassis_wz * 100.0f);
            send_mode = 1;
            supercap_cmd = 0;
						
					if(last_robot_mode  != ROBOT_MODE_CLIMB)
					{
							rc_target_world_yaw_angle = imu_data->yaw_angle;
					}

            Shooter_Control_Disable();
            break;
        }
 }


		 if(climb_cmd != 0)
		 {
			    send_vy = 0;
					send_wz = 0;
					rc_target_world_yaw_angle = imu_data->yaw_angle;
		 }
		 
		 
		//雷达辅吊云台参数切换
    if(is_base_hitting_now != is_base_hitting_last_frame)
    {
        if(is_base_hitting_now)
        {
            gimbal_pitch_smc_imu.c = gimbal_pitch_c_rader;
            gimbal_pitch_smc_imu.k_exp = gimbal_pitch_k_exp_rader;
            gimbal_pitch_smc_imu.epsilon = gimbal_pitch_epsilon_rader; 
					  gimbal_pitch_smc_imu.beta = gimbal_pitch_beta_rader; 
        }
        else
        {
            gimbal_pitch_smc_imu.c = gimbal_pitch_c;
            gimbal_pitch_smc_imu.k_exp = gimbal_pitch_k_exp;
            gimbal_pitch_smc_imu.epsilon = gimbal_pitch_epsilon;
						gimbal_pitch_smc_imu.beta = gimbal_pitch_beta;
        }
    }
    is_base_hitting_last_frame = is_base_hitting_now; 
		 
 
    float gimbal_yaw_current = 0.0f;
    int16_t gimbal_pitch_current = 0;
    
    if(robot_mode != ROBOT_MODE_CHASSIS_ONLY && robot_mode != ROBOT_MODE_OFF)
    {
			  if(gimbal_mode == GIMBAL_MODE_IMU)//跟随
        {
					  //Yaw
					  yaw_angle_diff = rc_target_world_yaw_angle - imu_data->yaw_angle;
            while(yaw_angle_diff > 180.0f) yaw_angle_diff -= 360.0f;
            while(yaw_angle_diff < -180.0f) yaw_angle_diff += 360.0f;
					
				  	gimbal_yaw_smc_imu.ref_pos = rc_target_world_yaw_angle; // 目标 deg
            gimbal_yaw_smc_imu.ref_spd = 0.0f;
            gimbal_yaw_smc_imu.fdb_pos = imu_data->yaw_angle;     // 反馈 deg
            gimbal_yaw_smc_imu.fdb_spd = imu_data->yaw_gyro;      // 反馈 deg/s
            
            gimbal_yaw_current  = -SMC_Calc(&gimbal_yaw_smc_imu);

					
					
						//Pitch
						float pitch_angle_diff = rc_target_world_pitch_angle - imu_data->pitch_angle;

            gimbal_pitch_smc_imu.ref_pos = rc_target_world_pitch_angle;
            gimbal_pitch_smc_imu.ref_spd = 0.0f; //目标速度为0
            gimbal_pitch_smc_imu.fdb_pos = imu_data->pitch_angle;
            gimbal_pitch_smc_imu.fdb_spd = imu_data->pitch_gyro;
            
            gimbal_pitch_current = (int16_t)SMC_Calc(&gimbal_pitch_smc_imu);
						


						if(is_base_hitting_now)
						{
								float current_err = rc_target_world_pitch_angle - imu_data->pitch_angle;
								
								if(current_err > 0.05f) 
								{
										gimbal_pitch_current += friction_offset;
								}
								else if (current_err < -0.05f) 
								{
										gimbal_pitch_current -= friction_offset;
								}
						}
						
					  gimbal_yaw_motor.pid_angle.target = rc_target_world_yaw_angle;
            gimbal_yaw_motor.pid_angle.measured = imu_data->yaw_angle;
            gimbal_yaw_motor.pid_speed.target = target_yaw_speed;
            gimbal_yaw_motor.pid_speed.measured = imu_data->yaw_gyro;
            gimbal_yaw_motor.target_current = gimbal_yaw_current;
						gimbal_pitch_motor.pid_angle.target = rc_target_world_pitch_angle;
						gimbal_pitch_motor.pid_angle.measured = imu_data->pitch_angle;
						gimbal_pitch_motor.pid_speed.target = target_pitch_speed;
						gimbal_pitch_motor.pid_speed.measured = imu_data->pitch_gyro;
						gimbal_pitch_motor.target_current = gimbal_pitch_current;
				}
				else
				{
					  //Yaw
					  float yaw_encoder_diff = rc_target_yaw_encoder - gimbal_yaw_motor.current_total_angle;
            float target_yaw_speed = PID_Calculate(&gimbal_yaw_motor.pid_angle, yaw_encoder_diff, 0.0f);
            gimbal_yaw_current = PID_Calculate(&gimbal_yaw_motor.pid_speed, target_yaw_speed, imu_data->yaw_gyro);

					
					  //Pitch
						float pitch_encoder_diff = rc_target_pitch_encoder - gimbal_pitch_motor.current_total_angle;
					  float target_pitch_speed = PID_Calculate(&gimbal_pitch_motor.pid_angle, pitch_encoder_diff, 0.0f);

					  gimbal_pitch_current = (int16_t)PID_Calculate(&gimbal_pitch_motor.pid_speed, target_pitch_speed, imu_data->pitch_gyro);
					
						gimbal_yaw_motor.pid_angle.target = rc_target_yaw_encoder;
						gimbal_yaw_motor.pid_angle.measured = gimbal_yaw_motor.current_total_angle;
						gimbal_yaw_motor.pid_speed.target = target_yaw_speed;
						gimbal_yaw_motor.pid_speed.measured = imu_data->yaw_gyro;
						gimbal_yaw_motor.target_current = (int16_t)(gimbal_yaw_current);
						gimbal_pitch_motor.pid_angle.target = rc_target_pitch_encoder;
						gimbal_pitch_motor.pid_angle.measured = gimbal_pitch_motor.current_total_angle;
						gimbal_pitch_motor.pid_speed.target = target_pitch_speed;
						gimbal_pitch_motor.pid_speed.measured = imu_data->pitch_gyro;
						gimbal_pitch_motor.target_current = gimbal_pitch_current;
				}
				
		 }
		else
    {  
			  gimbal_yaw_current = 0.0f;
        gimbal_pitch_current = 0.0f;
        gimbal_yaw_motor.target_current = 0;
        gimbal_pitch_motor.target_current = 0;
    }
		
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

	    	LK_Send_Torque(&hfdcan1, gimbal_yaw_current);
		
		    DJI_Motor_Send_Command(&hfdcan3, 0x1FF, gimbal_pitch_current, 0, 0, 0);


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
        Send_Chassis_Command(&hfdcan1, send_vx, send_vy, send_wz, send_mode, supercap_cmd, climb_cmd, led_cmd, key_shift, friction_report, is_base_locked, battle_mode);

		    Dial_Send_Speed(&hfdcan1, shooter_out->dial_target_speed);
		
				static uint32_t ui_send_cnt = 0;
				if (++ui_send_cnt >= 50) 
				{
						ui_send_cnt = 0;
						
						float yaw_deg = gimbal_yaw_motor.mechanical_degree; 
						float pit_deg = PITCH_HORIZON_DEGREE - gimbal_pitch_motor.mechanical_degree;

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
                    update_gimbal_yaw(rx_msg.data);
                }
                //拨盘电机反馈
                else if(rx_msg.can_id == RX_DIAL_FDB_ID)
                {
                    Shooter_CAN_Feedback(rx_msg.can_id, rx_msg.data);
                }
								//双板通信
                else if(rx_msg.can_id == RX_CHASSIS_DATA_ID) 
                {
									  update_chassis_data(rx_msg.data);
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
                    update_gimbal_pitch(rx_msg.data);
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






//更新云台Yaw电机(LK)
void update_gimbal_yaw(const uint8_t* data)
{
    //校验命令字
    if(data[0] != 0xA1) return;

    gimbal_yaw_motor.feedback.temperature = (int8_t)data[1];


    int16_t raw_current = (int16_t)((data[3] << 8) | data[2]);
    gimbal_yaw_motor.feedback.torque = raw_current * LK_RAW_TO_CURRENT;//力矩，实际上是电流(安培)
    gimbal_yaw_motor.feedback.current = raw_current; //原始值也存一下

    
    int16_t raw_speed = (int16_t)((data[5] << 8) | data[4]);
    gimbal_yaw_motor.feedback.velocity = (float)raw_speed;


    uint16_t raw_encoder = (uint16_t)((data[7] << 8) | data[6]);
    gimbal_yaw_motor.feedback.angle = raw_encoder; //(0 ~ 65535)

	
    gimbal_yaw_motor.last_update_tick = osKernelGetTickCount();

    Gimbal_Angle_Update(&gimbal_yaw_motor, 65536.0f);
}




//更新云台Pitch电机
void update_gimbal_pitch(const uint8_t* data)
{
    gimbal_pitch_motor.feedback.angle = (uint16_t)(data[0] << 8) | data[1];
    gimbal_pitch_motor.feedback.actual_speed = (int16_t)((data[2] << 8) | data[3]);
    gimbal_pitch_motor.feedback.current = (int16_t)((data[4] << 8) | data[5]);
    gimbal_pitch_motor.feedback.temperature = data[6];
    gimbal_pitch_motor.last_update_tick = osKernelGetTickCount();
    Gimbal_Angle_Update(&gimbal_pitch_motor, 8192.0f);
}



//底盘发给云台的数据包
typedef union
{
    uint8_t data[8];
    
    __packed struct
    {
        float bullet_speed;     
        
        uint32_t shooter_heat   : 10; //当前热量 (0~1023)
        uint32_t heat_limit     : 9;  //热量上限 (0~511)
        uint32_t cooling_value  : 7;  //冷却速率 (0~127)
        uint32_t color          : 1;  //颜色 (0=蓝, 1=红)
        uint32_t ref_online     : 1;  //裁判系统在线 (0=离线, 1=在线)
			
			
        uint32_t reserved       : 4;  //剩余4位
    } pack;
} Chassis_To_Gimbal_Msg_t;


// 更新底盘发来的信息
void update_chassis_data(const uint8_t *data)
{
    Chassis_To_Gimbal_Msg_t rx_msg;
    
    memcpy(rx_msg.data, data, 8);

    Shooter_Set_Referee_Data(rx_msg.pack.shooter_heat, rx_msg.pack.heat_limit, rx_msg.pack.cooling_value, rx_msg.pack.bullet_speed);
    color        = rx_msg.pack.color;
    is_referee_online    = rx_msg.pack.ref_online;
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
