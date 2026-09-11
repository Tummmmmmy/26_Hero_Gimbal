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
#define LK_YAW_ID 0x141
#define GIMBAL_PITCH_MOTOR_ID 5


//双板通信CAN ID
#define TX_CHASSIS_COMMAND_ID 0x300
#define TX_CHASSIS_UI_ID 0x308
#define RX_CHASSIS_DATA_ID 0x304

volatile uint8_t color = 2;//颜色 (0=蓝, 1=红)
volatile uint16_t shooter_heat = 0;
volatile float initial_speed = 0.0f;
volatile uint16_t heat_limit = 100.0f;
volatile uint16_t shooter_cooling = 20.0f;
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

#define LK_CURRENT_TO_RAW  (2048.0f / 16.5f)  // Amps -> -2048~2048
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


uint32_t gap = 200;
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
float ratio = 0.784f;

//16.5m/s
uint32_t high_speed = 4450.0f;       //4990.0     5090.0f      5140.0f
uint16_t fric_offset = 0;
#define FRICTION_TARGET_SPEED_HIGH high_speed
#define FRICTION_RATIO_HIGH ratio

//12m/s
uint32_t low_speed = 3210.0f;
float low_ratio = 0.56f;
#define FRICTION_TARGET_SPEED_LOW  low_speed

#define FRICTION_RATIO_LOW        low_ratio


//摩擦轮半径
#define WHEEL_RADIUS_INNER  35.0f //一级半径
#define WHEEL_RADIUS_OUTER  35.0f // 二级半径  30.0f

//拨盘
#define DIAL_ONE_SHOOT_ENCODER PI / 3.0f    //一发编码器值
#define CONTINUOUS_FIRE_RATE 2000 //4200    //连发频率: 100ms/发(10Hz 100, 15Hz 66 , 20Hz 50, 25Hz 40)


//卡弹参数
#define DIAL_JAM_CURRENT_THRESHOLD 6.0f     //判定卡弹的电流阈值
#define DIAL_JAM_SPEED_THRESHOLD 1.5f       //判定卡弹的速度阈值(rpm)
#define DIAL_JAM_DETECT_THRESHOLD 100       //持续100ms判定为卡弹
#define DIAL_JAM_WAIT_TIME 400              //回退后等待400ms


float dial_back = 0.445f;//0.49

//校准参数
#define DIAL_HOMING_SPEED -2.2f  //-2.5            //反转速度    2.2f
#define DIAL_HOMING_CURRENT_THRESHOLD 1.0f   //撞限位的电流阈值   0.75f
#define DIAL_HOMING_SPEED_THRESHOLD 2.05f     //撞限位的速度阈值     1.1f
#define DIAL_HOMING_OFFSET dial_back         //撞限位后回退距离
#define DIAL_HOMING_CHECK_DELAY 200          //刚启动反转时忽略电流检测的时间，防误判


//热量参数
float shooter_predict_heat = 0.0f;          //预测热量
#define HERO_HEAT_PER_SHOT 100.0f           //英雄每发热量
#define HEAT_SAFETY_MARGIN 5.0f             //安全余量(两发间隔)


 //记录上次发射时刻
 uint32_t last_shoot_time = 0;

Shooter_Controller_t shooter_controller;
Shot_Detection_e current_shot_status = SHOT_NOT_DETECTED;



Motor_Controller dial_motor;
Motor_Controller friction_motor_inner_left;
Motor_Controller friction_motor_inner_right;
Motor_Controller friction_motor_inner_up;

Motor_Controller friction_motor_outer_left;
Motor_Controller friction_motor_outer_right;
Motor_Controller friction_motor_outer_up;



//拨盘电机PID参数
//位置环
const float dial_angle_kp = 12.0f;//15
const float dial_angle_ki = 0.0f;
const float dial_angle_kd = 0.0f;
const float dial_angle_Integral_Limit = 1.0f;
const float dial_angle_MaxOutput = 25.0f;//最大目标速度RPM

//速度环
const float dial_speed_kp = 0.3f;
const float dial_speed_ki = 0.0f;
const float dial_speed_kd = 0.0f;
const float dial_speed_Integral_Limit = 2.0f;
const float dial_speed_MaxOutput = 5.0f;



//摩擦轮电机PID参数
//16.5m/s
float friction_speed_kp_inner_high = 4.0f;
float friction_speed_ki_inner_high = 0.0f;
float friction_speed_kd_inner_high = 0.0f;

float friction_speed_kp_outer_high = 10.0f;
float friction_speed_ki_outer_high = 0.0f;
float friction_speed_kd_outer_high = 0.0f;

//12m/s
float friction_speed_kp_inner_low = 8.0f;
float friction_speed_ki_inner_low = 0.0f;
float friction_speed_kd_inner_low = 0.0f;

float friction_speed_kp_outer_low = 10.0f;
float friction_speed_ki_outer_low = 0.0f;
float friction_speed_kd_outer_low = 0.0f;


const float friction_speed_Integral_Limit = 50.0f;
const float friction_speed_MaxOutput = 16380.0f;

static float last_fric_kp_inner = 0.0f;


//底盘跟随PID参数
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
void Shooter_Init(void);




//使能达妙电机
void DM_Motor_Enable(void);


//更新云台yaw轴角度
void Gimbal_Angle_Update(Motor_Controller *motor, float encoder_range);

//大疆电机控制指令
void DJI_Motor_Send_Command(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

//MIT模式控制指令
void DM4310_Send_MIT_Command(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, float pos_rad, float vel_rad_s, float kp, float kd, float torque);

//发送4310状态指令
void DM4310_Send_Mode_Command(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, uint8_t command);

//发送给底盘指令
void Send_Chassis_Command(FDCAN_HandleTypeDef* hfdcan, int16_t vx, int16_t vy, int16_t wz, uint8_t mode, uint8_t supercap_req, uint8_t climb_cmd, uint8_t led_cmd , uint8_t key_shift ,uint8_t friction_state, uint8_t is_base_locked);

//控制循环
void Robot_Control_Loop(const IMU_Data_t* imu_data);

//发射控制
void Shooter_Control_Loop(void);

//检测是否发射弹丸
void Shooter_Check_Real_Fire(void);

//热量模拟
void Shooter_Update_Heat(void);

//是否允许开火
uint8_t Shooter_Check_Heat_Allowed(void);


//CAN接收解码任务
void Can_Decode_Task(void const *argument);

//控制任务
void Gimbal_Control_task(void const * argument);

//重置弹道统计数据
void Bullet_Stats_Reset(void);

//更新云台Pitch电机
void update_gimbal_pitch(const uint8_t* data);

//更新云台Yaw电机
void update_gimbal_yaw(const uint8_t* data);

//更新拨盘电机
void update_dial_motor(const uint8_t* data);

//更新摩擦轮电机
void update_friction_motors(const uint8_t* data, uint32_t can_id);

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

#define DM_P_MIN -12.5f
#define DM_P_MAX 12.5f
#define DM_V_MIN -45.0f
#define DM_V_MAX 45.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

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
/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (span < 1e-6f) return 0;
    x = (x > x_max) ? x_max : ((x < x_min) ? x_min : x);
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
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
    Motor_Init(&dial_motor, DM_DIAL_SEND_ID, &hfdcan1);
	  PID_Init(&dial_motor.pid_angle, dial_angle_kp, dial_angle_ki, dial_angle_kd, dial_angle_Integral_Limit, dial_angle_MaxOutput);
    PID_Init(&dial_motor.pid_speed, dial_speed_kp, dial_speed_ki, dial_speed_kd, dial_speed_Integral_Limit, dial_speed_MaxOutput);

    
	  //摩擦轮
    Motor_Init(&friction_motor_inner_left,  FRICTION_INNER_LEFT_ID,  &hfdcan2);
    Motor_Init(&friction_motor_inner_right, FRICTION_INNER_RIGHT_ID, &hfdcan2);
    Motor_Init(&friction_motor_inner_up,    FRICTION_INNER_UP_ID,    &hfdcan2);
    
    Motor_Init(&friction_motor_outer_left,  FRICTION_OUTER_LEFT_ID,  &hfdcan2);
    Motor_Init(&friction_motor_outer_right, FRICTION_OUTER_RIGHT_ID, &hfdcan2);
    Motor_Init(&friction_motor_outer_up,    FRICTION_OUTER_UP_ID,    &hfdcan2);
		
		PID_Init(&friction_motor_inner_left.pid_speed,  friction_speed_kp_inner_high, friction_speed_ki_inner_high, friction_speed_kd_inner_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_inner_right.pid_speed, friction_speed_kp_inner_high, friction_speed_ki_inner_high, friction_speed_kd_inner_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_inner_up.pid_speed,    friction_speed_kp_inner_high, friction_speed_ki_inner_high, friction_speed_kd_inner_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    
    PID_Init(&friction_motor_outer_left.pid_speed,  friction_speed_kp_outer_high, friction_speed_ki_outer_high, friction_speed_kd_outer_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_outer_right.pid_speed, friction_speed_kp_outer_high, friction_speed_ki_outer_high, friction_speed_kd_outer_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_outer_up.pid_speed,    friction_speed_kp_outer_high, friction_speed_ki_outer_high, friction_speed_kd_outer_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
	
	  //Yaw
		SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
	  //Pitch
		SMC_Init(&gimbal_pitch_smc_imu, gimbal_pitch_c, gimbal_pitch_k_exp, gimbal_pitch_epsilon, gimbal_pitch_beta, gimbal_pitch_boundary, gimbal_pitch_out_max, 0.02f, 0.001f);		 

		//底盘跟随
	  PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, chassis_follow_Integral_Limit, chassis_follow_MaxOutput);
	
	  //初始化发射
    Shooter_Init();
}




//初始化发射
void Shooter_Init(void)
{
    memset(&shooter_controller, 0, sizeof(Shooter_Controller_t));
    shooter_controller.state = SHOOTER_OFF;
    shooter_controller.is_calibrated = 0;
}



//更新云台yaw轴角度
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








/**
  * @brief  大疆电机控制指令发送
  * @param  hfdcan: FDCAN句柄指针
  * @param  std_id: 目标控制CAN ID
  * @param  current: 对应4个电机的目标电流
  */
//大疆电机控制指令
void DJI_Motor_Send_Command(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t tx_data[8];

    TxHeader.Identifier = std_id; 
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    tx_data[0] = (uint8_t)(current1 >> 8);
    tx_data[1] = (uint8_t)(current1 & 0xFF);
    tx_data[2] = (uint8_t)(current2 >> 8);
    tx_data[3] = (uint8_t)(current2 & 0xFF);
    tx_data[4] = (uint8_t)(current3 >> 8);
    tx_data[5] = (uint8_t)(current3 & 0xFF);
    tx_data[6] = (uint8_t)(current4 >> 8);
    tx_data[7] = (uint8_t)(current4 & 0xFF);

    //直接写入硬件 FIFO (检查是否有空位)
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
		{
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data);
    }
}




//发送LK电机开启命令
void LK_Motor_Enable(FDCAN_HandleTypeDef* hfdcan)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0x88;//电机运行
    //其他字节全是0x00

    TxHeader.Identifier = LK_YAW_ID; 
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    //发送
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data);
    }
}






/**
  * @brief  瓴控电机转矩控制发送
  * @param  hfdcan: FDCAN句柄指针
  * @param  id: 电机实际ID (1, 2)
  * @param  current_amps: 目标电流值 (安培)
  */
void LK_Send_Torque(FDCAN_HandleTypeDef* hfdcan, float current_amps)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t tx_data[8];
    
    //限制电流幅度
    if(current_amps > 16.5f) current_amps = 16.5f;
    if(current_amps < -16.5f) current_amps = -16.5f;

    //转换为 -2048 ~ 2048 的整数
    int16_t send_current = (int16_t)(current_amps * LK_CURRENT_TO_RAW);

    TxHeader.Identifier = LK_YAW_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;     //标准帧
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; //数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; //数据长度 8字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;  //关闭波特率切换 (Classic CAN)
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;   //经典 CAN 格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; //不存入事件队列
    TxHeader.MessageMarker = 0;

    tx_data[0] = 0xA1; //转矩控制命令
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = (uint8_t)(send_current & 0xFF);
    tx_data[5] = (uint8_t)((send_current >> 8) & 0xFF);
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;

    //直接写入硬件 FIFO (检查是否有空位)
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data);
    }
}




void LK_Send_Query(FDCAN_HandleTypeDef* hfdcan)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t tx_data[8];
    
    TxHeader.Identifier = LK_YAW_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;     //标准帧
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; //数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; //数据长度 8字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;  //关闭波特率切换 (Classic CAN)
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;   //经典 CAN 格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; //不存入事件队列
    TxHeader.MessageMarker = 0;

    tx_data[0] = 0x9C;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;

    //直接写入硬件 FIFO (检查是否有空位)
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data);
    }
}



void DM_Motor_Send_MIT_Command(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos_rad, float vel_rad_s, float kp, float kd, float torque_nm, float t_max)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    uint16_t pos_uint = float_to_uint(pos_rad, DM_P_MIN, DM_P_MAX, 16);
    uint16_t vel_uint = float_to_uint(vel_rad_s, DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp_uint  = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_uint  = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t tor_uint = float_to_uint(torque_nm, -t_max, t_max, 12);

    tx_data[0] = pos_uint >> 8;
    tx_data[1] = pos_uint & 0xFF;
    tx_data[2] = vel_uint >> 4;
    tx_data[3] = ((vel_uint & 0x0F) << 4) | (kp_uint >> 8);
    tx_data[4] = kp_uint & 0xFF;
    tx_data[5] = kd_uint >> 4;
    tx_data[6] = ((kd_uint & 0x0F) << 4) | (tor_uint >> 8);
    tx_data[7] = tor_uint & 0xFF;

    tx_header.Identifier = motor_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    //直接写入硬件 FIFO (检查是否有空位)
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}




///**
//  * @brief  达妙速度控制指令
//  * @param  hcan        CAN句柄
//  * @param  motor_id    电机ID
//  * @param  target_vel  目标速度 (rad/s, 32位浮点数)
//  * @retval None
//  */void DM_Motor_Send_Speed_Command(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float target_vel)
//{
//    FDCAN_TxHeaderTypeDef tx_header;
//    uint8_t tx_data[4];

//    uint8_t *p_vel = (uint8_t *)&target_vel;
//    tx_data[0] = p_vel[0];
//    tx_data[1] = p_vel[1];
//    tx_data[2] = p_vel[2];
//    tx_data[3] = p_vel[3];

//    tx_header.Identifier = motor_id; 
//    tx_header.IdType = FDCAN_STANDARD_ID;
//    tx_header.TxFrameType = FDCAN_DATA_FRAME;
//    tx_header.DataLength = FDCAN_DLC_BYTES_4;
//    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
//    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
//    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    tx_header.MessageMarker = 0;

//    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
//    {
//        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
//    }
//}



#define TX_DIAL_CMD_ID    0x303
#define RX_DIAL_FDB_ID    0x302
//将达妙速度控制指令发送给底盘
void Dial_Send_Speed(FDCAN_HandleTypeDef *hfdcan, float target_vel)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[4];

    uint8_t *p_vel = (uint8_t *)&target_vel;
    tx_data[0] = p_vel[0];
    tx_data[1] = p_vel[1];
    tx_data[2] = p_vel[2];
    tx_data[3] = p_vel[3];

    tx_header.Identifier = TX_DIAL_CMD_ID; 
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_4;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}


#define TX_DIAL_MODE_ID   0x301
void Dial_Send_Mode_Command(FDCAN_HandleTypeDef *hfdcan, uint8_t command)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};

    tx_header.Identifier = TX_DIAL_MODE_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
		{
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}




//云台发底盘数据包
typedef union
{
    uint8_t data[8];
    
    __packed struct
    {
        int64_t  vx : 16;             //(-16384 ~ 16383)
        int64_t  vy : 16; 
        int64_t  wz : 16; 
        
        uint64_t mode           : 1;  //底盘状态(0=失能, 1=使能)
        uint64_t supercap_req   : 1;  //超电(0=关, 1=开)
        uint64_t climb_cmd      : 2;  //上坎指令(0~3)(0=全收, 1=前伸, 2=后伸)
        uint64_t led_cmd        : 1;  //LED指令
        uint64_t key_shift      : 1;  //Shift键状态(0=松开, 1=按下)
        uint64_t friction_state : 2;  // 0: 关, 1: 低速 (12m/s), 2: 高速 (16m/s)
			  uint64_t is_base_locked : 1;  //基地已锁定
			
			  uint64_t battle_mode    : 2;  //键鼠模式(0: 失能, 1:跟随, 2: 底盘锁 3: 吊射)
        
        uint64_t reserved       : 5; //剩5位
    } pack;
} Gimbal_To_Chassis_Msg_t;

#define LIMIT_15BIT(val)  ((val) > 16383 ? 16383 : ((val) < -16384 ? -16384 : (val)))
#define LIMIT_16BIT(val)  ((val) > 32767 ? 32767 : ((val) < -32768 ? -32768 : (val)))
void Send_Chassis_Command(FDCAN_HandleTypeDef* hfdcan, int16_t vx, int16_t vy, int16_t wz, uint8_t mode, uint8_t supercap_req, uint8_t climb_cmd, uint8_t led_cmd , uint8_t key_shift ,uint8_t friction_state, uint8_t is_base_locked)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    Gimbal_To_Chassis_Msg_t tx_msg = {0};

    TxHeader.Identifier = TX_CHASSIS_COMMAND_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;         //标准帧
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;     //数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;     //长度8字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;      //关闭波特率切换(Classic CAN)
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;       //经典 CAN 格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; //不存入事件队列
    TxHeader.MessageMarker = 0;

		
    tx_msg.pack.vx = LIMIT_16BIT(vx);
    tx_msg.pack.vy = LIMIT_16BIT(vy);
    tx_msg.pack.wz = LIMIT_16BIT(wz);
    
    tx_msg.pack.mode           = mode & 0x01;
    tx_msg.pack.supercap_req   = supercap_req & 0x01;
    tx_msg.pack.climb_cmd      = climb_cmd & 0x03;
    tx_msg.pack.led_cmd        = led_cmd & 0x01;
    tx_msg.pack.key_shift      = key_shift & 0x01;
    tx_msg.pack.friction_state = friction_state & 0x03;
		tx_msg.pack.is_base_locked = is_base_locked & 0x01;
		tx_msg.pack.battle_mode    = battle_mode & 0x03;

    //直接写入硬件发送 FIFO(检查是否有空位)
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_msg.data);
    }
}






typedef union
{
    uint8_t data[8]; 
    
    __packed struct
    {
        uint64_t yaw_angle_x100   : 16; //Yawn轴机械角度
        int64_t  pitch_angle_x100 : 16; //Pitch俯仰机械角度
			
			  uint64_t friction_speed   : 13; //摩擦轮目标转速
			
        uint64_t reserved        : 19; //剩19位
    } pack;
} Gimbal_UI_Msg_t;

//发送UI数据到底盘
void Send_Chassis_UI_Data(FDCAN_HandleTypeDef* hfdcan, float yaw_deg, float pitch_deg)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    Gimbal_UI_Msg_t ui_msg = {0}; 

    TxHeader.Identifier = TX_CHASSIS_UI_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;         //标准帧
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;     //数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;     //长度8字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;      //关闭波特率切换(Classic CAN)
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;       //经典 CAN 格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; //不存入事件队列
    TxHeader.MessageMarker = 0;

    int32_t yaw_deg_x100 = (int16_t)(yaw_deg * 100.0f);
    int32_t pit_deg_x100 = (int16_t)(pitch_deg * 100.0f);
		LIMIT_MIN_MAX(yaw_deg_x100, 0, 35999);
		LIMIT_MIN_MAX(pit_deg_x100, -18000, 18000);
    ui_msg.pack.yaw_angle_x100   = (uint16_t)yaw_deg_x100;
    ui_msg.pack.pitch_angle_x100 = pit_deg_x100;
		
		
		ui_msg.pack.friction_speed = (shooter_controller.friction_target_speed > 8191) ? 8191 : shooter_controller.friction_target_speed;

    //直接写入硬件发送 FIFO(检查是否有空位)
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, ui_msg.data);
    }
}





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
							  Bullet_Stats_Reset();
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
		

	  if(robot_mode != ROBOT_MODE_OFF && dial_motor.state != 1 && shooter_controller.state != SHOOTER_OFF) 
    {
        Dial_Send_Mode_Command(&hfdcan1, 0xFC);
    }
		
		
		
		//重新使能
		if(robot_mode != ROBOT_MODE_OFF && last_robot_mode == ROBOT_MODE_OFF)
    {		
		    SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
				SMC_Init(&gimbal_pitch_smc_imu, gimbal_pitch_c, gimbal_pitch_k_exp, gimbal_pitch_epsilon, gimbal_pitch_beta, gimbal_pitch_boundary, gimbal_pitch_out_max, 0.02f, 0.001f);		 

				PID_Init(&dial_motor.pid_angle, dial_angle_kp, dial_angle_ki, dial_angle_kd, dial_angle_Integral_Limit, dial_angle_MaxOutput);
				PID_Init(&dial_motor.pid_speed, dial_speed_kp, dial_speed_ki, dial_speed_kd, dial_speed_Integral_Limit, dial_speed_MaxOutput);
			
        last_fric_kp_inner = 0.0f;
			
			  PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, chassis_follow_Integral_Limit, chassis_follow_MaxOutput);
			
        rc_target_world_yaw_angle = imu_data->yaw_angle;
        rc_target_world_pitch_angle = imu_data->pitch_angle;
			
		  	Dial_Send_Mode_Command(&hfdcan1, 0xFC);
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
							
								PID_Init(&dial_motor.pid_angle, dial_angle_kp, dial_angle_ki, dial_angle_kd, 0, 0);
								PID_Init(&dial_motor.pid_speed, dial_speed_kp, dial_speed_ki, dial_speed_kd, 0, 0);			
								
							  PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, 0, 0);
							
								gimbal_yaw_motor.target_current = 0;
								gimbal_pitch_motor.target_current = 0;
							  dial_motor.target_torque = 0.0f;
							
                LK_Send_Torque(&hfdcan1, 0);
								DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x1FF, 0, 0, 0, 0);
								DJI_Motor_Send_Command(&hfdcan2, 0x200, 0, 0, 0, 0);
								
								Dial_Send_Speed(&hfdcan1, 0);
								Dial_Send_Mode_Command(&hfdcan1, 0xFD);
							
								send_vx = 0; send_vy = 0; send_wz = 0;
                send_mode = 0;  supercap_cmd = 0;

							  shooter_controller.state = SHOOTER_OFF;
								fric_level = 0;
                Shooter_Control_Loop(); 
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
						if(rc->key.map.G && rc->key.map.W && !key_W_last_state) high_speed += offset_step;
						if(rc->key.map.G && rc->key.map.S && !key_S_last_state) high_speed -= offset_step;

						if(rc->key.map.F && rc->key.map.W && !key_W_last_state) low_speed += offset_step;
						if(rc->key.map.F && rc->key.map.S && !key_S_last_state) low_speed -= offset_step;
						
						
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
						
						shooter_controller.state = SHOOTER_OFF;
					
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
					     	shooter_controller.state = SHOOTER_OFF;
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

            shooter_controller.state = SHOOTER_OFF;
					
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

            shooter_controller.state = SHOOTER_OFF;
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
		
        Shooter_Control_Loop();
		
	    	LK_Send_Torque(&hfdcan1, gimbal_yaw_current);
		
		    DJI_Motor_Send_Command(&hfdcan3, 0x1FF, gimbal_pitch_current, 0, 0, 0);


        DJI_Motor_Send_Command(&hfdcan2, 0x200, friction_motor_outer_left.target_current, friction_motor_outer_right.target_current, friction_motor_outer_up.target_current, 0);
        DJI_Motor_Send_Command(&hfdcan2, 0x1FF, friction_motor_inner_left.target_current, friction_motor_inner_right.target_current, friction_motor_inner_up.target_current ,0);
		
        Send_Chassis_Command(&hfdcan1, send_vx, send_vy, send_wz, send_mode, supercap_cmd, climb_cmd, led_cmd, key_shift, shooter_controller.friction_state ,is_base_locked);

		    Dial_Send_Speed(&hfdcan1, dial_motor.target_speed);
		
				static uint32_t ui_send_cnt = 0;
				if (++ui_send_cnt >= 50) 
				{
						ui_send_cnt = 0;
						
						float yaw_deg = gimbal_yaw_motor.mechanical_degree; 
						float pit_deg = PITCH_HORIZON_DEGREE - gimbal_pitch_motor.mechanical_degree;

						Send_Chassis_UI_Data(&hfdcan1, yaw_deg, pit_deg);
				}
        last_robot_mode = robot_mode;
}











//检查摩擦轮是否全部就绪
static uint8_t Check_Friction_Ready(void)
{
	  //是否在线
    if (!friction_motor_inner_left.is_online  ||
        !friction_motor_inner_right.is_online ||
        !friction_motor_inner_up.is_online    ||
        !friction_motor_outer_left.is_online  ||
        !friction_motor_outer_right.is_online ||
        !friction_motor_outer_up.is_online)
    {
        return 0;
    }
		
		
		//是否在转
    if (ABS(friction_motor_inner_left.feedback.actual_speed)  < 250.0f ||
        ABS(friction_motor_inner_right.feedback.actual_speed) < 250.0f ||
        ABS(friction_motor_inner_up.feedback.actual_speed)    < 250.0f ||
        ABS(friction_motor_outer_left.feedback.actual_speed)  < 800.0f ||
        ABS(friction_motor_outer_right.feedback.actual_speed) < 800.0f ||
        ABS(friction_motor_outer_up.feedback.actual_speed)    < 800.0f)
    {
        return 0;
    }
    return 1;
}




//弹速自适应控制参数
#define SPEED_TARGET_HIGH      16.3f  //弹速上限(超过此值减速)
#define SPEED_TARGET_LOW       16.1f  //弹速下限(低于此值加速)
#define RPM_ADJUST_STEP        5.0f  //摩擦轮每次微调的步进(RPM)  //5
#define MAX_AUTO_OFFSET        0.0f //最大允许补偿值          //25

#define MAX_WINDOW_SIZE        80


//波形
float print_initial_speed = 0.0f;       //要打印的弹速
static uint16_t vofa_timer = 0;      //持续计时器
float bullet_speed_avg = 0.0f;       //平均弹速
float bullet_speed_range = 0.0f;     //极差
float bullet_speed_variance = 0.0f;  //方差
uint32_t total_shot_count = 0;       //发射数


float auto_speed_offset = 0.0f;     //最终摩擦轮补偿值

float speed_window[MAX_WINDOW_SIZE] = {0};


uint8_t high_speed_count = 0;    //连续超速计数器
uint8_t low_speed_count = 0;     //连续低速计数器
float last_referee_speed = 0.0f; //上一次的弹速
static uint32_t shoot_seq_cnt = 0; //记录一轮射击打到了第几发
float pending_speed = 0.0f; //暂存上一发的弹速



//统计弹速数据
static void process_new_bullet_speed(float new_speed)
{
    total_shot_count++;

    uint8_t current_index = (total_shot_count - 1) % MAX_WINDOW_SIZE; 
    speed_window[current_index] = new_speed;

    uint8_t valid_count = (total_shot_count < MAX_WINDOW_SIZE) ? total_shot_count : MAX_WINDOW_SIZE;
    if (valid_count == 0) return;

    //计算 平均值, 最大值, 最小值
    float sum = 0.0f;
    float max_speed = speed_window[0];
    float min_speed = speed_window[0];

    for(uint8_t i = 0; i < valid_count; i++)
  	{
        sum += speed_window[i];
        if (speed_window[i] > max_speed) max_speed = speed_window[i];
        if (speed_window[i] < min_speed) min_speed = speed_window[i];
    }
    
    bullet_speed_avg = sum / valid_count;
    bullet_speed_range = max_speed - min_speed;
    
    //计算方差
    float variance_sum = 0.0f;
    for (uint8_t i = 0; i < valid_count; i++)
		{
        variance_sum += (speed_window[i] - bullet_speed_avg) * (speed_window[i] - bullet_speed_avg);
    }
    bullet_speed_variance = variance_sum / valid_count;
}




/**
  * @brief  重置弹道统计数据和自适应补偿
  */
void Bullet_Stats_Reset(void)
{
    //清空滑动窗口数组
    memset(speed_window, 0, sizeof(speed_window));
    
    //重置计数器和索引
    total_shot_count = 0;
    shoot_seq_cnt = 0;
    
    //清空统计结果
    bullet_speed_avg = 0.0f;
    bullet_speed_range = 0.0f;
    bullet_speed_variance = 0.0f;
    
    high_speed_count = 0;
    low_speed_count = 0;
}




//弹速自调整
void Bullet_Speed_Adaptive_Control(float latest_speed)
{
    //连续两发高,减速
    if(latest_speed > SPEED_TARGET_HIGH)
    {
        high_speed_count++;
        low_speed_count = 0; 
        
        if(high_speed_count >= 2) 
        {
            auto_speed_offset -= RPM_ADJUST_STEP;
            high_speed_count = 0;
        }
    }
    //连续两发低,加速
    else if (latest_speed < SPEED_TARGET_LOW)
    {
        low_speed_count++;
        high_speed_count = 0; 
        
        if(low_speed_count >= 2) 
        {
            auto_speed_offset += RPM_ADJUST_STEP;
            low_speed_count = 0;
        }
    }
    //状态良好
    else
    {
        high_speed_count = 0;
        low_speed_count = 0;
    }
    //限幅
    if(auto_speed_offset > MAX_AUTO_OFFSET)  auto_speed_offset = MAX_AUTO_OFFSET;
    if(auto_speed_offset < -MAX_AUTO_OFFSET) auto_speed_offset = -MAX_AUTO_OFFSET;
}





uint32_t last_bullet_update_tick = 0;
void Bullet_Speed_Adaptive_Process(void)
{
    if(initial_speed != last_referee_speed && initial_speed > 11.0f && initial_speed < 17.0f)
    {
			 shoot_seq_cnt++;
			
			 last_bullet_update_tick = osKernelGetTickCount(); 
			
        if(shoot_seq_cnt > 2) 
        {
            if(robot_mode != ROBOT_MODE_OFF)
            {
							  //更新统计数据
                process_new_bullet_speed(pending_speed);
							  //弹速自调整
                Bullet_Speed_Adaptive_Control(pending_speed);
							
                print_initial_speed = initial_speed; 
                vofa_timer = 500;
            }
        }
				pending_speed = initial_speed;
				
        last_referee_speed = initial_speed;
    }
    
    if(robot_mode == ROBOT_MODE_OFF)
    {
        auto_speed_offset = 0.0f;
        high_speed_count = 0;
        low_speed_count = 0;
			
			  shoot_seq_cnt = 0; 
    }
}




/**
  * @brief  拨盘电机重新上电处理
  */
static void Dial_Recover_Process(void)
{
    static uint32_t dial_power_on_time = 0;
    static uint8_t dial_recover_step = 0;
    static bool dial_was_offline = false;

    //拨盘离线
    if(!dial_motor.is_online)
    {
        dial_was_offline = true;
        dial_recover_step = 0;
        
        shooter_controller.state = SHOOTER_OFF;
        shooter_controller.single_shot_fired_flag = 0;
        shooter_controller.continuous_fire_timer = 0;
        shooter_controller.jam_detect_timer = 0;
        shooter_controller.jam_timeout_timer = 0;
        return;
    }
    //拨盘重新上电
    if(dial_motor.is_online && dial_was_offline)
    {
        if(dial_recover_step == 0)
        {
            dial_power_on_time = xTaskGetTickCount(); //记录重新上线的时间戳
            shooter_controller.state = SHOOTER_OFF;
            dial_recover_step = 1;
        }
        else if(dial_recover_step == 1)
        {
            shooter_controller.state = SHOOTER_OFF;

            //等200ms防欠压
            if(xTaskGetTickCount() - dial_power_on_time > 200)
            {
                Dial_Send_Mode_Command(&hfdcan1, 0xFC);
                
                dial_was_offline = false;
                dial_recover_step = 0;
            }
        }
    }
}



#define USE_AUTO_CALIBRATION 1 //1:开启撞限位校准, 0:关闭校准(当前位置为零点)

//发射控制
void Shooter_Control_Loop(void)
{
	  //获取时间
    uint32_t now = xTaskGetTickCount();
	
	  //更新热量
    Shooter_Update_Heat();
	
	  //检测是否发弹
    Shooter_Check_Real_Fire();
	
	  //拨盘电机重新上电处理
	  Dial_Recover_Process();
	
	  //左键边缘检测
	  static bool mouse_l_last_state = false;
    bool mouse_l_just_pressed = (rc->mouse.press_l && !mouse_l_last_state);
    mouse_l_last_state = rc->mouse.press_l;

    //中键边缘检测
    static bool mouse_m_last_state = false;
    bool mouse_m_just_pressed = (rc->mouse.press_middle && !mouse_m_last_state);
    mouse_m_last_state = rc->mouse.press_middle;

    
		//发射指令
    uint8_t fire_command = 0;

    //遥控拨轮开火
    if(rc->rc.wheel > 250 && robot_mode != ROBOT_MODE_AIMING) 
    {
        fire_command = 1;
    }
		
		//自瞄自动开火
    if(robot_mode == ROBOT_MODE_AIMING && 
						 Vision_IsOnline() && 
						 VisionData.is_tracking == 1 && 
						 VisionData.cmd_mode == 2)
    {
        fire_command = 1;
    }

    //鼠标中键开火
    if(mouse_m_just_pressed) 
    {
        fire_command = 1;
    }
		
    //鼠标左键开火
    if(is_aiming_now)//锁敌自动开火
    {
        if(rc->mouse.press_l && VisionData.cmd_mode == 2)
				{
            fire_command = 1;
        }
    }
    else if(mouse_l_just_pressed) 
    {
         fire_command = 1;
    }
		


    //切换12m/s
    static bool f_key_last = false;
    if(rc->key.map.F && !f_key_last)
		{
        fric_level = 1;
    }
    f_key_last = rc->key.map.F;

    //切换16m/s
    static bool g_key_last = false;
    if (rc->key.map.G && !g_key_last) {
        fric_level = 2;
    }
    g_key_last = rc->key.map.G;
		

		
	  //摩擦轮目标转速和线速度比率
	  float friction_target_speed = 0.0f;
    float friction_ratio = 0.85f;
		float friction_offs = 0.0f;
		float fric_left_kp_offs = -0.5f;//0.5
		float fric_right_kp_offs = 0.1f;//0.1
		
    float cur_fric_kp_inner = friction_speed_kp_inner_high;
    float cur_fric_ki_inner = friction_speed_ki_inner_high;
    float cur_fric_kd_inner = friction_speed_kd_inner_high;
    
    float cur_fric_kp_outer = friction_speed_kp_outer_high;
    float cur_fric_ki_outer = friction_speed_ki_outer_high;
    float cur_fric_kd_outer = friction_speed_kd_outer_high;
		
    if(control_source == CONTROL_SOURCE_VTM) 
    {
        if(fric_level == 1)
				{
            friction_target_speed = FRICTION_TARGET_SPEED_LOW;
            friction_ratio = FRICTION_RATIO_LOW;
					  shooter_controller.friction_state = 1;
					  friction_offs = 0.0f;
					  fric_left_kp_offs = 0.0f;
					  fric_right_kp_offs = 0.0f;
					
						cur_fric_kp_inner = friction_speed_kp_inner_low;
						cur_fric_ki_inner = friction_speed_ki_inner_low;
						cur_fric_kd_inner = friction_speed_kd_inner_low;
						
						cur_fric_kp_outer = friction_speed_kp_outer_low;
						cur_fric_ki_outer = friction_speed_ki_outer_low;
						cur_fric_kd_outer = friction_speed_kd_outer_low;
        }
				else if(fric_level == 2)
				{
            friction_target_speed = FRICTION_TARGET_SPEED_HIGH;
            friction_ratio = FRICTION_RATIO_HIGH;
					  shooter_controller.friction_state = 2;
					  friction_offs = 0.0f;
					  fric_left_kp_offs = -0.4f;
					  fric_right_kp_offs = 0.1f;
					
					  cur_fric_kp_inner = friction_speed_kp_inner_high;
            cur_fric_ki_inner = friction_speed_ki_inner_high;
            cur_fric_kd_inner = friction_speed_kd_inner_high;
            
            cur_fric_kp_outer = friction_speed_kp_outer_high;
            cur_fric_ki_outer = friction_speed_ki_outer_high;
            cur_fric_kd_outer = friction_speed_kd_outer_high;
        }
    } 
    else 
    {
					if(robot_mode == ROBOT_MODE_SHOOT || (robot_mode == ROBOT_MODE_BATTLE && gimbal_mode == GIMBAL_MODE_ENCODER))
					{
							friction_target_speed = FRICTION_TARGET_SPEED_HIGH;
							friction_ratio = FRICTION_RATIO_HIGH;
					  	shooter_controller.friction_state = 1;
						  friction_offs = 0.0f;
						  fric_left_kp_offs = -0.4f;
						  fric_right_kp_offs = 0.1f;
						
							cur_fric_kp_inner = friction_speed_kp_inner_high;
							cur_fric_ki_inner = friction_speed_ki_inner_high;
							cur_fric_kd_inner = friction_speed_kd_inner_high;
							
							cur_fric_kp_outer = friction_speed_kp_outer_high;
							cur_fric_ki_outer = friction_speed_ki_outer_high;
							cur_fric_kd_outer = friction_speed_kd_outer_high;
					}
					else if (robot_mode == ROBOT_MODE_SHOOT_LOW || robot_mode == ROBOT_MODE_AIMING || robot_mode == ROBOT_MODE_M_AIMING || (robot_mode == ROBOT_MODE_BATTLE && gimbal_mode == GIMBAL_MODE_IMU))
					{
							friction_target_speed = FRICTION_TARGET_SPEED_LOW;
							friction_ratio = FRICTION_RATIO_LOW;
						  shooter_controller.friction_state = 2;
					  	friction_offs = 0.0f;
						  fric_left_kp_offs = 0.0f;
						  fric_right_kp_offs = 0.0f;
						
							cur_fric_kp_inner = friction_speed_kp_inner_low;
							cur_fric_ki_inner = friction_speed_ki_inner_low;
							cur_fric_kd_inner = friction_speed_kd_inner_low;
							
							cur_fric_kp_outer = friction_speed_kp_outer_low;
							cur_fric_ki_outer = friction_speed_ki_outer_low;
							cur_fric_kd_outer = friction_speed_kd_outer_low;
					}
    }
		
    
    if(cur_fric_kp_inner != last_fric_kp_inner)
    {
			  //一级
        friction_motor_inner_left.pid_speed.Kp = cur_fric_kp_inner + fric_left_kp_offs;
        friction_motor_inner_left.pid_speed.Ki = cur_fric_ki_inner;
        friction_motor_inner_left.pid_speed.Kd = cur_fric_kd_inner;
        friction_motor_inner_right.pid_speed.Kp = cur_fric_kp_inner + fric_right_kp_offs;
        friction_motor_inner_right.pid_speed.Ki = cur_fric_ki_inner;
        friction_motor_inner_right.pid_speed.Kd = cur_fric_kd_inner;
        friction_motor_inner_up.pid_speed.Kp = cur_fric_kp_inner;
        friction_motor_inner_up.pid_speed.Ki = cur_fric_ki_inner;
        friction_motor_inner_up.pid_speed.Kd = cur_fric_kd_inner;

			  //二级
        friction_motor_outer_left.pid_speed.Kp = cur_fric_kp_outer;
        friction_motor_outer_left.pid_speed.Ki = cur_fric_ki_outer;
        friction_motor_outer_left.pid_speed.Kd = cur_fric_kd_outer;
        friction_motor_outer_right.pid_speed.Kp = cur_fric_kp_outer;
        friction_motor_outer_right.pid_speed.Ki = cur_fric_ki_outer;
        friction_motor_outer_right.pid_speed.Kd = cur_fric_kd_outer;
        friction_motor_outer_up.pid_speed.Kp = cur_fric_kp_outer;
        friction_motor_outer_up.pid_speed.Ki = cur_fric_ki_outer;
        friction_motor_outer_up.pid_speed.Kd = cur_fric_kd_outer;

        friction_motor_inner_left.pid_speed.i_out = 0.0f;
        friction_motor_inner_right.pid_speed.i_out = 0.0f;
        friction_motor_inner_up.pid_speed.i_out = 0.0f;
        friction_motor_outer_left.pid_speed.i_out = 0.0f;
        friction_motor_outer_right.pid_speed.i_out = 0.0f;
        friction_motor_outer_up.pid_speed.i_out = 0.0f;

        last_fric_kp_inner = cur_fric_kp_inner;
    }
		
		
	  //卡弹检测
    if(shooter_controller.state != SHOOTER_OFF && shooter_controller.state != SHOOTER_JAMMED && shooter_controller.state != SHOOTER_CALIBRATING)
    {
        if(ABS(dial_motor.feedback.torque) > DIAL_JAM_CURRENT_THRESHOLD&& 
           ABS(dial_motor.feedback.velocity) < DIAL_JAM_SPEED_THRESHOLD) 
        {
            shooter_controller.jam_detect_timer++;
        }
        else 
        {
            if(shooter_controller.jam_detect_timer > 10)
            {
                shooter_controller.jam_detect_timer -= 10; 
            }
            else
            {
                shooter_controller.jam_detect_timer = 0;
            }
        }
        
				//卡弹
        if(shooter_controller.jam_detect_timer > DIAL_JAM_DETECT_THRESHOLD)
        {
            shooter_controller.dial_target_rad  = dial_motor.current_total_angle_rad - DIAL_ONE_SHOOT_ENCODER * 0.75f;
            shooter_controller.jam_detect_timer = 0;
				  	shooter_controller.jam_timeout_timer = 0;
					  shooter_controller.jam_process_step = 0;  //0：正在后退
					  shooter_controller.state = SHOOTER_JAMMED;
        }
    }


    //校准拨盘
		if(shooter_controller.state != SHOOTER_JAMMED)
    {
        if((rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_MID) || (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_UP) || (rc->rc.s_left == DR16_SW_MID && rc->rc.s_right == DR16_SW_UP) || (robot_mode == ROBOT_MODE_BATTLE && (fric_level == 1 || fric_level == 2 )) || (rc->rc.s_left == DR16_SW_UP && rc->rc.s_right == DR16_SW_DOWN)) 
        {
					
#if USE_AUTO_CALIBRATION 
            //如果还没校准过
            if(shooter_controller.is_calibrated == 0)
            {
                //进入校准状态
                if(shooter_controller.state != SHOOTER_CALIBRATING)
                {
                    shooter_controller.state = SHOOTER_CALIBRATING;
                    shooter_controller.homing_step = 0;
                    shooter_controller.homing_timer = 0;
                }
						}
						else
#endif			
            //已校准或不校准
            if(shooter_controller.state == SHOOTER_OFF)
            {
								//防跳变
								float dial_error = dial_motor.current_total_angle_rad - shooter_controller.dial_last_normal_rad;
								
								while(dial_error > DIAL_ONE_SHOOT_ENCODER - 0.2f) 
								{
										shooter_controller.dial_last_normal_rad += DIAL_ONE_SHOOT_ENCODER;
										dial_error -= DIAL_ONE_SHOOT_ENCODER;
								}
								while (dial_error < -DIAL_ONE_SHOOT_ENCODER - 0.2f) 
								{
										shooter_controller.dial_last_normal_rad -= DIAL_ONE_SHOOT_ENCODER;
										dial_error += DIAL_ONE_SHOOT_ENCODER;
								}
								
							  shooter_controller.dial_target_rad = shooter_controller.dial_last_normal_rad;
                shooter_controller.state = SHOOTER_READY;
            }
        }
        else
        {
            shooter_controller.state = SHOOTER_OFF;
        }
    }
		
		
    switch(shooter_controller.state)
    {
        case SHOOTER_OFF:
        {
            shooter_controller.friction_target_speed = 0.0f;
            shooter_controller.dial_target_rad = dial_motor.current_total_angle_rad;
            shooter_controller.single_shot_fired_flag = 0;
            shooter_controller.jam_detect_timer = 0;
            break;
        }
				
        case SHOOTER_CALIBRATING:
        {
            shooter_controller.friction_target_speed = 0.0f;

            //0:反转找限位
            if (shooter_controller.homing_step == 0)
            {
                shooter_controller.homing_timer++;
                //延时检测，避开启动电流
                if(shooter_controller.homing_timer > DIAL_HOMING_CHECK_DELAY)
                {
                    //撞到限位
                    if(ABS(dial_motor.feedback.torque) > DIAL_HOMING_CURRENT_THRESHOLD &&
                        ABS(dial_motor.feedback.velocity) < DIAL_HOMING_SPEED_THRESHOLD)
                    {
                        shooter_controller.dial_target_rad = dial_motor.current_total_angle_rad + DIAL_HOMING_OFFSET;
                        shooter_controller.homing_step = 1;
                        shooter_controller.homing_timer = 0;
                    }
                }
                //超时保护
                if(shooter_controller.homing_timer > 4000) 
                {
                     shooter_controller.state = SHOOTER_OFF;
                }
            }
            else if (shooter_controller.homing_step == 1)
            {
							  //校准完成
                if(fabs(shooter_controller.dial_target_rad - dial_motor.current_total_angle_rad) < 0.1f)
                {
                    shooter_controller.is_calibrated = 1; 
                    shooter_controller.dial_last_normal_rad = shooter_controller.dial_target_rad;
                    shooter_controller.state = SHOOTER_READY;
                }
            }
            break;
        }

        case SHOOTER_READY:
        {
            shooter_controller.friction_target_speed = friction_target_speed;
            shooter_controller.friction_linear_ratio = friction_ratio;
            shooter_controller.jam_detect_timer = 0;
					
					
					
				  	if(robot_mode == ROBOT_MODE_AIMING)
						{
							  gap = 5000;
						}
            else if (robot_mode == ROBOT_MODE_BATTLE && is_aiming_now == 1)
            {
                gap = 2100;
            }
						else
						{
							  gap = 200;
						}
					
            uint8_t is_cooldown_ok = (now - last_shoot_time > gap);
            					
					
					  if(Check_Friction_Ready() == 1)
            {
								if(fire_command && Shooter_Check_Heat_Allowed() && is_cooldown_ok && shooter_controller.single_shot_fired_flag == 0)
								{
										//直接设目标打第一发
										shooter_controller.dial_target_rad = shooter_controller.dial_last_normal_rad + DIAL_ONE_SHOOT_ENCODER;
										
										//先扣热量，重置检测
										shooter_predict_heat += HERO_HEAT_PER_SHOT;
										current_shot_status = SHOT_NOT_DETECTED;
									
								  	shooter_controller.single_shot_fired_flag = 1;
									
										shooter_controller.state = SHOOTER_SINGLE_SHOT;
									
									
									  //记录发射时间
                    last_shoot_time = now;
								}
								else if((robot_mode == ROBOT_MODE_AIMING && !fire_command) || (robot_mode != ROBOT_MODE_AIMING && ABS(rc->rc.wheel) < 100))
								{
										shooter_controller.single_shot_fired_flag = 0;//准备下次发射
								}
				  	}
            break;
        }

        case SHOOTER_SINGLE_SHOT:
        {
            shooter_controller.friction_target_speed = friction_target_speed;
            shooter_controller.friction_linear_ratio = friction_ratio;
					
            //任务完成判断
            if(fabs(shooter_controller.dial_target_rad - dial_motor.current_total_angle_rad) < 0.2f)
            {
                if(shooter_controller.dial_target_rad > shooter_controller.dial_last_normal_rad)
                {
                    shooter_controller.dial_last_normal_rad = shooter_controller.dial_target_rad;
                }
								
							  //如果空枪，退热量
                if(current_shot_status == SHOT_NOT_DETECTED)
								{
                    shooter_predict_heat -= HERO_HEAT_PER_SHOT;
                    if(shooter_predict_heat < 0) shooter_predict_heat = 0;
                }

                shooter_controller.state = SHOOTER_READY;
            }
            break;
        }
				
				//卡弹处理
        case SHOOTER_JAMMED:
        {
            shooter_controller.friction_target_speed = friction_target_speed;
            shooter_controller.friction_linear_ratio = friction_ratio;

				  	shooter_controller.jam_timeout_timer++;

            //正在后退
            if(shooter_controller.jam_process_step == 0)
            {
                //是否退到了目标位置
                if(fabs(shooter_controller.dial_target_rad - dial_motor.current_total_angle_rad) < 0.2f)
                {
									  shooter_controller.jam_detect_timer = 0;
                    shooter_controller.jam_process_step = 1;//后退完成
                }
            }
            //后退后等待
            else if (shooter_controller.jam_process_step == 1)
            {
                shooter_controller.jam_detect_timer++;

                //等待一会
                if(shooter_controller.jam_detect_timer > DIAL_JAM_WAIT_TIME)
                {
                    shooter_controller.dial_target_rad = shooter_controller.dial_last_normal_rad + DIAL_ONE_SHOOT_ENCODER;
                    
                    shooter_controller.jam_detect_timer = 0;
                    shooter_controller.state = SHOOTER_SINGLE_SHOT;
                }
            }
						
						//卡弹处理超时
						if(shooter_controller.jam_timeout_timer > 2000) 
            {
                shooter_controller.state = SHOOTER_OFF;
						  	shooter_controller.single_shot_fired_flag = 0;
            }
						
            break;
        }
    }
		
		
		if(shooter_controller.friction_target_speed > 1000.0f)
    {
					//二级目标转速
					float target_rpm_outer = shooter_controller.friction_target_speed;
					
					//弹速自调整
					Bullet_Speed_Adaptive_Process();
					
			//		float friction_final_rpm = friction_base_rpm + auto_speed_offset;
					
					
					//一级目标转速
					float target_rpm_inner = target_rpm_outer * (WHEEL_RADIUS_OUTER / WHEEL_RADIUS_INNER) * shooter_controller.friction_linear_ratio;

					
					//摩擦轮目标转速
					friction_motor_inner_left.target_speed  =  target_rpm_inner;
					friction_motor_inner_right.target_speed = -target_rpm_inner - friction_offs;//-25.0f      - 100.0f
					friction_motor_inner_up.target_speed    = target_rpm_inner;

					friction_motor_outer_left.target_speed  =  target_rpm_outer;
					friction_motor_outer_right.target_speed = -target_rpm_outer;
					friction_motor_outer_up.target_speed    = target_rpm_outer;
					
					
					friction_motor_inner_left.target_current  = (int16_t)PID_Calculate(&friction_motor_inner_left.pid_speed,  friction_motor_inner_left.target_speed,  (float)friction_motor_inner_left.feedback.actual_speed);
					friction_motor_inner_right.target_current = (int16_t)PID_Calculate(&friction_motor_inner_right.pid_speed, friction_motor_inner_right.target_speed, (float)friction_motor_inner_right.feedback.actual_speed);
					friction_motor_inner_up.target_current    = (int16_t)PID_Calculate(&friction_motor_inner_up.pid_speed,    friction_motor_inner_up.target_speed,    (float)friction_motor_inner_up.feedback.actual_speed);
					friction_motor_outer_left.target_current  = (int16_t)PID_Calculate(&friction_motor_outer_left.pid_speed,  friction_motor_outer_left.target_speed,  (float)friction_motor_outer_left.feedback.actual_speed);
					friction_motor_outer_right.target_current = (int16_t)PID_Calculate(&friction_motor_outer_right.pid_speed, friction_motor_outer_right.target_speed, (float)friction_motor_outer_right.feedback.actual_speed);
					friction_motor_outer_up.target_current    = (int16_t)PID_Calculate(&friction_motor_outer_up.pid_speed,    friction_motor_outer_up.target_speed,    (float)friction_motor_outer_up.feedback.actual_speed);
			}
			else
			{
					friction_motor_inner_left.target_current  = 0;
					friction_motor_inner_right.target_current = 0;
					friction_motor_inner_up.target_current    = 0;
					friction_motor_outer_left.target_current  = 0;
					friction_motor_outer_right.target_current = 0;
					friction_motor_outer_up.target_current    = 0;

					friction_motor_inner_left.pid_speed.i_out = 0.0f;
					friction_motor_inner_left.pid_speed.pre_error = 0.0f;

					friction_motor_inner_right.pid_speed.i_out = 0.0f;
					friction_motor_inner_right.pid_speed.pre_error = 0.0f;

					friction_motor_inner_up.pid_speed.i_out = 0.0f;
					friction_motor_inner_up.pid_speed.i_out = 0.0f;

					friction_motor_outer_left.pid_speed.i_out = 0.0f;
					friction_motor_outer_left.pid_speed.pre_error = 0.0f;

					friction_motor_outer_right.pid_speed.i_out = 0.0f;
					friction_motor_outer_right.pid_speed.pre_error = 0.0f;

					friction_motor_outer_up.pid_speed.i_out = 0.0f;
					friction_motor_outer_up.pid_speed.pre_error = 0.0f;			}
		
		
		//摩擦轮是否开启
		if(Check_Friction_Ready() == 1 && shooter_controller.friction_target_speed > 200.0f)
		{
			
		}
		else shooter_controller.friction_state = 0;
 
		
//		//MIT力矩控制
//    if(shooter_controller.state != SHOOTER_OFF)
//    {
//        //拨盘校准
//        if(shooter_controller.state == SHOOTER_CALIBRATING && shooter_controller.homing_step == 0)
//        {
//            dial_motor.target_speed = DIAL_HOMING_SPEED;
//					  dial_motor.target_torque = PID_Calculate(&dial_motor.pid_speed, dial_motor.target_speed, dial_motor.feedback.velocity);
//        }
//        else
//        {
//            dial_motor.target_speed = PID_Calculate(&dial_motor.pid_angle, shooter_controller.dial_target_rad, dial_motor.current_total_angle_rad);
//            dial_motor.target_torque = PID_Calculate(&dial_motor.pid_speed, dial_motor.target_speed, dial_motor.feedback.velocity);
//        }
//    }
		//速度控制
	  if(shooter_controller.state != SHOOTER_OFF)
    {
        if(shooter_controller.state != SHOOTER_CALIBRATING && 
            shooter_controller.state != SHOOTER_JAMMED)
        {
						//当前拨盘误差
						float dial_error = shooter_controller.dial_target_rad - dial_motor.current_total_angle_rad;
						
						float max_error = 2.2f * DIAL_ONE_SHOOT_ENCODER; 
					
						//如果误差离谱
						if(fabs(dial_error) > max_error)
						{
								shooter_controller.dial_target_rad = shooter_controller.dial_last_normal_rad;

								shooter_controller.state = SHOOTER_READY; 
								shooter_controller.single_shot_fired_flag = 0;
								shooter_controller.continuous_fire_timer = 0;
								shooter_controller.jam_detect_timer = 0;
								shooter_controller.jam_timeout_timer = 0;
								shooter_controller.jam_process_step = 0;
						}
				}
			
        //位置环
        dial_motor.target_speed = PID_Calculate(&dial_motor.pid_angle, shooter_controller.dial_target_rad, dial_motor.current_total_angle_rad);

        //校准校准
        if(shooter_controller.state == SHOOTER_CALIBRATING && shooter_controller.homing_step == 0)
        {
            dial_motor.target_speed = DIAL_HOMING_SPEED; 
        }
				
				//如果摩擦轮没转拨盘不转
        if(shooter_controller.state != SHOOTER_CALIBRATING && 
            shooter_controller.state != SHOOTER_JAMMED)
        {
            if(Check_Friction_Ready() == 0)
            {
                dial_motor.target_speed = 0.0f;
            }
				}
    }
    else
    {
			  dial_motor.target_speed = 0.0f;
        dial_motor.target_torque = 0;
    }
    dial_motor.target_total_angle = shooter_controller.dial_target_rad;
}




//检测是否发射弹丸
void Shooter_Check_Real_Fire(void) 
{
    static uint8_t is_spin_up_done = 0;

	  //目标速度
	  float target_rpm_outer = fabs(shooter_controller.friction_target_speed);
		float target_rpm_inner = target_rpm_outer * (WHEEL_RADIUS_OUTER / WHEEL_RADIUS_INNER) * shooter_controller.friction_linear_ratio;

    //关摩擦轮不检测
    if(target_rpm_outer  < 1500.0f)
    {
        is_spin_up_done = 0;
        return; 
    }
		
    //计算平均速度
    float s1_avg_speed = (ABS(friction_motor_inner_left.feedback.actual_speed) + 
                          ABS(friction_motor_inner_right.feedback.actual_speed) + 
                          ABS(friction_motor_inner_up.feedback.actual_speed)) / 3.0f;
    
    float s2_avg_speed = (ABS(friction_motor_outer_left.feedback.actual_speed) + 
                          ABS(friction_motor_outer_right.feedback.actual_speed) + 
                          ABS(friction_motor_outer_up.feedback.actual_speed)) / 3.0f;
    

    if(is_spin_up_done == 0)
    {
        if(fabs(target_rpm_outer - s1_avg_speed) < 500.0f)
        {
            is_spin_up_done = 1;//摩擦轮启动完成
        }
        else return;
    }

    //发射检测
    uint8_t is_s1_drop = (target_rpm_inner - s1_avg_speed) > 200.0f;
    uint8_t is_s2_drop = (target_rpm_outer - s2_avg_speed) > 50.0f;

		//如果同时掉速
    if(is_s2_drop)
    {
        current_shot_status = SHOT_DETECTED;
    }
}





//热量模拟
void Shooter_Update_Heat(void)
{
    uint32_t current_time = xTaskGetTickCount();
    
    static uint32_t last_time = 0;//上一次的时间
    
    if(last_time == 0) 
    {
        last_time = current_time;
        return;
    }
    
    uint32_t dt = current_time - last_time;
    last_time = current_time;

    if(dt == 0) return;
		

    float real_heat = (float)shooter_heat;//裁判系统给的当前真实热量
		
		float cooling_rate = 0.0f;
		if(control_source == CONTROL_SOURCE_VTM) 
		{
			  cooling_rate = (float)shooter_cooling;
		}
		else
		{
				cooling_rate = 10000.0f;//(float)shooter_cooling;
		}
		
    
    //计算这段时间内的冷却量
    float cooling_amount = cooling_rate * ((float)dt / 1000.0f);

    if(shooter_predict_heat > 0.0f) shooter_predict_heat -= cooling_amount;
    if(shooter_predict_heat < 0.0f) shooter_predict_heat = 0.0f;
		
    //同步校准
    if(real_heat > shooter_predict_heat) 
    {
        shooter_predict_heat = real_heat;
    }
		
		
    if(shooter_controller.state != SHOOTER_SINGLE_SHOT && (current_time - last_shoot_time > 500))
    {
        if(shooter_predict_heat > real_heat + 30.0f)
        {
            shooter_predict_heat = real_heat - 15.0f;
        }
    }
}




//是否允许开火
uint8_t Shooter_Check_Heat_Allowed(void)
{
	if(control_source == CONTROL_SOURCE_VTM) 
	{
		  return(shooter_predict_heat + HERO_HEAT_PER_SHOT <= (float)heat_limit);
	}
	else
	{
      return(shooter_predict_heat + HERO_HEAT_PER_SHOT < (float)10000.0f - HEAT_SAFETY_MARGIN);
	}
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
                    update_dial_motor(rx_msg.data);
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
                    update_friction_motors(rx_msg.data, rx_msg.can_id);
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
		
if(vofa_timer > 0) vofa_timer--;
else print_initial_speed = 0.0f;
		
		
////自瞄
//			            VOFA_Print_Wave(4,
//(float)gimbal_yaw_smc_imu.ref_pos,
//(float)gimbal_yaw_smc_imu.fdb_pos,
//(float)gimbal_pitch_smc_imu.ref_pos,
//(float)gimbal_pitch_smc_imu.fdb_pos);
//		

					            VOFA_Print_Wave(10,
//吊射
(float)initial_speed,  //弹速
(float)bullet_speed_avg,     //平均弹速
(float)bullet_speed_variance,//方差
(float)bullet_speed_range, //极差
(float)friction_motor_outer_left.feedback.actual_speed,
-(float)friction_motor_outer_right.feedback.actual_speed,
(float)friction_motor_outer_up.feedback.actual_speed,
(float)friction_motor_inner_left.feedback.actual_speed,
-(float)friction_motor_inner_right.feedback.actual_speed,
(float)friction_motor_inner_up.feedback.actual_speed);



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





//更新拨盘电机
void update_dial_motor(const uint8_t* data)
{
		dial_motor.id = (data[0])&0x0F;
	  dial_motor.state = (data[0])>>4;
	
    uint16_t pos_int = (data[1] << 8) | data[2];
    uint16_t vel_int = (data[3] << 4) | (data[4] >> 4);
    uint16_t tor_int = ((data[4] & 0x0F) << 8) | data[5];

    //获取当前单圈位置(rad, -12.5 ~ +12.5)
    float current_pos_rad = uint_to_float(pos_int, -3.141593f, 3.141593f, 16);
    
	
    //首次运行，初始化
    if(dial_motor.last_feedback_position == 0.0f && dial_motor.round_count == 0)
    {
        dial_motor.last_feedback_position = current_pos_rad;
    }

    float pos_diff = current_pos_rad - dial_motor.last_feedback_position;

    //过零处理
    if(pos_diff > 3.14f)
    {
        dial_motor.round_count--;
    }
    else if (pos_diff < -3.14f)
    {
        dial_motor.round_count++;
    }

    //计算总角度
    dial_motor.current_total_angle_rad = dial_motor.round_count * (6.283186f) + current_pos_rad;
    dial_motor.current_total_angle_deg = dial_motor.current_total_angle_rad * RAD_TO_DEG;
    
    //更新
    dial_motor.feedback.position = current_pos_rad;
    dial_motor.feedback.velocity = uint_to_float(vel_int, -10, 10, 12);
    dial_motor.feedback.torque   = uint_to_float(tor_int, DM4340_DIAL_T_MIN, DM4340_DIAL_T_MAX, 12);
    dial_motor.last_feedback_position = current_pos_rad;
		
    dial_motor.last_update_tick = osKernelGetTickCount();
}




//更新摩擦轮电机
void update_friction_motors(const uint8_t* data, uint32_t can_id)
{
    Motor_Controller* motor = NULL;

    //根据ID匹配电机指针
    switch(can_id - 0x200)
    {
        case FRICTION_INNER_LEFT_ID:  motor = &friction_motor_inner_left;  break;
        case FRICTION_INNER_RIGHT_ID: motor = &friction_motor_inner_right; break;
        case FRICTION_INNER_UP_ID:    motor = &friction_motor_inner_up;    break;
        
        case FRICTION_OUTER_LEFT_ID:  motor = &friction_motor_outer_left;  break;
        case FRICTION_OUTER_RIGHT_ID: motor = &friction_motor_outer_right; break;
        case FRICTION_OUTER_UP_ID:    motor = &friction_motor_outer_up;    break;
        
        default: return; // 未知ID
    }

    if(motor != NULL)
    {
        motor->feedback.angle = (uint16_t)(data[0] << 8) | data[1];
        motor->feedback.actual_speed = (int16_t)((data[2] << 8) | data[3]);
        motor->feedback.current = (int16_t)((data[4] << 8) | data[5]);
        motor->feedback.temperature = data[6];
        motor->last_update_tick = osKernelGetTickCount();
    }
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

    initial_speed  = rx_msg.pack.bullet_speed;
    shooter_heat  = rx_msg.pack.shooter_heat;
    heat_limit    = rx_msg.pack.heat_limit;
    shooter_cooling = rx_msg.pack.cooling_value;
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
