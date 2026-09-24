/**
 * @file gimbal_control.c
 * @brief 云台控制实现：Yaw/Pitch 闭环、自瞄/基地打击参数与目标角管理
 */

#include "cmsis_os.h"
#include "motor.h"
#include "pid.h"
#include "smc.h"
#include "gimbal_control.h"
#include <math.h>
#include <string.h>

// ===== 电机与控制器私有状态 =====
static Motor_Controller gimbal_yaw_motor;
static Motor_Controller gimbal_pitch_motor;

static SMC_Handle_t gimbal_yaw_smc_imu;
static SMC_Handle_t gimbal_pitch_smc_imu;

//IMU 目标角度
static float rc_target_world_yaw_angle = 0.0f;
static float rc_target_world_pitch_angle = 0.0f;
//Encoder 目标角度
static float rc_target_yaw_encoder = 0.0f;
static float rc_target_pitch_encoder = 0.0f;

static Gimbal_Mode_e gimbal_mode = GIMBAL_MODE_IMU;
static uint8_t gimbal_control_initialized = 0;

static bool active = false;
static bool base_hitting = false;

static Gimbal_Output_t output;
static Gimbal_Debug_Data_t debug;

// ===== 控制参数 =====

//Yaw
//速度环
static const float gimbal_yaw_speed_kp_encoder = 0.25f;//0.45
static const float gimbal_yaw_speed_ki_encoder = 0.0f;
static const float gimbal_yaw_speed_kd_encoder = 3.25f;//4.25
static const float gimbal_yaw_speed_Integral_Limit_encoder = 1.0f;
static const float gimbal_yaw_speed_MaxOutput_encoder = 10.0f; //最大输出电流16.5A(max16.5A)
//位置环
static const float gimbal_yaw_angle_kp_encoder = 0.195f;//之前同步带是0.1f
static const float gimbal_yaw_angle_ki_encoder = 0.0f;
static const float gimbal_yaw_angle_kd_encoder = 0.0f;
static const float gimbal_yaw_angle_Integral_Limit_encoder = 0.0f;
static const float gimbal_yaw_angle_MaxOutput_encoder = 30.0f; //最大目标速度RPM


//SMC
static const float gimbal_yaw_c = 0.2f;          // c: 滑模面系数 ，决定回中快慢           0.35
static const float gimbal_yaw_k_exp = 10.0f;      // k_exp: 指数趋近 ，决定出力大小          7.5
static const float gimbal_yaw_epsilon = 0.0f;    // epsilon: 等速趋近 (克服静摩擦)，给小点防止震动
static const float gimbal_yaw_beta = 0.0f;       // beta: 干扰观测器增益 ，不够大挂不住
static const float gimbal_yaw_boundary = 0.5f;   // boundary: 边界层，防抖 
static const float gimbal_yaw_out_max = 10.0f;    // out_max: 最大输出                       10

//SMC_Vision
static float gimbal_yaw_c_vision = 3.5f;          // c: 滑模面系数 ，决定回中快慢        //1
static float gimbal_yaw_k_exp_vision = 0.75f;      // k_exp: 指数趋近 ，决定出力大小      //0.3
static const float gimbal_yaw_epsilon_vision = 0.0f;    // epsilon: 等速趋近 (克服静摩擦)，给小点防止震动
static const float gimbal_yaw_beta_vision = 0.0f;       // beta: 干扰观测器增益 ，不够大挂不住
static const float gimbal_yaw_boundary_vision = 0.5f;   // boundary: 边界层，防抖
static const float gimbal_yaw_out_max_vision = 5.0f;    // out_max: 最大输出


//Pitch

//速度环
static const float gimbal_pitch_speed_kp_encoder = 105.0f;//450
static const float gimbal_pitch_speed_ki_encoder = 0.1f;//15
static const float gimbal_pitch_speed_kd_encoder = 0.0f;
static const float gimbal_pitch_speed_Integral_Limit_encoder = 9000.0f;
static const float gimbal_pitch_speed_MaxOutput_encoder = 25000.0f;//最大输出电流
//位置环
static const float gimbal_pitch_angle_kp_encoder = 8.5f;//0.8
static const float gimbal_pitch_angle_ki_encoder = 0.0f;
static const float gimbal_pitch_angle_kd_encoder = 0.0f;
static const float gimbal_pitch_angle_Integral_Limit_encoder = 50.0f;
static const float gimbal_pitch_angle_MaxOutput_encoder = 400.0f;//最大目标速度RPM


//SMC
static const float gimbal_pitch_c = 5.0f;          // c: 滑模面系数 ，决定回中快慢           0.35
static const float gimbal_pitch_k_exp = 500.0f;      // k_exp: 指数趋近 ，决定出力大小          7.5
static const float gimbal_pitch_epsilon = 0.0f;    // epsilon: 等速趋近(克服静摩擦)
static const float gimbal_pitch_beta = 40.0f;       // beta: 干扰观测器增益
static const float gimbal_pitch_boundary = 0.5f;   // boundary: 边界层，防抖 
static const float gimbal_pitch_out_max = 25000.0f;    // out_max: 最大输出    

//SMC_Vision
static float gimbal_pitch_c_vision = 7.5f;//7          // c: 滑模面系数 ，决定回中快慢    //前哨8.0f   
static float gimbal_pitch_k_exp_vision = 700.0f;//600      // k_exp: 指数趋近 ，决定出力大小      //前哨550.0f
static const float gimbal_pitch_epsilon_vision = 0.0f;    // epsilon: 等速趋近 (克服静摩擦)
static const float gimbal_pitch_beta_vision = 40.0f;       // beta: 干扰观测器增益
static const float gimbal_pitch_boundary_vision = 0.5f;   // boundary: 边界层，防抖
static const float gimbal_pitch_out_max_vision = 25000.0f;    // out_max: 最大输出


//SMC_Rader
static float gimbal_pitch_c_rader = 5.0f;          // c: 滑模面系数 ，决定回中快慢           //7
static float gimbal_pitch_k_exp_rader = 200.0f;      // k_exp: 指数趋近 ，决定出力大小       //400
static float gimbal_pitch_epsilon_rader = 3000.0f;    // epsilon: 等速趋近 (克服静摩擦)      //3000
static float gimbal_pitch_beta_rader = 40.0f;       // beta: 干扰观测器增益                  //40


// ===== 常量 =====
#define GIMBAL_YAW_CENTER_ENCODER 30002

#define PITCH_HORIZON_DEGREE  121.1f  //枪管水平时的机械角度

#define LK_RAW_TO_CURRENT  (16.5f / 2048.0f)

static int16_t friction_offset = 1500;


// ===== 私有辅助函数 =====

//初始化单轴电机
static void Motor_Init(Motor_Controller* motor, uint16_t id, FDCAN_HandleTypeDef* hfdcan)
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


//编码器累计角度更新(圈数累计与过零处理)
static void Gimbal_Angle_Update(Motor_Controller *motor, float encoder_range)
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


//Yaw编码器偏差(±半圈归一化), 供底盘跟随/坐标系转换使用
float Gimbal_Get_Yaw_Encoder_Diff(void)
{
    float diff = (float)gimbal_yaw_motor.feedback.angle - GIMBAL_YAW_CENTER_ENCODER;

    while(diff > 32768.0f) diff -= 65536.0f;
    while(diff < -32768.0f) diff += 65536.0f;

    return diff;
}


// ===== 对外 API 实现 =====

//初始化云台电机与控制器
void Gimbal_Control_Init(void)
{
    //Yaw
    Motor_Init(&gimbal_yaw_motor, 1 , &hfdcan1);

    //Pitch
    Motor_Init(&gimbal_pitch_motor, GIMBAL_PITCH_MOTOR_ID, &hfdcan3);

    //Yaw
    SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
    //Pitch
    SMC_Init(&gimbal_pitch_smc_imu, gimbal_pitch_c, gimbal_pitch_k_exp, gimbal_pitch_epsilon, gimbal_pitch_beta, gimbal_pitch_boundary, gimbal_pitch_out_max, 0.02f, 0.001f);
}


//机器人重新使能时的云台侧复位
void Gimbal_On_Robot_Start(const IMU_Data_t* imu)
{
    SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
    SMC_Init(&gimbal_pitch_smc_imu, gimbal_pitch_c, gimbal_pitch_k_exp, gimbal_pitch_epsilon, gimbal_pitch_beta, gimbal_pitch_boundary, gimbal_pitch_out_max, 0.02f, 0.001f);

    rc_target_world_yaw_angle = imu->yaw_angle;
    rc_target_world_pitch_angle = imu->pitch_angle;
}


//机器人失能时的云台侧清零
void Gimbal_On_Robot_Off(void)
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

    gimbal_yaw_motor.target_current = 0;
    gimbal_pitch_motor.target_current = 0;
}


//云台控制模式切换(内部做边沿检测与重初始化)
void Gimbal_Set_Mode(Gimbal_Mode_e mode, const IMU_Data_t* imu)
{
    if(mode != gimbal_mode)
    {
        if(mode == GIMBAL_MODE_ENCODER)//发射模式
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

            rc_target_world_yaw_angle = imu->yaw_angle;
            rc_target_world_pitch_angle = imu->pitch_angle;
        }
    }
    gimbal_mode = mode;
}


//是否执行闭环控制
void Gimbal_Set_Active(bool enable)
{
    active = enable;
}


//自瞄参数切换(内部做边沿检测)
void Gimbal_Set_Aiming(bool aiming)
{
    static uint8_t is_aiming_last_frame = 0;

    if(aiming != is_aiming_last_frame)
    {
        if(aiming)//自瞄
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
    is_aiming_last_frame = aiming;
}


//雷达辅吊云台参数切换(内部做边沿检测)
void Gimbal_Set_Base_Hitting(bool hitting)
{
    static uint8_t is_base_hitting_last_frame = 0;

    if(hitting != is_base_hitting_last_frame)
    {
        if(hitting)
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
    is_base_hitting_last_frame = hitting;

    base_hitting = hitting;
}


//电机重新上线后的目标对齐
void Gimbal_Handle_Motor_Reconnect(const IMU_Data_t* imu)
{
    static bool yaw_was_offline = false;

    if(gimbal_yaw_motor.is_online && yaw_was_offline)
    {
        rc_target_world_yaw_angle = imu->yaw_angle;
    }
    yaw_was_offline = !gimbal_yaw_motor.is_online;

    static bool pitch_was_offline = false;
    if(gimbal_pitch_motor.is_online && pitch_was_offline)
    {
        rc_target_world_pitch_angle = imu->pitch_angle;
    }
    pitch_was_offline = !gimbal_pitch_motor.is_online;
}


//IMU首次就绪对齐
void Gimbal_Sync_On_First_Ready(const IMU_Data_t* imu)
{
    if(gimbal_control_initialized == 0 && (imu->yaw_angle != 0.0f && imu->pitch_angle != 0.0f))//陀螺仪第一次就绪
    {
        rc_target_world_yaw_angle = imu->yaw_angle;
        rc_target_world_pitch_angle = imu->pitch_angle;
        gimbal_control_initialized = 1;
    }
}


bool Gimbal_Is_Initialized(void)
{
    return gimbal_control_initialized != 0;
}


//RESET键：清除首次就绪标记
void Gimbal_Reset_Initialized(void)
{
    gimbal_control_initialized = 0;
}


//RESET键：Yaw SMC复位
void Gimbal_Reset_Yaw_SMC(void)
{
    SMC_Init(&gimbal_yaw_smc_imu, gimbal_yaw_c, gimbal_yaw_k_exp, gimbal_yaw_epsilon, gimbal_yaw_beta, gimbal_yaw_boundary, gimbal_yaw_out_max, 0.0f, 0.001f);
}


//将世界目标角同步到当前IMU角度
void Gimbal_Sync_World_Target_To_IMU(const IMU_Data_t* imu)
{
    rc_target_world_yaw_angle = imu->yaw_angle;
    rc_target_world_pitch_angle = imu->pitch_angle;
}


// ===== 目标角度接口 =====

void Gimbal_Set_World_Target(float yaw, float pitch)
{
    rc_target_world_yaw_angle = yaw;
    rc_target_world_pitch_angle = pitch;
}

void Gimbal_Set_World_Yaw(float yaw)
{
    rc_target_world_yaw_angle = yaw;
}

float Gimbal_Get_World_Yaw(void)
{
    return rc_target_world_yaw_angle;
}

float Gimbal_Get_World_Pitch(void)
{
    return rc_target_world_pitch_angle;
}

void Gimbal_Add_World_Target(float dyaw, float dpitch)
{
    rc_target_world_yaw_angle += dyaw;
    rc_target_world_pitch_angle += dpitch;
}

void Gimbal_Clamp_World_Pitch(float min_deg, float max_deg)
{
    if(rc_target_world_pitch_angle > max_deg)
    {
        rc_target_world_pitch_angle = max_deg;
    }
    else if(rc_target_world_pitch_angle < min_deg)
    {
        rc_target_world_pitch_angle = min_deg;
    }
}

void Gimbal_Add_Encoder_Target(float dyaw, float dpitch)
{
    rc_target_yaw_encoder += dyaw;
    rc_target_pitch_encoder += dpitch;
}

void Gimbal_Clamp_Encoder_Pitch(float min_val, float max_val)
{
    if(rc_target_pitch_encoder > max_val)
    {
        rc_target_pitch_encoder = max_val;
    }
    else if(rc_target_pitch_encoder < min_val)
    {
        rc_target_pitch_encoder = min_val;
    }
}


// ===== 闭环控制 =====

void Gimbal_Control_Update(const IMU_Data_t* imu)
{
    float   gimbal_yaw_current = 0.0f;
    int16_t gimbal_pitch_current = 0;

    if(active)
    {
        if(gimbal_mode == GIMBAL_MODE_IMU)//跟随
        {
            //Yaw
            float yaw_angle_diff = rc_target_world_yaw_angle - imu->yaw_angle;
            while(yaw_angle_diff > 180.0f) yaw_angle_diff -= 360.0f;
            while(yaw_angle_diff < -180.0f) yaw_angle_diff += 360.0f;

            gimbal_yaw_smc_imu.ref_pos = rc_target_world_yaw_angle; // 目标 deg
            gimbal_yaw_smc_imu.ref_spd = 0.0f;
            gimbal_yaw_smc_imu.fdb_pos = imu->yaw_angle;     // 反馈 deg
            gimbal_yaw_smc_imu.fdb_spd = imu->yaw_gyro;      // 反馈 deg/s

            gimbal_yaw_current  = -SMC_Calc(&gimbal_yaw_smc_imu);


            //Pitch
            float pitch_angle_diff = rc_target_world_pitch_angle - imu->pitch_angle;

            gimbal_pitch_smc_imu.ref_pos = rc_target_world_pitch_angle;
            gimbal_pitch_smc_imu.ref_spd = 0.0f; //目标速度为0
            gimbal_pitch_smc_imu.fdb_pos = imu->pitch_angle;
            gimbal_pitch_smc_imu.fdb_spd = imu->pitch_gyro;

            gimbal_pitch_current = (int16_t)SMC_Calc(&gimbal_pitch_smc_imu);


            if(base_hitting)
            {
                float current_err = rc_target_world_pitch_angle - imu->pitch_angle;

                if(current_err > 0.05f)
                {
                    gimbal_pitch_current += friction_offset;
                }
                else if (current_err < -0.05f)
                {
                    gimbal_pitch_current -= friction_offset;
                }
            }

            float target_yaw_speed = 0.0f;
            float target_pitch_speed = 0.0f;
            gimbal_yaw_motor.pid_angle.target = rc_target_world_yaw_angle;
            gimbal_yaw_motor.pid_angle.measured = imu->yaw_angle;
            gimbal_yaw_motor.pid_speed.target = target_yaw_speed;
            gimbal_yaw_motor.pid_speed.measured = imu->yaw_gyro;
            gimbal_yaw_motor.target_current = gimbal_yaw_current;
            gimbal_pitch_motor.pid_angle.target = rc_target_world_pitch_angle;
            gimbal_pitch_motor.pid_angle.measured = imu->pitch_angle;
            gimbal_pitch_motor.pid_speed.target = target_pitch_speed;
            gimbal_pitch_motor.pid_speed.measured = imu->pitch_gyro;
            gimbal_pitch_motor.target_current = gimbal_pitch_current;
        }
        else
        {
            //Yaw
            float yaw_encoder_diff = rc_target_yaw_encoder - gimbal_yaw_motor.current_total_angle;
            float target_yaw_speed = PID_Calculate(&gimbal_yaw_motor.pid_angle, yaw_encoder_diff, 0.0f);
            gimbal_yaw_current = PID_Calculate(&gimbal_yaw_motor.pid_speed, target_yaw_speed, imu->yaw_gyro);


            //Pitch
            float pitch_encoder_diff = rc_target_pitch_encoder - gimbal_pitch_motor.current_total_angle;
            float target_pitch_speed = PID_Calculate(&gimbal_pitch_motor.pid_angle, pitch_encoder_diff, 0.0f);

            gimbal_pitch_current = (int16_t)PID_Calculate(&gimbal_pitch_motor.pid_speed, target_pitch_speed, imu->pitch_gyro);

            gimbal_yaw_motor.pid_angle.target = rc_target_yaw_encoder;
            gimbal_yaw_motor.pid_angle.measured = gimbal_yaw_motor.current_total_angle;
            gimbal_yaw_motor.pid_speed.target = target_yaw_speed;
            gimbal_yaw_motor.pid_speed.measured = imu->yaw_gyro;
            gimbal_yaw_motor.target_current = (int16_t)(gimbal_yaw_current);
            gimbal_pitch_motor.pid_angle.target = rc_target_pitch_encoder;
            gimbal_pitch_motor.pid_angle.measured = gimbal_pitch_motor.current_total_angle;
            gimbal_pitch_motor.pid_speed.target = target_pitch_speed;
            gimbal_pitch_motor.pid_speed.measured = imu->pitch_gyro;
            gimbal_pitch_motor.target_current = gimbal_pitch_current;
        }
    }
    else
    {
        gimbal_yaw_current = 0.0f;
        gimbal_pitch_current = 0;
        gimbal_yaw_motor.target_current = 0;
        gimbal_pitch_motor.target_current = 0;
    }

    //输出快照，主调度器统一执行 CAN 发送
    output.yaw_current   = gimbal_yaw_current;
    output.pitch_current = gimbal_pitch_current;
    output.yaw_degree    = gimbal_yaw_motor.mechanical_degree;
    output.pitch_degree  = PITCH_HORIZON_DEGREE - gimbal_pitch_motor.mechanical_degree;
}


const Gimbal_Output_t* Gimbal_Get_Output(void)
{
    return &output;
}


//调试打印快照
const Gimbal_Debug_Data_t* Gimbal_Get_Debug_Data(void)
{
    debug.yaw_ref_pos       = gimbal_yaw_smc_imu.ref_pos;
    debug.yaw_fdb_pos       = gimbal_yaw_smc_imu.fdb_pos;
    debug.pitch_ref_pos     = gimbal_pitch_smc_imu.ref_pos;
    debug.pitch_fdb_pos     = gimbal_pitch_smc_imu.fdb_pos;
    debug.yaw_motor_angle   = (float)gimbal_yaw_motor.feedback.angle;
    debug.pitch_motor_angle = (float)gimbal_pitch_motor.feedback.angle;
    return &debug;
}


// ===== CAN 反馈解码 =====

//更新云台Yaw电机(LK)
void Gimbal_Update_Yaw(const uint8_t* data)
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
void Gimbal_Update_Pitch(const uint8_t* data)
{
    gimbal_pitch_motor.feedback.angle = (uint16_t)(data[0] << 8) | data[1];
    gimbal_pitch_motor.feedback.actual_speed = (int16_t)((data[2] << 8) | data[3]);
    gimbal_pitch_motor.feedback.current = (int16_t)((data[4] << 8) | data[5]);
    gimbal_pitch_motor.feedback.temperature = data[6];
    gimbal_pitch_motor.last_update_tick = osKernelGetTickCount();
    Gimbal_Angle_Update(&gimbal_pitch_motor, 8192.0f);
}


// ===== 电机在线检测 =====

static void Gimbal_Check_Motor_Online(Motor_Controller* motor, uint32_t current_tick)
{
    if(current_tick - motor->last_update_tick > 200)
    {
        motor->is_online = false;
    }
    else
    {
        motor->is_online = true;
    }
}


void Gimbal_Check_Motors_Online(uint32_t current_tick)
{
    Gimbal_Check_Motor_Online(&gimbal_yaw_motor, current_tick);
    Gimbal_Check_Motor_Online(&gimbal_pitch_motor, current_tick);
}
