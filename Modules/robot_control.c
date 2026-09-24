/**
 * @file robot_control.c
 * @brief 机器人总调度模块(Robot_Control_Loop)
 *
 * 整机级状态(模式、瞄准标志、LED 状态与各模式遥控增益)与最终的 CAN 发送
 * 都集中在本文件; 具体控制计算在 gimbal/chassis/shooter 三个模块内完成。
 */

#include "robot_control.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdbool.h>
#include "remote_control.h"
#include "INS_task.h"
#include "bsp_DR16.h"
#include "vision_link.h"
#include "servo.h"
#include "tim.h"
#include "fdcan.h"
#include "can_transmit.h"
#include "robot_mode.h"
#include "gimbal_control.h"
#include "chassis_control.h"
#include "shooter_control.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))

// ===== 整机状态(仅本模块持有) =====

// 系统LED状态(device_task 通过 Robot_Get_Led_State 读取)
static volatile LED_State_e led_state = LED_STATE_BOOTING;

// ===== 瞄准状态 =====

// 是否处于瞄准中(自瞄或手动自瞄)
static uint8_t is_aiming_now = 0;

// ===== 控制参数 =====

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

// ===== 对外接口 =====

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
    return led_state;
}

// ===== 内部辅助函数 =====

// V键复位: Yaw归零到180度并复位SMC(失能与战斗模式下均可触发)
static void Robot_Reset_Yaw_To_180(void)
{
    led_state = LED_STATE_RESET;

    LK_Send_Torque(&hfdcan1, 0);
    DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);

    Reset_All_IMU_Yaw_Only();

    Gimbal_Set_World_Yaw(180.0f);

    Gimbal_Reset_Yaw_SMC();
}

// 主动下发零输出: 云台/摩擦轮/拨盘停机(IMU未就绪/复位/失能时使用)
static void Robot_Zero_Outputs(void)
{
    LK_Send_Torque(&hfdcan1, 0);
    DJI_Motor_Send_Command(&hfdcan2, 0x1FF, 0, 0, 0, 0);
    DJI_Motor_Send_Command(&hfdcan2, 0x200, 0, 0, 0, 0);
    DJI_Motor_Send_Command(&hfdcan3, 0x1FF, 0, 0, 0, 0);
    Dial_Send_Speed(&hfdcan1, 0);
}


// 复位手势处理: 拨轮长按(仅OFF)触发整机复位, V键(OFF/BATTLE)触发Yaw归零;
// 整机复位含500拍等待(非阻塞), 返回1表示仍在等待期, 主循环本拍直接返回
static uint8_t Robot_Handle_Reset(const RC_Data_t* rc, Robot_Mode_e robot_mode)
{
    static uint32_t reset_trigger_timer = 0;   //拨轮长按计时
    static uint32_t reset_wait_timer = 0;      //复位执行前等待
    static bool key_V_last_state = false;

    //等待期: 不执行模式调度与CAN发送, LED保持复位指示
    if(reset_wait_timer > 0)
    {
        led_state = LED_STATE_RESET;
        reset_wait_timer--;

        if(reset_wait_timer > 0)
        {
            return 1;
        }

        //等待结束, 执行复位
        Reset_All_IMU_Yaw_Only();
        Shooter_Reset_Stats();
        Gimbal_Reset_Initialized();

        key_V_last_state = rc->key.map.V; //更新状态

        return 0;
    }

    uint8_t v_key_edge = (rc->key.map.V && !key_V_last_state);

    if(robot_mode == ROBOT_MODE_OFF)
    {
        if(rc->rc.wheel < -500)
        {
            reset_trigger_timer++;

            //持续足够长时间
            if(reset_trigger_timer >= 2000)
            {
                reset_trigger_timer = 0;
                led_state = LED_STATE_RESET;

                Robot_Zero_Outputs();

                reset_wait_timer = 500;

                return 1;
            }
        }
        else if(v_key_edge)
        {
            Robot_Reset_Yaw_To_180();
        }
        else
        {
            reset_trigger_timer = 0;
        }
    }
    else
    {
        reset_trigger_timer = 0;

        if(robot_mode == ROBOT_MODE_BATTLE && v_key_edge)
        {
            Robot_Reset_Yaw_To_180();
        }
    }

    key_V_last_state = rc->key.map.V; //更新状态

    return 0;
}


// 视觉目标解析: 弧度转角度, +180坐标系偏移, 按IMU yaw取最短路径;
// 与当前世界目标偏差在 gate_deg 内才写入, 返回1=已写入
static uint8_t Resolve_Vision_Aim_Target(const IMU_Data_t* imu_data, float gate_deg)
{
    float vision_yaw_deg = VisionData.target_yaw_rad * RAD_TO_DEG;
    float vision_pitch_deg = VisionData.target_pitch_rad * RAD_TO_DEG;

    float vision_target_yaw_angle = 180.0f + vision_yaw_deg;
    float vision_target_pitch_angle = 180.0f + vision_pitch_deg;

    //最短路径
    while(vision_target_yaw_angle - imu_data->yaw_angle > 180.0f)  vision_target_yaw_angle -= 360.0f;
    while(vision_target_yaw_angle - imu_data->yaw_angle < -180.0f) vision_target_yaw_angle += 360.0f;

    if(fabs(vision_target_yaw_angle - Gimbal_Get_World_Yaw()) < gate_deg && fabs(vision_target_pitch_angle - Gimbal_Get_World_Pitch()) < gate_deg)
    {
        Gimbal_Set_World_Target(vision_target_yaw_angle, vision_target_pitch_angle);
        return 1;
    }

    return 0;
}


// 自瞄/手动自瞄公共主体: 视觉跟踪或右杆跟随 + Pitch限位 + 底盘移动(风格由参数决定)
static void Handle_Aiming_Mode(const RC_Data_t* rc, const IMU_Data_t* imu_data, float yaw_encoder_diff, Chassis_Move_Style_e move_style)
{
    if(Vision_IsOnline() && VisionData.is_tracking == 1)
    {
        Resolve_Vision_Aim_Target(imu_data, 60.0f);
    }
    else
    {
        //更新目标角度
        float yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
        float pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;

        Gimbal_Add_World_Target(yaw_angle_increment, pitch_angle_increment);

        VisionData.target_yaw_rad = 0.0f;
        VisionData.target_pitch_rad = 0.0f;
    }

    //Pitch限位
    Gimbal_Clamp_World_Pitch(135.5f, 194.1f);

    //底盘(坐标变换, 具体风格由调用方决定)
    Chassis_Update_Move(rc, yaw_encoder_diff, move_style);
}

// 雷达基地打击辅助: 按视觉基地目标限幅步进逼近, 连续对准200拍后确认锁定
// 返回: 0=无基地目标, 1=本拍处理中, 2=本拍达到对准确认(已置锁定LED)
static uint8_t Robot_Handle_Base_Aim(const IMU_Data_t* imu_data)
{
    static uint16_t lock_confirm_timer = 0;

    //无基地目标
    if(VisionData.target_yaw_base_hit == 0 && VisionData.target_pitch_base_hit == 0) return 0;

    float vision_base_yaw_deg = VisionData.target_yaw_base_hit * RAD_TO_DEG;
    float vision_base_pitch_deg = VisionData.target_pitch_base_hit * RAD_TO_DEG;

    if(fabsf(vision_base_yaw_deg) < 180 && fabsf(vision_base_pitch_deg) < 90)
    {
        float yaw_step = vision_base_yaw_deg * kyaw;
        if(yaw_step > yaw_step_max)  yaw_step = yaw_step_max;
        if(yaw_step < -yaw_step_max) yaw_step = -yaw_step_max;

        Gimbal_Add_World_Target(yaw_step, 0.0f);

        float vision_target_base_pitch_angle = 180.0f - vision_base_pitch_deg;

        float pitch_diff = vision_target_base_pitch_angle - Gimbal_Get_World_Pitch();

        float pitch_step = pitch_diff * kpitch;
        if(pitch_step > pitch_step_max)  pitch_step = pitch_step_max;
        if(pitch_step < -pitch_step_max) pitch_step = -pitch_step_max;

        Gimbal_Add_World_Target(0.0f, pitch_step);

        float yaw_error = fabsf(vision_base_yaw_deg);
        float actual_pitch_err = fabsf(vision_target_base_pitch_angle - imu_data->pitch_angle);

        if(yaw_error <= 0.1f && actual_pitch_err <= 0.1f)
        {
            lock_confirm_timer++;
            if(lock_confirm_timer > 200)
            {
                led_state = LED_STATE_BASE_HIT_READY;
                lock_confirm_timer = 200;
                return 2;
            }
        }
        else if(yaw_error > 0.1f || actual_pitch_err > 0.1f)
        {
            lock_confirm_timer = 0;
            led_state = LED_STATE_ENABLED;
        }
    }

    return 1;
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
        last_robot_mode = ROBOT_MODE_OFF; 

		  	Robot_Zero_Outputs();
        return; 
    }


				//模式选择: 由 robot_mode 模块根据遥控来源/上坎状态决定, 同时维护模式LED指示
		Robot_Mode_Update(rc, control_source);
		robot_mode = Robot_Mode_Get();
		led_state = Robot_Mode_Get_Led_State();


		//RESET: 复位手势处理(拨轮长按整机复位/V键Yaw归零)
		if(Robot_Handle_Reset(rc, robot_mode))
		{
			return;   //等待期: 本拍不执行后续模式调度与CAN发送
		}


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

                Robot_Zero_Outputs();


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
								//雷达基地打击(R键)
						    if(rc->key.map.R)
						    {
										uint8_t base_aim_ret = Robot_Handle_Base_Aim(imu_data);
										if(base_aim_ret != 0)
										{
												is_base_hitting_now = 1;
												if(base_aim_ret == 2) is_base_locked = 1;   //对准确认后置基地锁定(供底盘上报)
										}
						    }
								else if(Vision_IsOnline() &&VisionData.is_tracking == 1 && rc->mouse.press_r )//右键自瞄   //&&
								{
											Resolve_Vision_Aim_Target(imu_data, 90.0f);
								}
								else
								{
										float yaw_increment = -(float)rc->mouse.x * RC_TO_ONE * ky_mouse;
										float pitch_increment = -(float)rc->mouse.y * RC_TO_ONE * kpit_mouse;

										Gimbal_Add_World_Target(yaw_increment, pitch_increment);

										VisionData.target_yaw_rad = 0.0f;
										VisionData.target_pitch_rad = 0.0f;
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
					   //雷达: 拨轮触发基地打击(连续对准200拍后确认, 由辅助函数内部处理)
					   if(ABS(rc->rc.wheel) > 500 && Robot_Handle_Base_Aim(imu_data) != 0)
						 {
							    is_base_hitting_now = 1;
						 }
						 else if(Vision_IsOnline() && VisionData.is_tracking == 1)
              {
								  Resolve_Vision_Aim_Target(imu_data, 60.0f);
              }
						else
						{
									//更新目标角度
									float yaw_angle_increment = -(float)rc->rc.right_x * RC_TO_ONE * ky_follow ;
									float pitch_angle_increment = -(float)rc->rc.right_y * RC_TO_ONE * kpit_follow;

									Gimbal_Add_World_Target(yaw_angle_increment, pitch_angle_increment);

									VisionData.target_yaw_rad = 0.0f;
									VisionData.target_pitch_rad = 0.0f;
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
						 Handle_Aiming_Mode(rc, imu_data, yaw_encoder_diff, CHASSIS_MOVE_AIMING);
            break;
				}

				case ROBOT_MODE_M_AIMING:
				{
						  Handle_Aiming_Mode(rc, imu_data, yaw_encoder_diff, CHASSIS_MOVE_M_AIMING);
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

        //射击: 组装系统上下文并交给 Shooter 模块(输入翻译/射击决策在模块内部完成)
        Shooter_Context_t shooter_ctx;
        shooter_ctx.rc             = rc;
        shooter_ctx.robot_mode     = robot_mode;
        shooter_ctx.gimbal_mode    = gimbal_mode;
        shooter_ctx.is_aiming_now  = is_aiming_now;
        shooter_ctx.control_source = control_source;

        Shooter_Control_Update(&shooter_ctx);
        const Shooter_Output_t *shooter_output = Shooter_Get_Output();
        if (shooter_output->dial_mode_cmd)
        {
            Dial_Send_Mode_Command(&hfdcan1, shooter_output->dial_mode_cmd);
        }

        //CAN 统一发送

        //云台: Yaw(瓴控) + Pitch(大疆)
        const Gimbal_Output_t *gimbal_output = Gimbal_Get_Output();
        LK_Send_Torque(&hfdcan1, gimbal_output->yaw_current);

        DJI_Motor_Send_Command(&hfdcan3, 0x1FF, gimbal_output->pitch_current, 0, 0, 0);

        //发射: 摩擦轮(二级+一级) + 拨盘速度
        DJI_Motor_Send_Command(&hfdcan2, 0x200, shooter_output->friction_current_outer_left, shooter_output->friction_current_outer_right, shooter_output->friction_current_outer_up, 0);
        DJI_Motor_Send_Command(&hfdcan2, 0x1FF, shooter_output->friction_current_inner_left, shooter_output->friction_current_inner_right, shooter_output->friction_current_inner_up, 0);

        Dial_Send_Speed(&hfdcan1, shooter_output->dial_target_speed);

        //双板指令(含摩擦轮状态上报)
        const Chassis_Output_t *chassis_output = Chassis_Get_Output();
        Send_Chassis_Command(&hfdcan1, chassis_output->vx, chassis_output->vy, chassis_output->wz, chassis_output->mode, chassis_output->supercap, chassis_output->climb, chassis_output->led, chassis_output->shift, shooter_output->friction_report, is_base_locked, chassis_output->battle_mode);

        //底盘UI(50ms 周期): 云台角度 + 摩擦轮转速
        static uint32_t ui_send_cnt = 0;
        if (++ui_send_cnt >= 50)
        {
            ui_send_cnt = 0;
            Send_Chassis_UI_Data(&hfdcan1, gimbal_output->yaw_degree, gimbal_output->pitch_degree, shooter_output->friction_target_speed);
        }

        last_robot_mode = robot_mode;
}
