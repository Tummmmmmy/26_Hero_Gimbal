/**
 * @file shooter_control.c
 * @brief 射击控制模块：摩擦轮、拨盘、弹速统计与热量管理
 *
 * 由 main.c 的比赛验证逻辑整理而来；发射机构状态与电机对象均已收归本模块私有。
 */

#include "main.h"
#include "cmsis_os.h"
#include "pid_controller.h"
#include "shooter_control.h"
#include <math.h>
#include <string.h>

// 射击模块私有状态（全部 static，不对外暴露内部实现）
// ===== 发射机构私有类型 =====
typedef enum
{
    SHOOTER_OFF,
    SHOOTER_CALIBRATING,
    SHOOTER_READY,
    SHOOTER_SINGLE_SHOT,
    SHOOTER_JAMMED
} Shooter_State_e;

typedef enum
{
    SHOT_NOT_DETECTED = 0,
    SHOT_DETECTED = 1
} Shot_Detection_e;

typedef struct
{
    Shooter_State_e state;
    float friction_target_speed;
    float friction_linear_ratio;
    float dial_target_rad;
    float dial_last_normal_rad;

    uint8_t single_shot_fired_flag;
    uint16_t continuous_fire_timer;
    uint16_t jam_detect_timer;
    uint16_t jam_timeout_timer;
    uint8_t jam_process_step;
    uint8_t is_calibrated;
    uint8_t homing_step;
    uint16_t homing_timer;
} Shooter_Controller_t;

// ===== 发射机构私有状态与电机对象 =====
static Motor_Controller dial_motor;
static Motor_Controller friction_motor_inner_left;
static Motor_Controller friction_motor_inner_right;
static Motor_Controller friction_motor_inner_up;
static Motor_Controller friction_motor_outer_left;
static Motor_Controller friction_motor_outer_right;
static Motor_Controller friction_motor_outer_up;

static Shooter_Controller_t shooter_controller;
static Shot_Detection_e current_shot_status = SHOT_NOT_DETECTED;

static float ratio = 0.784f;
static float low_ratio = 0.56f;
static float dial_back = 0.445f;
static uint32_t high_speed = 4450.0f;
static uint32_t low_speed = 3210.0f;

static volatile uint16_t shooter_heat = 0;
static volatile uint16_t heat_limit = 100;
static volatile uint16_t shooter_cooling = 20;
static volatile float initial_speed = 0.0f;
static float shooter_predict_heat = 0.0f;
static uint32_t last_shoot_time = 0;

static float friction_speed_kp_inner_high = 4.0f;
static float friction_speed_ki_inner_high = 0.0f;
static float friction_speed_kd_inner_high = 0.0f;
static float friction_speed_kp_outer_high = 10.0f;
static float friction_speed_ki_outer_high = 0.0f;
static float friction_speed_kd_outer_high = 0.0f;
static float friction_speed_kp_inner_low = 8.0f;
static float friction_speed_ki_inner_low = 0.0f;
static float friction_speed_kd_inner_low = 0.0f;
static float friction_speed_kp_outer_low = 10.0f;
static float friction_speed_ki_outer_low = 0.0f;
static float friction_speed_kd_outer_low = 0.0f;
static const float friction_speed_Integral_Limit = 50.0f;
static const float friction_speed_MaxOutput = 16380.0f;
static float last_fric_kp_inner = 0.0f;

static const float dial_angle_kp = 12.0f;
static const float dial_angle_ki = 0.0f;
static const float dial_angle_kd = 0.0f;
static const float dial_angle_Integral_Limit = 1.0f;
static const float dial_angle_MaxOutput = 25.0f;
static const float dial_speed_kp = 0.3f;
static const float dial_speed_ki = 0.0f;
static const float dial_speed_kd = 0.0f;
static const float dial_speed_Integral_Limit = 2.0f;
static const float dial_speed_MaxOutput = 5.0f;

static uint32_t gap = 200;
static bool s_use_referee_heat_limit = false;
static bool s_robot_off = true;
static bool s_aiming_mode = false;
static bool s_battle_aiming = false;
static bool s_wheel_released = false;
static uint8_t s_dial_enable_pending = 0;
static uint8_t s_dial_disable_pending = 0;
static Shooter_Output_t s_output;
static Shooter_Debug_Data_t s_debug;

static const float RAD_TO_DEG = 180.0f / PI;

#define DM_DIAL_SEND_ID 0x07
#define DM_DIAL_RECEIVE_ID 0x17
#define RX_DIAL_FDB_ID 0x302

#define FRICTION_OUTER_LEFT_ID  1
#define FRICTION_OUTER_RIGHT_ID 2
#define FRICTION_OUTER_UP_ID    3
#define FRICTION_INNER_LEFT_ID  5
#define FRICTION_INNER_RIGHT_ID 6
#define FRICTION_INNER_UP_ID    7

#define DM4340_DIAL_T_MIN -12.0f
#define DM4340_DIAL_T_MAX 12.0f

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


// 与 main.c 保持一致，避免文件间宏重复
#define ABS(x) ((x) > 0 ? (x) : -(x))

// 摩擦轮目标转速与线速度比例
#define FRICTION_TARGET_SPEED_HIGH high_speed
#define FRICTION_RATIO_HIGH ratio
#define FRICTION_TARGET_SPEED_LOW  low_speed
#define FRICTION_RATIO_LOW        low_ratio

// 摩擦轮半径
#define WHEEL_RADIUS_INNER  35.0f // 内圈半径
#define WHEEL_RADIUS_OUTER  35.0f // 外圈半径

// 拨盘单发编码器增量
#define DIAL_ONE_SHOOT_ENCODER PI / 3.0f

// 卡弹检测阈值
#define DIAL_JAM_CURRENT_THRESHOLD 6.0f
#define DIAL_JAM_SPEED_THRESHOLD 1.5f
#define DIAL_JAM_DETECT_THRESHOLD 100
#define DIAL_JAM_WAIT_TIME 400

// 校准回中参数
#define DIAL_HOMING_SPEED -2.2f
#define DIAL_HOMING_CURRENT_THRESHOLD 1.0f
#define DIAL_HOMING_SPEED_THRESHOLD 2.05f
#define DIAL_HOMING_OFFSET dial_back
#define DIAL_HOMING_CHECK_DELAY 200

// 热量管理
#define HERO_HEAT_PER_SHOT 100.0f
#define HEAT_SAFETY_MARGIN 5.0f


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
static float print_initial_speed = 0.0f;       //要打印的弹速
static uint16_t vofa_timer = 0;      //持续计时器
static float bullet_speed_avg = 0.0f;       //平均弹速
static float bullet_speed_range = 0.0f;     //极差
static float bullet_speed_variance = 0.0f;  //方差
static uint32_t total_shot_count = 0;       //发射数


static float auto_speed_offset = 0.0f;     //最终摩擦轮补偿值

static float speed_window[MAX_WINDOW_SIZE] = {0};


static uint8_t high_speed_count = 0;    //连续超速计数器
static uint8_t low_speed_count = 0;     //连续低速计数器
static float last_referee_speed = 0.0f; //上一次的弹速
static uint32_t shoot_seq_cnt = 0; //记录一轮射击打到了第几发
static float pending_speed = 0.0f; //暂存上一发的弹速



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
void Shooter_Reset_Stats(void)
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
static void Bullet_Speed_Adaptive_Control(float latest_speed)
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





static uint32_t last_bullet_update_tick = 0;
static void Bullet_Speed_Adaptive_Process(void)
{
    if(initial_speed != last_referee_speed && initial_speed > 11.0f && initial_speed < 17.0f)
    {
			 shoot_seq_cnt++;
			
			 last_bullet_update_tick = osKernelGetTickCount(); 
			
        if(shoot_seq_cnt > 2) 
        {
            if(!s_robot_off)
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
    
    if(s_robot_off)
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
                s_dial_enable_pending = 1;
                
                dial_was_offline = false;
                dial_recover_step = 0;
            }
        }
    }
}



#define USE_AUTO_CALIBRATION 1 //1:开启撞限位校准, 0:关闭校准(当前位置为零点)

//发射控制
static void Shooter_Update_Heat(void);
static void Shooter_Check_Real_Fire(void);
static uint8_t Shooter_Check_Heat_Allowed(void);

void Shooter_Control_Update(const Shooter_Cmd_t *cmd)
{
	  //获取时间
    uint32_t now = xTaskGetTickCount();
    uint8_t fire_command = cmd->fire_command;
    s_use_referee_heat_limit = cmd->use_referee_heat_limit;
	
	  //更新热量
    Shooter_Update_Heat();
	
	  //检测是否发弹
    Shooter_Check_Real_Fire();
	
	  //拨盘电机重新上电处理
	  Dial_Recover_Process();
	
	  //左键边缘检测
	      // ===== 根据主调度器翻译后的指令确定摩擦轮目标转速与参数 =====
    float friction_target_speed = 0.0f;
    float friction_ratio = 0.85f;
    float friction_offs = 0.0f;
    float fric_left_kp_offs = -0.5f;
    float fric_right_kp_offs = 0.1f;

    float cur_fric_kp_inner = friction_speed_kp_inner_high;
    float cur_fric_ki_inner = friction_speed_ki_inner_high;
    float cur_fric_kd_inner = friction_speed_kd_inner_high;

    float cur_fric_kp_outer = friction_speed_kp_outer_high;
    float cur_fric_ki_outer = friction_speed_ki_outer_high;
    float cur_fric_kd_outer = friction_speed_kd_outer_high;

    if (cmd->fric_mode == SHOOTER_MODE_LOW_SPEED)
    {
        friction_target_speed = FRICTION_TARGET_SPEED_LOW;
        friction_ratio = FRICTION_RATIO_LOW;
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
    else if (cmd->fric_mode == SHOOTER_MODE_HIGH_SPEED)
    {
        friction_target_speed = FRICTION_TARGET_SPEED_HIGH;
        friction_ratio = FRICTION_RATIO_HIGH;
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
        if(cmd->fric_mode != SHOOTER_MODE_OFF)
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
					
					
					
				  	if(s_aiming_mode)
						{
							  gap = 5000;
						}
            else if (s_battle_aiming)
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
								else if((s_aiming_mode && !fire_command) || (!s_aiming_mode && s_wheel_released))
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
		// 摩擦轮是否已转起来（供主循环上报底盘档位时使用）
		s_output.friction_ready = (Check_Friction_Ready() == 1 && shooter_controller.friction_target_speed > 200.0f) ? 1 : 0;
 
		
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

    // 输出快照，主调度器统一执行 CAN 发送
    s_output.dial_target_speed = dial_motor.target_speed;
    s_output.friction_current_inner_left = friction_motor_inner_left.target_current;
    s_output.friction_current_inner_right = friction_motor_inner_right.target_current;
    s_output.friction_current_inner_up = friction_motor_inner_up.target_current;
    s_output.friction_current_outer_left = friction_motor_outer_left.target_current;
    s_output.friction_current_outer_right = friction_motor_outer_right.target_current;
    s_output.friction_current_outer_up = friction_motor_outer_up.target_current;
    s_output.friction_target_speed = shooter_controller.friction_target_speed;

    // 拨盘使能/失能指令（0=无需发送，0xFC=使能，0xFD=失能）
    if (s_dial_disable_pending)
    {
        s_output.dial_mode_cmd = 0xFD;
        s_dial_disable_pending = 0;
    }
    else if (s_dial_enable_pending)
    {
        s_output.dial_mode_cmd = 0xFC;
        s_dial_enable_pending = 0;
    }
    // 机器人未失能且拨盘尚未使能时，持续补发使能帧
    else if (!s_robot_off && shooter_controller.state != SHOOTER_OFF && dial_motor.state != 1)
    {
        s_output.dial_mode_cmd = 0xFC;
    }
    else
    {
        s_output.dial_mode_cmd = 0;
    }
}




//检测是否发射弹丸
static void Shooter_Check_Real_Fire(void) 
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
static void Shooter_Update_Heat(void)
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
		if(s_use_referee_heat_limit) 
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
static uint8_t Shooter_Check_Heat_Allowed(void)
{
	if(s_use_referee_heat_limit) 
	{
		  return(shooter_predict_heat + HERO_HEAT_PER_SHOT <= (float)heat_limit);
	}
	else
	{
      return(shooter_predict_heat + HERO_HEAT_PER_SHOT < (float)10000.0f - HEAT_SAFETY_MARGIN);
	}
}


// ===== 对外 API =====

static void Shooter_Motor_Init(Motor_Controller* motor, uint16_t id, FDCAN_HandleTypeDef* hfdcan)
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

void Shooter_Control_Init(void)
{
    Shooter_Motor_Init(&dial_motor, DM_DIAL_SEND_ID, &hfdcan1);
    PID_Init(&dial_motor.pid_angle, dial_angle_kp, dial_angle_ki, dial_angle_kd, dial_angle_Integral_Limit, dial_angle_MaxOutput);
    PID_Init(&dial_motor.pid_speed, dial_speed_kp, dial_speed_ki, dial_speed_kd, dial_speed_Integral_Limit, dial_speed_MaxOutput);

    Shooter_Motor_Init(&friction_motor_inner_left,  FRICTION_INNER_LEFT_ID,  &hfdcan2);
    Shooter_Motor_Init(&friction_motor_inner_right, FRICTION_INNER_RIGHT_ID, &hfdcan2);
    Shooter_Motor_Init(&friction_motor_inner_up,    FRICTION_INNER_UP_ID,    &hfdcan2);
    Shooter_Motor_Init(&friction_motor_outer_left,  FRICTION_OUTER_LEFT_ID,  &hfdcan2);
    Shooter_Motor_Init(&friction_motor_outer_right, FRICTION_OUTER_RIGHT_ID, &hfdcan2);
    Shooter_Motor_Init(&friction_motor_outer_up,    FRICTION_OUTER_UP_ID,    &hfdcan2);

    PID_Init(&friction_motor_inner_left.pid_speed,  friction_speed_kp_inner_high, friction_speed_ki_inner_high, friction_speed_kd_inner_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_inner_right.pid_speed, friction_speed_kp_inner_high, friction_speed_ki_inner_high, friction_speed_kd_inner_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_inner_up.pid_speed,    friction_speed_kp_inner_high, friction_speed_ki_inner_high, friction_speed_kd_inner_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_outer_left.pid_speed,  friction_speed_kp_outer_high, friction_speed_ki_outer_high, friction_speed_kd_outer_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_outer_right.pid_speed, friction_speed_kp_outer_high, friction_speed_ki_outer_high, friction_speed_kd_outer_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);
    PID_Init(&friction_motor_outer_up.pid_speed,    friction_speed_kp_outer_high, friction_speed_ki_outer_high, friction_speed_kd_outer_high, friction_speed_Integral_Limit, friction_speed_MaxOutput);

    memset(&shooter_controller, 0, sizeof(shooter_controller));
    shooter_controller.state = SHOOTER_OFF;
    shooter_controller.is_calibrated = 0;
    memset(&s_output, 0, sizeof(s_output));
    memset(&s_debug, 0, sizeof(s_debug));
    s_dial_enable_pending = 0;
    s_dial_disable_pending = 0;
}

void Shooter_Control_Disable(void)
{
    shooter_controller.state = SHOOTER_OFF;
    shooter_controller.single_shot_fired_flag = 0;
    shooter_controller.jam_detect_timer = 0;
    shooter_controller.jam_timeout_timer = 0;
    shooter_controller.jam_process_step = 0;
}

void Shooter_On_Robot_Start(void)
{
    PID_Init(&dial_motor.pid_angle, dial_angle_kp, dial_angle_ki, dial_angle_kd, dial_angle_Integral_Limit, dial_angle_MaxOutput);
    PID_Init(&dial_motor.pid_speed, dial_speed_kp, dial_speed_ki, dial_speed_kd, dial_speed_Integral_Limit, dial_speed_MaxOutput);
    last_fric_kp_inner = 0.0f;
    s_dial_enable_pending = 1;
}

void Shooter_On_Robot_Off(void)
{
    Shooter_Control_Disable();
    PID_Init(&dial_motor.pid_angle, dial_angle_kp, dial_angle_ki, dial_angle_kd, 0, 0);
    PID_Init(&dial_motor.pid_speed, dial_speed_kp, dial_speed_ki, dial_speed_kd, 0, 0);
    dial_motor.target_torque = 0.0f;
    s_dial_disable_pending = 1;
}

const Shooter_Output_t* Shooter_Get_Output(void)
{
    return &s_output;
}

void Shooter_Set_Referee_Data(uint16_t heat, uint16_t limit, uint16_t cooling, float speed)
{
    shooter_heat = heat;
    heat_limit = limit;
    shooter_cooling = cooling;
    initial_speed = speed;
}

void Shooter_Set_System_State(bool robot_off, bool aiming_mode, bool battle_aiming, bool wheel_released)
{
    s_robot_off = robot_off;
    s_aiming_mode = aiming_mode;
    s_battle_aiming = battle_aiming;
    s_wheel_released = wheel_released;
}

static void Shooter_Update_Dial_Motor(const uint8_t* data)
{
    dial_motor.id = (data[0]) & 0x0F;
    dial_motor.state = (data[0]) >> 4;

    uint16_t pos_int = (data[1] << 8) | data[2];
    uint16_t vel_int = (data[3] << 4) | (data[4] >> 4);
    uint16_t tor_int = ((data[4] & 0x0F) << 8) | data[5];

    float current_pos_rad = uint_to_float(pos_int, -3.141593f, 3.141593f, 16);

    if (dial_motor.last_feedback_position == 0.0f && dial_motor.round_count == 0)
    {
        dial_motor.last_feedback_position = current_pos_rad;
    }

    float pos_diff = current_pos_rad - dial_motor.last_feedback_position;
    if (pos_diff > 3.14f)
    {
        dial_motor.round_count--;
    }
    else if (pos_diff < -3.14f)
    {
        dial_motor.round_count++;
    }

    dial_motor.current_total_angle_rad = dial_motor.round_count * (6.283186f) + current_pos_rad;
    dial_motor.current_total_angle_deg = dial_motor.current_total_angle_rad * RAD_TO_DEG;

    dial_motor.feedback.position = current_pos_rad;
    dial_motor.feedback.velocity = uint_to_float(vel_int, -10, 10, 12);
    dial_motor.feedback.torque = uint_to_float(tor_int, DM4340_DIAL_T_MIN, DM4340_DIAL_T_MAX, 12);
    dial_motor.last_feedback_position = current_pos_rad;
    dial_motor.last_update_tick = osKernelGetTickCount();
}

static void Shooter_Update_Friction_Motors(const uint8_t* data, uint32_t can_id)
{
    Motor_Controller* motor = NULL;
    switch (can_id - 0x200)
    {
        case FRICTION_INNER_LEFT_ID:  motor = &friction_motor_inner_left;  break;
        case FRICTION_INNER_RIGHT_ID: motor = &friction_motor_inner_right; break;
        case FRICTION_INNER_UP_ID:    motor = &friction_motor_inner_up;    break;
        case FRICTION_OUTER_LEFT_ID:  motor = &friction_motor_outer_left;  break;
        case FRICTION_OUTER_RIGHT_ID: motor = &friction_motor_outer_right; break;
        case FRICTION_OUTER_UP_ID:    motor = &friction_motor_outer_up;    break;
        default: return;
    }

    motor->feedback.angle = (uint16_t)(data[0] << 8) | data[1];
    motor->feedback.actual_speed = (int16_t)((data[2] << 8) | data[3]);
    motor->feedback.current = (int16_t)((data[4] << 8) | data[5]);
    motor->feedback.temperature = data[6];
    motor->last_update_tick = osKernelGetTickCount();
}

void Shooter_CAN_Feedback(uint32_t can_id, const uint8_t* data)
{
    if (can_id == RX_DIAL_FDB_ID)
    {
        Shooter_Update_Dial_Motor(data);
    }
    else if (can_id >= 0x201 && can_id <= 0x207)
    {
        Shooter_Update_Friction_Motors(data, can_id);
    }
}

static void Shooter_Check_Motor_Online(Motor_Controller* motor, uint32_t current_tick)
{
    if (current_tick - motor->last_update_tick > 200)
    {
        motor->is_online = false;
    }
    else
    {
        motor->is_online = true;
    }
}

void Shooter_Check_Motors_Online(uint32_t current_tick)
{
    Shooter_Check_Motor_Online(&dial_motor, current_tick);
    Shooter_Check_Motor_Online(&friction_motor_inner_left, current_tick);
    Shooter_Check_Motor_Online(&friction_motor_inner_right, current_tick);
    Shooter_Check_Motor_Online(&friction_motor_inner_up, current_tick);
    Shooter_Check_Motor_Online(&friction_motor_outer_left, current_tick);
    Shooter_Check_Motor_Online(&friction_motor_outer_right, current_tick);
    Shooter_Check_Motor_Online(&friction_motor_outer_up, current_tick);
}

void Shooter_Debug_Tick(void)
{
    if (vofa_timer > 0) vofa_timer--;
    else print_initial_speed = 0.0f;
}

const Shooter_Debug_Data_t* Shooter_Get_Debug_Data(void)
{
    s_debug.referee_bullet_speed = initial_speed;
    s_debug.print_initial_speed = print_initial_speed;
    s_debug.bullet_speed_avg = bullet_speed_avg;
    s_debug.bullet_speed_variance = bullet_speed_variance;
    s_debug.bullet_speed_range = bullet_speed_range;
    s_debug.friction_outer_left_speed = friction_motor_outer_left.feedback.actual_speed;
    s_debug.friction_outer_right_speed = friction_motor_outer_right.feedback.actual_speed;
    s_debug.friction_outer_up_speed = friction_motor_outer_up.feedback.actual_speed;
    s_debug.friction_inner_left_speed = friction_motor_inner_left.feedback.actual_speed;
    s_debug.friction_inner_right_speed = friction_motor_inner_right.feedback.actual_speed;
    s_debug.friction_inner_up_speed = friction_motor_inner_up.feedback.actual_speed;
    return &s_debug;
}

void Shooter_Tune_High_Speed(int32_t delta)
{
    high_speed += delta;
}

void Shooter_Tune_Low_Speed(int32_t delta)
{
    low_speed += delta;
}
