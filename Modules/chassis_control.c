/**
 * @file chassis_control.c
 * @brief 底盘控制模块实现
 */

#include "chassis_control.h"
#include "main.h"               // PI
#include "pid.h"
#include "Ramp.h"
#include "shooter_control.h"    // 裁判数据注入
#include "bsp_DR16.h"           // DR16_SW_UP/MID/DOWN
#include <math.h>
#include <string.h>

#define ABS(x) ((x) > 0 ? (x) : -(x))

//速度上限
static const float Max_x      = 80.0f;
static const float Max_y      = 80.0f;
static const float Max_z      = 80.0f;
static const float Max_Top_z  = 163.0f;
static const float KEY_Top_z  = 163.0f;

//底盘跟随PID参数
static float chassis_follow_kp = 0.2f;//350.0f
static float chassis_follow_ki = 0.0f;
static float chassis_follow_kd = 0.0f;
static float chassis_follow_Integral_Limit = 300.0f;
static float chassis_follow_MaxOutput = 100.0f;//最大Wz指令值     //50.0f
static PID_Controller chassis_follow_pid;

//底盘跟随
static const float kf  = 0.07f;//0.5
static const float Min = 150.0f;//150

//斜坡滤波
static const float Vx_RAMP_K_ACCEL = 200.0f;
static const float Vx_RAMP_K_DECEL = 1000.0f;
static const float Vy_RAMP_K_ACCEL = 800.0f;
static const float Vy_RAMP_K_DECEL = 1000.0f;
static const float Wz_RAMP_K_ACCEL = 300.0f;
static const float Wz_RAMP_K_DECEL = 500.0f;

static const float CLIMB_RAMP_K_ACCEL = 200.0f;
static const float CLIMB_RAMP_K_DECEL = 1000.0f;
static const float CLIMB_MAX_SPEED = 60.0f;

static const float climb_target_vx = 60.0f;

//Yaw编码器偏差转换
static const float YAW_ENCODER_DIFF_TO_DEG = 360.0f / 65536.0f;
static const float YAW_ENCODER_DIFF_TO_RAD = 2.0f * PI / 65536.0f;

//输出快照
static Chassis_Output_t output;

//上坎状态
static Climb_Mode_e climb_mode = CLIMB_MODE_OFF;
static uint8_t keyboard_climb_cmd = 0;
static uint8_t climb_cmd = 0;

//战斗模式内部状态(斜坡滤波器/底盘锁定)
static float chassis_vx_f = 0.0f;
static float chassis_vy_f = 0.0f;
static float chassis_wz_f = 0.0f;
static bool is_chassis_locked = false;  //底盘是否锁死
static bool key_Z_last_state = false;

//双板反馈
static volatile uint8_t robot_color = 2;      //颜色 (0=蓝, 1=红)

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

//初始化底盘跟随PID与输出快照
void Chassis_Control_Init(void)
{
    PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, chassis_follow_Integral_Limit, chassis_follow_MaxOutput);

    memset(&output, 0, sizeof(output));
    output.led = 1;
    climb_mode = CLIMB_MODE_OFF;
}

//机器人重新使能: 恢复PID限幅
void Chassis_On_Robot_Start(void)
{
    PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, chassis_follow_Integral_Limit, chassis_follow_MaxOutput);
}

//机器人失能: PID限幅清零并停机
void Chassis_On_Robot_Off(void)
{
    PID_Init(&chassis_follow_pid, chassis_follow_kp, chassis_follow_ki, chassis_follow_kd, 0, 0);

    output.vx = 0; output.vy = 0; output.wz = 0;
    output.mode = 0;  output.supercap = 0;
}

//上坎状态机与上坎/LED指令更新
void Chassis_Update_Climb(const RC_Data_t* rc, Robot_Mode_e robot_mode, bool e_key_just_pressed)
{
    //小陀螺标志每个控制周期复位
    output.shift = 0;

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
            output.led = 0;
        }
        else
        {
            keyboard_climb_cmd = 0;//全收
            output.led = 1;
        }
        climb_cmd = keyboard_climb_cmd;
    }
    else
    {
        //上坎模式切换
        keyboard_climb_cmd = 0;
        climb_cmd = 0;
        output.led = 1;

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
            output.led = 0;
            if(rc->rc.s_right == DR16_SW_UP)        climb_cmd = 2;//前收后伸
            else if (rc->rc.s_right == DR16_SW_MID) climb_cmd = 1;//前伸
            else climb_cmd = 0;//全收
        }
    }
}

//战斗模式(键鼠)底盘计算
void Chassis_Update_Battle(const RC_Data_t* rc, float yaw_encoder_diff, uint8_t is_aiming_now, uint8_t is_base_shooting)
{
    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_wz = 0.0f;

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

        output.battle_mode = BATTLE_LOB;
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

            output.battle_mode = BATTLE_GIMBAL;
        }
        else
        {
            output.battle_mode = BATTLE_FOLLOW;

            if(rc->key.map.W && !rc->key.map.G && !rc->key.map.F && !rc->key.map.R) target_vx = Max_x;
            else if(rc->key.map.S && !rc->key.map.G && !rc->key.map.F && !rc->key.map.R) target_vx = -Max_x;

            if(rc->key.map.A) target_vy = -Max_y;
            else if (rc->key.map.D) target_vy = Max_y;

            if(rc->key.map.W && (rc->key.map.A || rc->key.map.D))
            {
                target_vx *= 0.6f;
                target_vy *= 1.3f;
            }

            if(rc->key.map.SHIFT) //小陀螺
            {
                target_wz = KEY_Top_z;
                output.shift = 1;

                target_vx *= 1.1f;
                target_vy *= 1.7f;
            }
            else//底盘跟随
            {
                if(is_aiming_now == 1 && yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD < 45.0f)
                {
                    //自瞄且偏差较小时不自旋(保持 target_wz = 0)
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
    }

    //坐标系转换
    float yaw_rad_diff = -yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD;
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

    output.vx = (int16_t)(chassis_vx_f * 100.0f);
    output.vy = (int16_t)(chassis_vy_f * 100.0f);
    output.wz = (int16_t)(chassis_wz_f * 100.0f);
    output.mode = 1;
    output.supercap = rc->request_super_cap ? 1 : 0;
}

//常规底盘移动计算
void Chassis_Update_Move(const RC_Data_t* rc, float yaw_encoder_diff, Chassis_Move_Style_e style)
{
    //坐标系转换
    float yaw_rad_diff = -yaw_encoder_diff * YAW_ENCODER_DIFF_TO_RAD;
    float cos_yaw_diff = cosf(yaw_rad_diff);
    float sin_yaw_diff = sinf(yaw_rad_diff);

    float raw_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
    float raw_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;

    float final_vx = raw_vx * cos_yaw_diff + raw_vy * sin_yaw_diff;
    float final_vy = -raw_vx * sin_yaw_diff + raw_vy * cos_yaw_diff;

    output.vx = (int16_t)(final_vx * 100.0f);
    output.vy = (int16_t)(final_vy * 100.0f);
    output.mode = 1;

    switch(style)
    {
        case CHASSIS_MOVE_GIMBAL_ONLY:
            //不改自旋, 不请求超级电容
            output.supercap = 0;
            break;

        case CHASSIS_MOVE_FOLLOW:
            if(ABS(rc->rc.wheel) > 2 && rc->rc.s_left == DR16_SW_MID) //小陀螺
            {
                output.wz  = (int16_t)(rc->rc.wheel * RC_TO_ONE  * Max_Top_z * 100.0f);//放大发送保留精度，底盘接收后需要/100
            }
            else//底盘跟随
            {
                float follow_Wz = yaw_encoder_diff * -kf;

                float calc_Wz = PID_Calculate(&chassis_follow_pid, 0.0f, follow_Wz);

                output.wz = (int16_t)(calc_Wz * 100.0f); // 放大发送

                if(ABS(output.wz) < Min)
                {
                    output.wz = 0;
                }
            }
            output.supercap = rc->request_super_cap ? 1 : 0;
            break;

        case CHASSIS_MOVE_AIMING:
            if(ABS(rc->rc.wheel) > 2 )//&& rc->rc.s_left == RC_SW_MID) //小陀螺
            {
                output.wz = (int16_t)(rc->rc.wheel * RC_TO_ONE  * Max_Top_z * 1.5f * 100.0f);//放大发送保留精度，底盘接收后需要/100
            }
            else
            {
                output.wz = 0;
            }
            output.supercap = rc->request_super_cap ? 1 : 0;
            break;

        case CHASSIS_MOVE_M_AIMING:
            //不改自旋
            output.supercap = rc->request_super_cap ? 1 : 0;
            break;
    }
}

//裸底盘计算(不做坐标变换)
void Chassis_Set_Direct(const RC_Data_t* rc, Chassis_Direct_Style_e style)
{
    float chassis_vx = (float)rc->rc.left_y * RC_TO_ONE * Max_x;
    float chassis_vy = (float)rc->rc.left_x * RC_TO_ONE * Max_y;
    float chassis_wz = (float)rc->rc.right_x * RC_TO_ONE * Max_z;

    output.vx = (int16_t)(chassis_vx * 100.0f);
    output.vy = (int16_t)(chassis_vy * 100.0f);
    output.wz = (int16_t)(chassis_wz * 100.0f);
    output.mode = 1;

    if(style == CHASSIS_DIRECT_AUTO_SUPERCAP)
    {
        if(ABS(rc->rc.left_y) > 600) output.supercap = 1;
        else output.supercap = 0;
    }
    else
    {
        output.supercap = 0;
    }
}

//底盘指令清零(发射模式等不动底盘的场景)
void Chassis_Stop_Movement(void)
{
    output.vx = 0; output.vy = 0; output.wz = 0;
    output.mode = 0; output.supercap = 0;
}

//上坎时清零横移/自旋(主循环 climb_cmd 后处理)
void Chassis_Zero_Turn(void)
{
    output.vy = 0;
    output.wz = 0;
}

//战斗子状态写入
void Chassis_Set_Battle_Mode(Battle_Mode_e mode)
{
    output.battle_mode = (uint8_t)mode;
}

//上坎状态/指令查询
Climb_Mode_e Chassis_Get_Climb_Mode(void)
{
    return climb_mode;
}

uint8_t Chassis_Get_Climb_Cmd(void)
{
    return climb_cmd;
}

//获取输出快照(零拷贝只读)
const Chassis_Output_t* Chassis_Get_Output(void)
{
    return &output;
}

// 更新底盘发来的信息
void Chassis_Decode_Feedback(const uint8_t *data)
{
    Chassis_To_Gimbal_Msg_t rx_msg;

    memcpy(rx_msg.data, data, 8);

    Shooter_Set_Referee_Data(rx_msg.pack.shooter_heat, rx_msg.pack.heat_limit, rx_msg.pack.cooling_value, rx_msg.pack.bullet_speed);
    robot_color        = rx_msg.pack.color;
}

//己方颜色
uint8_t Chassis_Get_Color(void)
{
    return robot_color;
}
