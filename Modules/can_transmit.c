/**
 * @file can_transmit.c
 * @brief CAN 发送协议层
 *
 * 负责把控制量打包成具体电机/设备协议并发送。
 * 底层 FDCAN 硬件收发由 BSP/bsp_can 提供。
 */
#include "can_transmit.h"
#include "bsp_can.h"

#define LK_CURRENT_TO_RAW  (2048.0f / 16.5f)  // Amps -> -2048~2048

#define DM_P_MIN -12.5f
#define DM_P_MAX 12.5f
#define DM_V_MIN -45.0f
#define DM_V_MAX 45.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

#define TX_CHASSIS_COMMAND_ID 0x300
#define TX_CHASSIS_UI_ID 0x308

#define TX_DIAL_CMD_ID    0x303
#define TX_DIAL_MODE_ID   0x301

#define LIMIT_16BIT(val)  ((val) > 32767 ? 32767 : ((val) < -32768 ? -32768 : (val)))

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



/**
  * @brief  大疆电机控制指令发送
  * @param  hfdcan: FDCAN句柄指针
  * @param  std_id: 目标控制CAN ID
  * @param  current: 对应4个电机的目标电流
  */
//大疆电机控制指令
void DJI_Motor_Send_Command(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    uint8_t tx_data[8];


    tx_data[0] = (uint8_t)(current1 >> 8);
    tx_data[1] = (uint8_t)(current1 & 0xFF);
    tx_data[2] = (uint8_t)(current2 >> 8);
    tx_data[3] = (uint8_t)(current2 & 0xFF);
    tx_data[4] = (uint8_t)(current3 >> 8);
    tx_data[5] = (uint8_t)(current3 & 0xFF);
    tx_data[6] = (uint8_t)(current4 >> 8);
    tx_data[7] = (uint8_t)(current4 & 0xFF);

    //直接写入硬件 FIFO (检查是否有空位)
    BSP_CAN_Send(hfdcan, std_id, tx_data, FDCAN_DLC_BYTES_8);
}


// 备用：暂未调用
//发送LK电机开启命令
void LK_Motor_Enable(FDCAN_HandleTypeDef* hfdcan)
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = 0x88;//电机运行
    //其他字节全是0x00


    //发送
    BSP_CAN_Send(hfdcan, LK_YAW_ID, tx_data, FDCAN_DLC_BYTES_8);
}



/**
  * @brief  瓴控电机转矩控制发送
  * @param  hfdcan: FDCAN句柄指针
  * @param  id: 电机实际ID (1, 2)
  * @param  current_amps: 目标电流值 (安培)
  */
void LK_Send_Torque(FDCAN_HandleTypeDef* hfdcan, float current_amps)
{
    uint8_t tx_data[8];
    
    //限制电流幅度
    if(current_amps > 16.5f) current_amps = 16.5f;
    if(current_amps < -16.5f) current_amps = -16.5f;

    //转换为 -2048 ~ 2048 的整数
    int16_t send_current = (int16_t)(current_amps * LK_CURRENT_TO_RAW);


    tx_data[0] = 0xA1; //转矩控制命令
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = (uint8_t)(send_current & 0xFF);
    tx_data[5] = (uint8_t)((send_current >> 8) & 0xFF);
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;

    //直接写入硬件 FIFO (检查是否有空位)
    BSP_CAN_Send(hfdcan, LK_YAW_ID, tx_data, FDCAN_DLC_BYTES_8); //数据长度 8字节
}


// 备用：暂未调用
void LK_Send_Query(FDCAN_HandleTypeDef* hfdcan)
{
    uint8_t tx_data[8];
    

    tx_data[0] = 0x9C;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;

    //直接写入硬件 FIFO (检查是否有空位)
    BSP_CAN_Send(hfdcan, LK_YAW_ID, tx_data, FDCAN_DLC_BYTES_8); //数据长度 8字节
}


// 备用：暂未调用
void DM_Motor_Send_MIT_Command(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos_rad, float vel_rad_s, float kp, float kd, float torque_nm, float t_max)
{
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


    //直接写入硬件 FIFO (检查是否有空位)
    BSP_CAN_Send(hfdcan, motor_id, tx_data, FDCAN_DLC_BYTES_8);
}



//将达妙速度控制指令发送给底盘
void Dial_Send_Speed(FDCAN_HandleTypeDef *hfdcan, float target_vel)
{
    uint8_t tx_data[4];

    uint8_t *p_vel = (uint8_t *)&target_vel;
    tx_data[0] = p_vel[0];
    tx_data[1] = p_vel[1];
    tx_data[2] = p_vel[2];
    tx_data[3] = p_vel[3];


    BSP_CAN_Send(hfdcan, TX_DIAL_CMD_ID, tx_data, FDCAN_DLC_BYTES_4);
}



void Dial_Send_Mode_Command(FDCAN_HandleTypeDef *hfdcan, uint8_t command)
{
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};


    BSP_CAN_Send(hfdcan, TX_DIAL_MODE_ID, tx_data, FDCAN_DLC_BYTES_8);
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


void Send_Chassis_Command(FDCAN_HandleTypeDef* hfdcan, int16_t vx, int16_t vy, int16_t wz, uint8_t mode, uint8_t supercap_req, uint8_t climb_cmd, uint8_t led_cmd , uint8_t key_shift ,uint8_t friction_state, uint8_t is_base_locked, uint8_t battle_mode)
{
    Gimbal_To_Chassis_Msg_t tx_msg = {0};


		
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
    BSP_CAN_Send(hfdcan, TX_CHASSIS_COMMAND_ID, tx_msg.data, FDCAN_DLC_BYTES_8);     //长度8字节
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
void Send_Chassis_UI_Data(FDCAN_HandleTypeDef* hfdcan, float yaw_deg, float pitch_deg, float friction_speed)
{
    Gimbal_UI_Msg_t ui_msg = {0}; 


    int32_t yaw_deg_x100 = (int16_t)(yaw_deg * 100.0f);
    int32_t pit_deg_x100 = (int16_t)(pitch_deg * 100.0f);
		LIMIT_MIN_MAX(yaw_deg_x100, 0, 35999);
		LIMIT_MIN_MAX(pit_deg_x100, -18000, 18000);
    ui_msg.pack.yaw_angle_x100   = (uint16_t)yaw_deg_x100;
    ui_msg.pack.pitch_angle_x100 = pit_deg_x100;
		
		
		ui_msg.pack.friction_speed = (friction_speed > 8191) ? 8191 : friction_speed;

    //直接写入硬件发送 FIFO(检查是否有空位)
    BSP_CAN_Send(hfdcan, TX_CHASSIS_UI_ID, ui_msg.data, FDCAN_DLC_BYTES_8);     //长度8字节
}
