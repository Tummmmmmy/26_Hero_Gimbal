/**
 * @file vision_link.h
 * @brief 视觉通信接口与报文结构定义
 */

#ifndef VISION_LINK_H
#define VISION_LINK_H

#include "main.h"


//视觉判定超时阈值
#define VISION_TIMEOUT_MS   30u 


//强制1字节对齐，确保与C++的内存布局一致
#pragma pack(1) 

/** @brief 视觉接收结构体 */
typedef struct
{
    uint8_t head[2];    // 帧头 'S', 'P'
    uint8_t mode;       // 0:空闲 1:自瞄不开火 2:自瞄开火
    float yaw;          // 目标yaw (弧度)
    float yaw_vel;
    float yaw_acc;
    float pitch;        // 目标pitch (弧度)
    float pitch_vel;
    float pitch_acc;

	  float yaw_base_hit;
    float pitch_base_hit;

    uint16_t crc16;
} Vision_Recv_t;


/** @brief 视觉发送结构体 */
typedef struct
{
    uint8_t head[2];    //帧头 'S', 'P'
    uint8_t mode;       
    float q[4];         //四元数 [w, x, y, z]
    float yaw;          
    float yaw_vel;      
    float pitch;        
    float pitch_vel;    
    float bullet_speed; 
    uint16_t bullet_count;
	  uint8_t color;       //颜色：0: 红, 1: 蓝
    uint16_t crc16;     
} Vision_Send_t;

#pragma pack() //取消强制对齐


typedef struct
{
    //接收到的目标数据
    float target_yaw_rad;     // 目标Yaw (弧度)
    float target_pitch_rad;   // 目标Pitch (弧度)
    float target_yaw_acc;     // 目标Yaw加速度 (rad/s^2)
    float target_pitch_acc;   // 目标Pitch加速度 (rad/s^2)

	  float target_yaw_base_hit;
    float target_pitch_base_hit;

    uint8_t cmd_mode;         // 视觉下达的模式

    //状态标志
    uint8_t is_tracking;        // 是否有目标
    uint32_t last_update_time;  // 上次更新时间

} Vision_Data_t;


// 全局变量声明
extern Vision_Data_t VisionData;


/** @brief 视觉通信初始化 */
void Vision_Init(void);

/** @brief 判断视觉是否在线 */
bool Vision_IsOnline(void);

/** @brief 校验并解析视觉报文 */
void Vision_Process_Data(uint8_t *buf, uint32_t len); 

/** @brief 打包并发送数据给视觉 */
void Vision_Send_Data(void); 


/** @brief 清除目标数据 */
void Clear_Vision_Target(void);

/** @brief 异步非阻塞 USB 重启状态机 */
void Vision_Force_Reconnect_Start(void);

/** @brief USB异步重启状态机 */
void Vision_Reconnect_Task_Async(void);


#endif
