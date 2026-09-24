/**
 * @file Vision_can.h
 * @brief [废弃] 早期 CAN 视觉通信协议定义
 *
 * 已被 vision_link(USB CDC) 取代, 未加入 Keil 工程编译, 仅保留备查。
 */

#ifndef VISION_CAN_H
#define VISION_CAN_H


#include "main.h"


#define VISION_RX_CAN_ID 0xff
#define VISION_TX_Q_CAN_ID 0x01//0x01
#define VISION_TX_S_CAN_ID 0x110//0x110


/** @brief 视觉接收数据(旧CAN协议) */
typedef struct
{
    uint8_t is_controlling; //是否控制
    uint8_t is_shooting;    //是否开火
    float yaw_angle;        //目标Yaw(弧度)
    float pitch_angle;      //目标Pitch(弧度)
    float distance;         //距离(米)

    uint32_t update_time;   //上次更新时间
} Vision_Rec_Data_t;


extern Vision_Rec_Data_t vision_recv_data;


void Vision_CAN_Decode(uint32_t std_id, uint8_t *rx_data);

void Vision_Send_IMU(FDCAN_HandleTypeDef *hfdcan);

void Vision_Send_Status(FDCAN_HandleTypeDef *hfdcan, float bullet_speed, uint8_t mode, uint8_t shoot_mode);
#endif
