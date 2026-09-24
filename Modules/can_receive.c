/**
 * @file can_receive.c
 * @brief CAN 接收路由模块
 *
 * 按总线/ID 把收到的 CAN 帧分发到对应模块的解码接口
 */

#include "can_receive.h"
#include "fdcan.h"
#include "can_transmit.h"
#include "gimbal_control.h"
#include "chassis_control.h"
#include "shooter_control.h"

// 拨盘电机反馈帧 ID(DM 电机)
#define RX_DIAL_FDB_ID 0x302

void CAN_Rx_Route(const CAN_Rx_Message_t* msg)
{
    if(msg->hfdcan->Instance == FDCAN1)
    {
        //云台Yaw电机反馈
        if(msg->can_id == LK_YAW_ID)
        {
            Gimbal_Update_Yaw(msg->data);
        }
        //拨盘电机反馈
        else if(msg->can_id == RX_DIAL_FDB_ID)
        {
            Shooter_CAN_Feedback(msg->can_id, msg->data);
        }
        //双板通信
        else if(msg->can_id == RX_CHASSIS_DATA_ID)
        {
            Chassis_Decode_Feedback(msg->data);
        }
    }
    else if(msg->hfdcan->Instance == FDCAN2)
    {
        //摩擦轮电机
        if(msg->can_id >= 0x201 && msg->can_id <= 0x207)
        {
            Shooter_CAN_Feedback(msg->can_id, msg->data);
        }
    }
    else if(msg->hfdcan->Instance == FDCAN3)
    {
        //云台Pitch电机反馈
        if(msg->can_id == (0x200 + GIMBAL_PITCH_MOTOR_ID))
        {
            Gimbal_Update_Pitch(msg->data);
        }
    }
}
