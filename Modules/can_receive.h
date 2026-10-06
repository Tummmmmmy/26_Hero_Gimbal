/**
 * @file can_receive.h
 * @brief CAN 接收模块接口
 *
 */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "bsp_can.h"

/** @brief 按总线/ID 分发CAN接收帧到对应模块 */
void CAN_Rx_Route(const CAN_Rx_Message_t* msg);

#endif
