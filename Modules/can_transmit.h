/**
 * @file can_transmit.h
 * @brief CAN 发送协议层接口
 *
 */

#ifndef CAN_TRANSMIT_H
#define CAN_TRANSMIT_H

#include "main.h"

//云台Yaw CAN ID
#define LK_YAW_ID 0x141



/** @brief 大疆电机指令发送 */
void DJI_Motor_Send_Command(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

/** @brief LK 电机转矩控制发送 */
void LK_Send_Torque(FDCAN_HandleTypeDef* hfdcan, float current_amps);

/** @brief 达妙速度控制指令发送给底盘 */
void Dial_Send_Speed(FDCAN_HandleTypeDef *hfdcan, float target_vel);

/** @brief 拨盘模式指令发送给底盘 */
void Dial_Send_Mode_Command(FDCAN_HandleTypeDef *hfdcan, uint8_t command);

/** @brief 底盘控制指令发送 */
void Send_Chassis_Command(FDCAN_HandleTypeDef* hfdcan, int16_t vx, int16_t vy, int16_t wz, uint8_t mode, uint8_t supercap_req, uint8_t climb_cmd, uint8_t led_cmd, uint8_t key_shift, uint8_t friction_state, uint8_t is_base_locked, uint8_t battle_mode);

/** @brief UI数据发送 */
void Send_Chassis_UI_Data(FDCAN_HandleTypeDef* hfdcan, float yaw_deg, float pitch_deg, float friction_speed);

#endif
