#include "vision_can.h"
#include "string.h"
#include "cmsis_os.h"
#include "fdcan.h"

Vision_Rec_Data_t vision_recv_data;


extern float q0, q1, q2, q3;


/**
 * @brief  解析CAN接收到的视觉数据
 * @note   在 Can_Decode_Task 中被调用
 */
void Vision_CAN_Decode(uint32_t std_id, uint8_t *rx_data)
{
    if (std_id == VISION_RX_CAN_ID)
    {
        vision_recv_data.is_controlling = rx_data[0];
        vision_recv_data.is_shooting    = rx_data[1];

        int16_t yaw_i16 = (int16_t)(rx_data[2] << 8 | rx_data[3]);
        vision_recv_data.yaw_angle = (float)yaw_i16 / 10000.0f;

        int16_t pitch_i16 = (int16_t)(rx_data[4] << 8 | rx_data[5]);
        vision_recv_data.pitch_angle = (float)pitch_i16 / 10000.0f;

        int16_t dist_i16 = (int16_t)(rx_data[6] << 8 | rx_data[7]);
        vision_recv_data.distance = (float)dist_i16 / 1000.0f; // 距离单位通常是米，除以1000

        vision_recv_data.update_time = osKernelGetTickCount();
    }
}

/**
 * @brief  通过CAN发送IMU四元数
 */
void Vision_Send_IMU(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.Identifier = VISION_TX_Q_CAN_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    //根据你的协议注释 (x, y, z, w)，正确打包
    int16_t x = (int16_t)(q1 * 10000.0f); // q1 is x
    int16_t y = (int16_t)(q2 * 10000.0f); // q2 is y
    int16_t z = (int16_t)(q3 * 10000.0f); // q3 is z
    int16_t w = (int16_t)(q0 * 10000.0f); // q0 is w

    tx_data[0] = x >> 8; tx_data[1] = x;
    tx_data[2] = y >> 8; tx_data[3] = y;
    tx_data[4] = z >> 8; tx_data[5] = z;
    tx_data[6] = w >> 8; tx_data[7] = w;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}




/**
 * @brief  通过CAN发送状态信息 (直接发送版)
 */
void Vision_Send_Status(FDCAN_HandleTypeDef *hfdcan, float bullet_speed, uint8_t mode, uint8_t shoot_mode)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.Identifier = VISION_TX_S_CAN_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    int16_t speed = (int16_t)(bullet_speed * 100.0f);

    tx_data[0] = speed >> 8;
    tx_data[1] = speed;
    tx_data[2] = mode;
    tx_data[3] = shoot_mode;
    tx_data[4] = 0; tx_data[5] = 0;
    tx_data[6] = 0; tx_data[7] = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
    }
}









