/**
 * @file bsp_DR16.c
 * @brief DR16 接收机驱动
 *
 */

#include "bsp_DR16.h"
#include "usart.h"
#include "string.h"
#include "stdlib.h"
#define ABS(x) ((x) > 0 ? (x) : -(x))


static DR16_Data_t dr16_data = {0}; //遥控器数据
static uint32_t dr16_last_update_tick = 0; //上一次接收时间


//接收缓冲区：32字节对齐，适配H7 Cache
__attribute__((aligned(32))) static uint8_t sbus_rx_buf[DR16_RX_BUF_NUM]; 


/**
  * @brief  遥控器初始化
  */
void DR16_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&DR16_UART_HANDLE, sbus_rx_buf, DR16_RX_BUF_NUM);
}


/**
  * @brief  判断DR16是否在线
  */
bool DR16_IsOnline(void)
{
    if(dr16_last_update_tick == 0) return false;
    return ((osKernelGetTickCount() - dr16_last_update_tick) <= DR16_TIMEOUT_MS);
}


/**
  * @brief  获取 DR16 最新数据
  */
const DR16_Data_t* DR16_Get_Data(void)
{
    DR16_Data_t temp_rc;

    //从队列异步提取
    if(osMessageQueueGet(rcQueueHandle, &temp_rc, NULL, 0) == osOK)
    {
        dr16_data = temp_rc;
    }

		//离线归零
    if(!DR16_IsOnline())
	  {
        memset(&dr16_data.rc, 0, sizeof(dr16_data.rc));
        memset(&dr16_data.mouse, 0, sizeof(dr16_data.mouse));
        memset(&dr16_data.key, 0, sizeof(dr16_data.key));
		}

    return &dr16_data ;
}


/**
  * @brief  DBUS 协议底层解码
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf)
{
    DR16_Data_t rc_raw = {0}; 

    rc_raw.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    rc_raw.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
    rc_raw.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;
    rc_raw.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
    rc_raw.rc.s[0]  = ((sbus_buf[5] >> 4) & 0x0003);
    rc_raw.rc.s[1]  = ((sbus_buf[5] >> 4) & 0x000C) >> 2;

    rc_raw.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
    rc_raw.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
    rc_raw.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
    rc_raw.mouse.press_l = sbus_buf[12];
    rc_raw.mouse.press_r = sbus_buf[13];
    rc_raw.key.v = sbus_buf[14] | (sbus_buf[15] << 8);
    rc_raw.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);

    //偏移处理
    for(int i=0; i<5; i++)
  	{
        rc_raw.rc.ch[i] -= DR16_CH_VALUE_OFFSET;
    }

    for(int i=0; i<5; i++)
    {
        if(rc_raw.rc.ch[i] > 660)  rc_raw.rc.ch[i] = 660;
        if(rc_raw.rc.ch[i] < -660) rc_raw.rc.ch[i] = -660;
    }

		//死区处理
    if(ABS(rc_raw.rc.ch[0]) <= 10) 
    {
        rc_raw.rc.ch[0] = 0;
    }
    if(ABS(rc_raw.rc.ch[3]) <= 10) 
    {
        rc_raw.rc.ch[3] = 0;
    }

    //异常检测
    if(ABS(rc_raw.rc.ch[0]) > 660 || ABS(rc_raw.rc.ch[1]) > 660)
	  {
        memset(&rc_raw, 0, sizeof(rc_raw));
    }

    dr16_last_update_tick = osKernelGetTickCount();//更新时间戳


    //FIFO环形推入队列
    if(osMessageQueueGetCount(rcQueueHandle) > 0)
  	{
        DR16_Data_t dummy;
        osMessageQueueGet(rcQueueHandle, &dummy, NULL, 0);//如果队列里有旧数据，先移除
    }
    osMessageQueuePut(rcQueueHandle, &rc_raw, 0, 0);
}


//接收回调
void DR16_RX_Callback_Handler(void)
{
	  SCB_InvalidateDCache_by_Addr((uint32_t *)sbus_rx_buf, DR16_RX_BUF_NUM);//H7 Cache 刷新

  	sbus_to_rc(sbus_rx_buf);//解析并入队
}


void DR16_Reset(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&DR16_UART_HANDLE, sbus_rx_buf, DR16_RX_BUF_NUM);
}
