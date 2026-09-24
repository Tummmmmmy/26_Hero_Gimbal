/**
 * @file bsp_VT03.h
 * @brief VT03 图传遥控驱动接口与帧格式定义
 */

#ifndef BSP_VT03_H
#define BSP_VT03_H

#include "main.h"
#include "usart.h"
#include <stdbool.h>


#define VT03_UART_HANDLE      huart1
#define VT03_PACK_SIZE        21u //完整帧长度 = 2(帧头) + 17(数据) + 2(CRC)
#define VT03_DATA_LEN         17u //纯数据长度
#define VT03_FRAME_HEADER_1   0xA9
#define VT03_FRAME_HEADER_2   0x53
#define VT03_RX_BUFFER_SIZE   128u

//离线判定超时阈值
#define VT03_TIMEOUT_MS       200u 

typedef struct
{
  union {
		uint8_t dataBuff[VT03_DATA_LEN];
	  struct {
			uint64_t ch_0:11;       //右摇杆 X
			uint64_t ch_1:11;       //右摇杆 Y
			uint64_t ch_2:11;       //左摇杆 Y
			uint64_t ch_3:11;       //左摇杆 X
			uint64_t mode_sw:2;     //C/N/S 滑块
			uint64_t pause:1;
			uint64_t fn_1:1;        //自定义键左
			uint64_t fn_2:1;        //自定义键右
			uint64_t wheel:11;      //拨轮
			uint64_t trigger:1;     //扳机
			int16_t mouse_x;
			int16_t mouse_y;
			int16_t mouse_z;
			uint8_t mouse_left:2;
			uint8_t mouse_right:2;
			uint8_t mouse_middle:2;
			union
			{
					uint16_t key_code;
					struct
					{
            bool press_W : 1;
            bool press_S : 1;
            bool press_A : 1;
            bool press_D : 1;

            bool press_Shift : 1;
            bool press_Ctrl : 1;

            bool press_Q : 1;
            bool press_E : 1;
            bool press_R : 1;
            bool press_F : 1;
            bool press_G : 1;
            bool press_Z : 1;

            bool press_X : 1;
            bool press_C : 1;
            bool press_V : 1;
            bool press_B : 1;
					} Key_Code;
			} keyBoard;
		};
	}data;
	bool is_online;           //在线状态标志位
} VT03_Receive_t;


//VT03初始化
void VT03_Init(void);

//判断VT03是否在线
bool VT03_IsOnline(void);

//获取处理后的遥控器数据
const VT03_Receive_t* VT03_GetData(void);

//接收回调
void VT03_RX_Callback_Handler(uint16_t Size);

//串口错误复位与重启函数
void VT03_Reset(void);

#endif
