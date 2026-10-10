/**
 * @file vision_link.c
 * @brief 视觉通信模块
 *
 */

#include "vision_link.h"
#include "chassis_control.h"
#include "usbd_cdc_if.h"
#include "CRC.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "string.h" 
#include "usbd_core.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceHS; 
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;


Vision_Data_t VisionData;

static Vision_Send_t VisionTxData;


//异步重连状态机
static uint8_t usb_reconnect_state = 0;       
static uint32_t usb_reconnect_timer = 0; 


//外部IMU
extern float pitch, yaw, total_yaw;
extern float gyro[3]; // gyro[0]=pitch_rate, gyro[2]=yaw_rate (rad/s)
//extern float quat[4]; 
extern float q0, q1, q2, q3;// 四元数 [w,x,y,z]




//视觉通信初始化
void Vision_Init(void)
{
    memset(&VisionData, 0, sizeof(VisionData));

    // 初始化发送帧头
    VisionTxData.head[0] = 'S'; // 0x53
    VisionTxData.head[1] = 'P'; // 0x50
}


//判断视觉是否在线
bool Vision_IsOnline(void)
{
    if(VisionData.last_update_time == 0) return false;
    return ((osKernelGetTickCount() - VisionData.last_update_time) < VISION_TIMEOUT_MS);
}


//清除目标数据
void Clear_Vision_Target(void)
{
    VisionData.is_tracking = 0;
    VisionData.cmd_mode = 0;
    VisionData.target_yaw_rad = 0.0f;
    VisionData.target_pitch_rad = 0.0f;
    VisionData.target_yaw_acc = 0.0f;
    VisionData.target_pitch_acc = 0.0f;
}


volatile uint32_t vision_count = 0;

//校验并解析视觉报文
void Vision_Process_Data(uint8_t *buf, uint32_t len)
{
    //长度校验 
    if(len != sizeof(Vision_Recv_t))
		{
        return;
    }

    //强制转换
    Vision_Recv_t* pRecv = (Vision_Recv_t*)buf;

    //帧头校验
    if(pRecv->head[0] != 'S' || pRecv->head[1] != 'P') {
        return;
    }

    //CRC校验 (从头开始算，不包括最后2字节)
    uint16_t cal_crc = Get_CRC16_Check_Sum((uint8_t*)pRecv, sizeof(Vision_Recv_t) - 2, 0xFFFF);

    if(cal_crc == pRecv->crc16)
    {
		  	vision_count++;

			  if(isnan(pRecv->yaw) || isinf(pRecv->yaw) || isnan(pRecv->pitch) || isinf(pRecv->pitch))
				{
            return; 
        }

        //检查数值是否在合理范围内 (比如 +/- 180度)
        // 视觉给的是 rad，所以范围是 +/- PI
        if(fabs(pRecv->yaw) > 3.15f || fabs(pRecv->pitch) > 1.57f)
				{
            return;
        }

        //校验通过，更新数据
        VisionData.cmd_mode = pRecv->mode;

        if(pRecv->mode != 0)
        {
            VisionData.is_tracking = 1;
            VisionData.target_yaw_rad = pRecv->yaw;
            VisionData.target_pitch_rad = pRecv->pitch;
            VisionData.target_yaw_acc = pRecv->yaw_acc;
            VisionData.target_pitch_acc = pRecv->pitch_acc;

            VisionData.last_update_time = osKernelGetTickCount();
        }
        else
        {
					    Clear_Vision_Target();

        }
			  VisionData.target_yaw_base_hit = pRecv->yaw_base_hit;
        VisionData.target_pitch_base_hit = pRecv->pitch_base_hit;
    }
}


//打包并发送数据给视觉 
void Vision_Send_Data(void)
{
    //填充IMU数据
    VisionTxData.mode = 1;

    // 四元数
    VisionTxData.q[0] = q0; // w
    VisionTxData.q[1] = q1; // x
    VisionTxData.q[2] = q2; // y
    VisionTxData.q[3] = q3; // z

    // 欧拉角 (rad)
    VisionTxData.yaw = (total_yaw - 180.0f) * (PI / 180.0f);
    VisionTxData.pitch = (pitch - 180.0f) * (PI / 180.0f);

    // 角速度 (rad/s)
    VisionTxData.yaw_vel = gyro[2];
    VisionTxData.pitch_vel = gyro[0];

    //弹速
    VisionTxData.bullet_speed = 11.75f;

    VisionTxData.bullet_count = 0; 


    //颜色(color 0=蓝, 1=红)
    if(Chassis_Get_Color() == 0) VisionTxData.color = 0;//打红方
		else if(Chassis_Get_Color() == 1) VisionTxData.color = 1;//打蓝方

    //计算CRC
     VisionTxData.crc16 = Get_CRC16_Check_Sum((uint8_t*)&VisionTxData, sizeof(Vision_Send_t) - 2, 0xFFFF);


		extern USBD_HandleTypeDef hUsbDeviceHS;
		USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;


    //检查发送器是否空闲
    if(hcdc != NULL && hcdc->TxState == 0) 
    {
        //USB发送
        CDC_Transmit_HS((uint8_t*)&VisionTxData, sizeof(Vision_Send_t));
    }
}



//异步非阻塞 USB 重启状态机
void Vision_Force_Reconnect_Start(void)
{
    if(usb_reconnect_state == 0)
		{
        usb_reconnect_state = 1; 
    }
}


void Vision_Reconnect_Task_Async(void)
{
    if(usb_reconnect_state == 1)
    {
        //瞬间断开 USB (关闭外设，拉低DP引脚让电脑断连)
        USBD_Stop(&hUsbDeviceHS);
        USBD_DeInit(&hUsbDeviceHS);
        HAL_PCD_DevDisconnect(&hpcd_USB_OTG_HS);

        usb_reconnect_timer = osKernelGetTickCount();
        usb_reconnect_state = 2; // 进入等待状态
    }
    else if(usb_reconnect_state == 2)
    {
        //挂起等待 500ms 后重新连接
        if(osKernelGetTickCount() - usb_reconnect_timer >= 500)
        {
            HAL_PCD_DevConnect(&hpcd_USB_OTG_HS);
            MX_USB_DEVICE_Init();
            Clear_Vision_Target(); //重连后清空旧数据
            usb_reconnect_state = 0; //重置完成
        }
    }
}
