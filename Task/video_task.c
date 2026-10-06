/**
 * @file video_task.c
 * @brief 自定义客户端桥接任务
 *
 */

#include "video_task.h"
#include "video_bridge.h"
#include "cmsis_os2.h"
#include "cmsis_os.h"


// H7 的 DMA 缓冲区必须 32 字节对齐！
// 这里定义全局的桥接结构体，加上对其属性，防止内存乱窜
#if defined ( __ICCARM__ ) || defined ( __CC_ARM ) || defined ( __GNUC__ )
__attribute__((section(".RAM_D2"))) __attribute__((aligned(32))) bridge_context_t g_bridge;
#else
__attribute__((aligned(32))) bridge_context_t g_bridge;
#endif




void Video_Bridge_Task(void *argument)
{
    osDelay(1000); 

    bridge_init(&g_bridge);

    uint32_t last_20ms_tick = osKernelGetTickCount();


		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

    for(;;)
    {
        bridge_poll(&g_bridge);


        uint32_t current_tick = osKernelGetTickCount();
        if(current_tick - last_20ms_tick >= 20)
        {
            bridge_on_20ms_tick(&g_bridge);
            last_20ms_tick += 20;
        }


			vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
