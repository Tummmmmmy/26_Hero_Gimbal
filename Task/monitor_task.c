#include "monitor_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shooter_control.h"
#include "gimbal_control.h"





//离线检测任务
void Monitor_Task(void const * argument)
{
    osDelay(1000); 
    
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
		const TickType_t TimeIncrement = pdMS_TO_TICKS(10);

    for(;;)
    {
        uint32_t now = osKernelGetTickCount();

        //云台
        Gimbal_Check_Motors_Online(now);
        
        //拨盘
        Shooter_Check_Motors_Online(now);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
