#include "vision_task.h"
#include "vision_link.h"
#include "cmsis_os.h"
#include "INS_task.h"



void vision_task(void const * argument)
{
	osDelay(2000); 
	
	Vision_Init();
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

	for(;;)
	{
		
		  Vision_Send_Data();
		
		
			vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

