/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "INS_task.h"
#include "bsp_DR16.h"
#include "bsp_VT03.h"
#include "bsp_buzzer.h"
#include "bsp_ano_protocol.h"
#include "device_task.h"
#include "usb_device.h"
#include "bsp_dwt.h"
#include "Servo_control.h"
#include "bsp_can.h"
#include "robot_control.h"
#include "control_task.h"
#include "print_task.h"
#include "can_task.h"


//#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */
//RC_ctrl_t *rc; 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */




/* USER CODE END Variables */
/* Definitions for All_Init */
osThreadId_t All_InitHandle;
const osThreadAttr_t All_Init_attributes = {
  .name = "All_Init",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
uint32_t IMU_Buffer[ 1024 ];
osStaticThreadDef_t IMUControlBlock;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .cb_mem = &IMUControlBlock,
  .cb_size = sizeof(IMUControlBlock),
  .stack_mem = &IMU_Buffer[0],
  .stack_size = sizeof(IMU_Buffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Gimbal_Task */
osThreadId_t Gimbal_TaskHandle;
const osThreadAttr_t Gimbal_Task_attributes = {
  .name = "Gimbal_Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CanDecodeTask */
osThreadId_t CanDecodeTaskHandle;
const osThreadAttr_t CanDecodeTask_attributes = {
  .name = "CanDecodeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for VisionTask */
osThreadId_t VisionTaskHandle;
const osThreadAttr_t VisionTask_attributes = {
  .name = "VisionTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanTransmitTask */
osThreadId_t CanTransmitTaskHandle;
const osThreadAttr_t CanTransmitTask_attributes = {
  .name = "CanTransmitTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AnoPrintTask */
osThreadId_t AnoPrintTaskHandle;
const osThreadAttr_t AnoPrintTask_attributes = {
  .name = "AnoPrintTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for DeviceTask */
osThreadId_t DeviceTaskHandle;
const osThreadAttr_t DeviceTask_attributes = {
  .name = "DeviceTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PrintEntryTask */
osThreadId_t PrintEntryTaskHandle;
const osThreadAttr_t PrintEntryTask_attributes = {
  .name = "PrintEntryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for VideoBridgeTask */
osThreadId_t VideoBridgeTaskHandle;
const osThreadAttr_t VideoBridgeTask_attributes = {
  .name = "VideoBridgeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal4,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for rcQueue */
osMessageQueueId_t rcQueueHandle;
const osMessageQueueAttr_t rcQueue_attributes = {
  .name = "rcQueue"
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};
/* Definitions for anoPrintQueue */
osMessageQueueId_t anoPrintQueueHandle;
const osMessageQueueAttr_t anoPrintQueue_attributes = {
  .name = "anoPrintQueue"
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
osStaticSemaphoreDef_t imuBinarySemControlBlock;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01",
  .cb_mem = &imuBinarySemControlBlock,
  .cb_size = sizeof(imuBinarySemControlBlock),
};
/* Definitions for anoTxSem */
osSemaphoreId_t anoTxSemHandle;
const osSemaphoreAttr_t anoTxSem_attributes = {
  .name = "anoTxSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void All_Init_Run(void *argument);
extern void IMU_task(void const *argument);
extern void vision_task(void *argument);
extern void Print_Task(void *argument);
void Device_Task(void *argument);
extern void Video_Bridge_Task(void *argument);
extern void Monitor_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  imuBinarySem01Handle = osSemaphoreNew(1, 1, &imuBinarySem01_attributes);

  /* creation of anoTxSem */
  anoTxSemHandle = osSemaphoreNew(1, 1, &anoTxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rcQueue */
  rcQueueHandle = osMessageQueueNew (1, sizeof(DR16_Data_t), &rcQueue_attributes);

  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (20, sizeof(CAN_Rx_Message_t), &canRxQueue_attributes);

  /* creation of anoPrintQueue */
  anoPrintQueueHandle = osMessageQueueNew (10, sizeof(Print_Msg_t), &anoPrintQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of All_Init */
  All_InitHandle = osThreadNew(All_Init_Run, NULL, &All_Init_attributes);

  /* creation of INSTask */
  INSTaskHandle = osThreadNew((osThreadFunc_t)IMU_task, NULL, &INSTask_attributes);

  /* creation of Gimbal_Task */
  Gimbal_TaskHandle = osThreadNew((osThreadFunc_t)Gimbal_Control_task, NULL, &Gimbal_Task_attributes);

  /* creation of CanDecodeTask */
  CanDecodeTaskHandle = osThreadNew((osThreadFunc_t)Can_Decode_Task, NULL, &CanDecodeTask_attributes);

  /* creation of VisionTask */
  VisionTaskHandle = osThreadNew(vision_task, NULL, &VisionTask_attributes);

  /* creation of AnoPrintTask */
  AnoPrintTaskHandle = osThreadNew(Print_Task, NULL, &AnoPrintTask_attributes);

  /* creation of DeviceTask */
  DeviceTaskHandle = osThreadNew(Device_Task, NULL, &DeviceTask_attributes);

  /* creation of PrintEntryTask */
  PrintEntryTaskHandle = osThreadNew((osThreadFunc_t)Print_Entry_Task, NULL, &PrintEntryTask_attributes);

  /* creation of VideoBridgeTask */
  VideoBridgeTaskHandle = osThreadNew(Video_Bridge_Task, NULL, &VideoBridgeTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(Monitor_Task, NULL, &MonitorTask_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_All_Init_Run */
/**
  * @brief  Function implementing the All_Init thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_All_Init_Run */
void All_Init_Run(void *argument)
{
  /* USER CODE BEGIN All_Init_Run */
	
	osDelay(100);

	MX_USB_DEVICE_Init();
	
	taskENTER_CRITICAL();  //进入临界区
	
	BSP_FDCAN_Config();
	
	DR16_Init();
	
	Robot_Param_Init();
	
	taskEXIT_CRITICAL();   //退出临界区
	
	vTaskDelete(NULL);  
  /* USER CODE END All_Init_Run */
}

/* USER CODE BEGIN Header_Device_Task */
/**
* @brief Function implementing the DeviceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Device_Task */
void Device_Task(void *argument)
{
  /* USER CODE BEGIN Device_Task */
  /* Infinite loop */
  for(;;)
  {
    Device_task();
  }
  /* USER CODE END Device_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
