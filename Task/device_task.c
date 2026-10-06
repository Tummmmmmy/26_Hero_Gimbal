/**
 * @file device_task.c
 * @brief 设备指示任务
 *
 */

#include "device_task.h"
#include "robot_control.h"
#include "ws2812.h"
#include "cmsis_os2.h"


void Device_task(void)
{

	osDelay(500);

	while(1)
	{	
			switch(Robot_Get_Led_State())
			{
					case LED_STATE_BOOTING:
					    WS2812_Set_Colour(50, 50, 0); //黄色
							break;

					case LED_STATE_DISABLED:
							WS2812_Set_Colour(50, 0, 0);   //红色
							break;

					case LED_STATE_ENABLED:
						  WS2812_Set_Colour(50, 50, 0); //黄色
							break;

					case LED_STATE_CLIMB_FRONT:

							break;

					case LED_STATE_CLIMB_BACK:

							break;


					case LED_STATE_BASE_HIT_READY:
					{
							for(int i = 0; i <= 100; i+=10) 
						  { 
									WS2812_Set_Colour(0, (uint8_t)(100 * (i/150.0f)), (uint8_t)(150 * (i/100.0f)));
									osDelay(5);
							}
							break;
					}

					case LED_STATE_RESET:
					{
                for(int i = 0; i < 3; i++)
                {
                    WS2812_Set_Colour(50, 50, 0);
                    osDelay(100);
                    WS2812_Set_Colour(0, 0, 0);
                    osDelay(50);
                }
							break;
					}

					default:
							WS2812_Set_Colour(255, 0, 0);
							osDelay(50);
							WS2812_Set_Colour(0, 0, 0);
							break;
				}
			osDelay(100);
	}
}


/**
  * @brief  实现一次完整的呼吸灯效果
  * @param  r, g, b: 呼吸灯的目标颜色
  * @param  delay_ms: 每一步亮度变化的延时，值越小，呼吸越快
  */
void WS2812_Do_Breathe(uint8_t r, uint8_t g, uint8_t b, uint8_t delay_ms)
{
    //从暗到亮
    for (int i = 0; i <= 100; i++)
  	{
        //使用 i/100.0 作为亮度百分比
        uint8_t current_r = (uint8_t)(r * (i / 100.0f));
        uint8_t current_g = (uint8_t)(g * (i / 100.0f));
        uint8_t current_b = (uint8_t)(b * (i / 100.0f));
        WS2812_Set_Colour(current_r, current_g, current_b);
        osDelay(delay_ms);
    }

    //从亮到暗
    for (int i = 100; i >= 0; i--)
		{
        uint8_t current_r = (uint8_t)(r * (i / 100.0f));
        uint8_t current_g = (uint8_t)(g * (i / 100.0f));
        uint8_t current_b = (uint8_t)(b * (i / 100.0f));
        WS2812_Set_Colour(current_r, current_g, current_b);
        osDelay(delay_ms);
    }
}
