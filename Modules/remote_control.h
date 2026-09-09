#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#include <stdbool.h>


//统一的遥控器数据结构体
typedef struct {

    struct {
        int16_t right_x;
        int16_t right_y;
        int16_t left_x;
        int16_t left_y;
		  	int16_t wheel;
        uint8_t s_left;
        uint8_t s_right;
				uint8_t pause:1;
				uint8_t fn_1:1;        //自定义键左
				uint8_t fn_2:1;        //自定义键右
				uint8_t trigger:1;     //扳机
    } rc;

    //鼠标
    struct {
        int16_t x;
        int16_t y;
        int16_t z; //滚轮
        bool press_l;
        bool press_r;
			  bool press_middle;
    } mouse;

    //键盘
    union {
        uint16_t v;
        struct {
            bool W:1, S:1, A:1, D:1, SHIFT:1, CTRL:1, Q:1, E:1,
                 R:1, F:1, G:1, Z:1, X:1, C:1, V:1, B:1;
        } map;
    } key;
     bool request_super_cap;
} RC_Data_t;



//遥控控制源
typedef enum {
    CONTROL_SOURCE_NONE,  //全部离线
    CONTROL_SOURCE_DR16,  //DR16在线
    CONTROL_SOURCE_VTM    //VTM在线
} ControlSource_e;



extern volatile ControlSource_e control_source;

//刷新遥控器状态
void RC_Update(void);         

//获取最终处理好的数据指针
const RC_Data_t* RC_Get_Data(void);

#endif
