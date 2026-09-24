/**
 * @file remote_control.c
 * @brief 遥控器输入统一模块
 *
 * 把 DR16 与 VT03 两路遥控数据统一成 RC_Data_t, 并维护当前控制源
 * (CONTROL_SOURCE_DR16 / CONTROL_SOURCE_VTM)。
 */

#include "remote_control.h"
#include "bsp_DR16.h"
#include "bsp_vt03.h"
#include "string.h"


//最终遥控数据
static RC_Data_t final_rc_data = {0};

//当前遥控器控制源
volatile ControlSource_e control_source = CONTROL_SOURCE_NONE;


void RC_Update(void)
{
		//决定用哪个遥控
    if(VT03_IsOnline())
    {
        control_source = CONTROL_SOURCE_VTM;
    }
    else if(DR16_IsOnline())
    {
        control_source = CONTROL_SOURCE_DR16;
    }
    else
    {
        control_source = CONTROL_SOURCE_NONE;
    }


	  //清空输出缓存
    memset(&final_rc_data, 0, sizeof(RC_Data_t));

    if(control_source == CONTROL_SOURCE_VTM)
    {
			  const VT03_Receive_t* vtm_raw = VT03_GetData();

        //填充VT03数据
        final_rc_data.rc.right_x = vtm_raw->data.ch_0 - 1024;
        final_rc_data.rc.right_y = vtm_raw->data.ch_1 - 1024;
        final_rc_data.rc.left_y  = vtm_raw->data.ch_2 - 1024;
        final_rc_data.rc.left_x  = vtm_raw->data.ch_3 - 1024;
		  	final_rc_data.rc.wheel = -(vtm_raw->data.wheel - 1024);
			  final_rc_data.rc.pause = vtm_raw->data.pause;
		  	final_rc_data.rc.fn_1 = vtm_raw->data.fn_1;
        final_rc_data.rc.fn_2 = vtm_raw->data.fn_2;
		  	final_rc_data.rc.trigger = vtm_raw->data.trigger;

        //C(0)/N(1)/S(2)转换成和DR16一样的1(UP)/2(DOWN)/3(MID)
        if(vtm_raw->data.mode_sw == 0) final_rc_data.rc.s_left = 2;       //C -> DOWN
        else if(vtm_raw->data.mode_sw == 1) final_rc_data.rc.s_left = 3; //N -> MID
        else if(vtm_raw->data.mode_sw == 2) final_rc_data.rc.s_left = 1; //S -> UP

        //鼠标
        final_rc_data.mouse.x = vtm_raw->data.mouse_x;
        final_rc_data.mouse.y = vtm_raw->data.mouse_y;
        final_rc_data.mouse.press_l = vtm_raw->data.mouse_left;
        final_rc_data.mouse.press_r = vtm_raw->data.mouse_right;
        final_rc_data.mouse.press_middle = vtm_raw->data.mouse_middle;

        //键盘
        final_rc_data.key.v = vtm_raw->data.keyBoard.key_code;

		  	final_rc_data.request_super_cap = vtm_raw->data.keyBoard.Key_Code.press_C;
    }
    else if(control_source == CONTROL_SOURCE_DR16)
    {
			  const DR16_Data_t* DR16_data = DR16_Get_Data();

        //填充DR16数据
        final_rc_data.rc.right_x = DR16_data->rc.ch[0];
        final_rc_data.rc.right_y = DR16_data->rc.ch[1];
        final_rc_data.rc.left_x  = DR16_data->rc.ch[2];
        final_rc_data.rc.left_y  = DR16_data->rc.ch[3];
		   	final_rc_data.rc.wheel   = DR16_data->rc.ch[4];

        final_rc_data.rc.s_left  = DR16_data->rc.s[1];//左拨杆
        final_rc_data.rc.s_right = DR16_data->rc.s[0];//右拨杆


			  //鼠标
        final_rc_data.mouse.x = DR16_data->mouse.x;
        final_rc_data.mouse.y = DR16_data->mouse.y;
        final_rc_data.mouse.z = DR16_data->mouse.z;
        final_rc_data.mouse.press_l = DR16_data->mouse.press_l;
        final_rc_data.mouse.press_r = DR16_data->mouse.press_r;

	  		//键盘
        final_rc_data.key.v = DR16_data->key.v;

				//超电
			  final_rc_data.request_super_cap = final_rc_data.key.map.C ||  (((DR16_data->rc.s[1] == DR16_SW_MID && DR16_data->rc.s[0] == DR16_SW_MID) || (DR16_data->rc.s[1] == DR16_SW_DOWN && DR16_data->rc.s[0] == DR16_SW_MID)) && DR16_data->rc.ch[3] > 600);
    }
}


const RC_Data_t* RC_Get_Data(void)
{
    return &final_rc_data;
}
