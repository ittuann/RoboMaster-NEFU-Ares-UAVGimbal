/*
 * remote_control.h
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#include "user_lib.h"
#include "bsp_usart.h"
#include "chassis.h"
#include "gimbal.h"
#include "can_receive.h"

#define RC_FRAME_LENGTH 18u								// 遥控器通信协议字节长度
#define SBUS_RX_BUF_NUM 36u

/* ----------------------- RC Channel Definition ---------------------------- */
#define RC_CH_VALUE_MIN         	((uint16_t)364)
#define RC_CH_VALUE_OFFSET      	((uint16_t)1024)
#define RC_CH_VALUE_MAX         	((uint16_t)1684)
#define CHASSIS_X_CHANNEL 			1					// 底盘前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 			0					// 底盘左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL			2					// 在特殊模式下，可以通过遥控器控制旋转
#define YAW_CHANNEL   				0					// yaw控制通道
#define PITCH_CHANNEL 				1					// pitch控制通道
#define CHASSIS_RC_DEADLINE			10					// 底盘遥感死区
#define GIMBAL_RC_DEADLINE			10					// 云台遥感死区
/* ----------------------- RC Switch Definition ----------------------------- */
#define RC_SW_UP                	((uint16_t)1)
#define RC_SW_MID               	((uint16_t)3)
#define RC_SW_DOWN              	((uint16_t)2)
#define switch_is_up(s)         	(s == RC_SW_UP)
#define switch_is_mid(s)        	(s == RC_SW_MID)
#define switch_is_down(s)			(s == RC_SW_DOWN)
#define RC_SWLeft                	(1)
#define RC_SWRight                	(0)
/* ----------------------- PC Key Definition -------------------------------- */
#define KEY_PRESSED_OFFSET_W		((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S		((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A		((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D		((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT	((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL		((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q		((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E		((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R		((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F		((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G		((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z		((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X		((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C		((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V		((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B		((uint16_t)1 << 15)

typedef struct __attribute__((packed))
{
        struct __attribute__((packed))
        {
                int16_t ch[5];		// 最大值1684 中间值1024 最小值 364
                char s[2];			// 最大值3 最小值1
        } rc;
        struct __attribute__((packed))
        {
                int16_t x;			// 最大值32767 最小值-32768 静止值0
                int16_t y;			// 最大值32767 最小值-32768 静止值0
                int16_t z;			// 最大值32767 最小值-32768 静止值0
                uint8_t press_l;	// 最大值1 最小值0
                uint8_t press_r;	// 最大值1 最小值0
        } mouse;
        struct __attribute__((packed))
        {
                uint16_t v;			// 位值标识
        } key;
} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

extern	void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
extern	void Remote_Control_Init(void);
extern	void RC_Gimbal_Diagram(void);
extern	void RC_Chassis_Diagram(void);

extern	void RC_Restart(uint16_t dma_buf_num);
extern	void RC_Unable(void);

#endif
