/*
 * remote_control.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "remote_control.h"

extern UART_HandleTypeDef	huart3;
extern DMA_HandleTypeDef	hdma_usart3_rx;

RC_ctrl_t rc_ctrl;							// 遥控器数据结构体
static LpfRC1st_t RC_CH_FIR[4];				// 遥控器滤波结构体

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];	// 接收原始数据, 为18个字节, 给了36个字节长度, 防止DMA传输越界

/**
  * @brief          遥控器初始化
  */
void Remote_Control_Init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          遥控器D-BUS协议解析
  * @notion			接收机每隔14ms通过DBUS发送一帧18字节数据
  * @notion			遥控器发射机开关位 1上2下3中；鼠标在XYZ轴的移动速度 负左正右；鼠标按键 0没按下1按下
  * @notion			每个键盘按键对应一个bit Bit0-W；Bit1-S；Bit2-A；Bit3-D；Bit4-Q；Bit5-E；Bit6-Shift；Bit7-Ctrl
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == 0 || rc_ctrl == 0) {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        // 遥控器通道0 右摇杆横向
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; // 遥控器通道1 右摇杆纵向
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |
                         (sbus_buf[4] << 10)) &0x07ff;						// 遥控器通道2 左摇杆横向
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; // 遥控器通道3 左摇杆纵向
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  		// 遥控器发射机 S1 左侧开关位
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;					// 遥控器发射机 S2 右侧开关位
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    // 鼠标在X轴的移动速度
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    // 鼠标在Y轴的移动速度
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  // 鼠标在Z轴的移动速度
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  // 鼠标左键是否按下
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  // 鼠标右键是否按下
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    // 键盘按键
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL保留字段

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;								// 减去中值后为[-660, 660]
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
  * @brief          遥控器的死区限制，因为遥控器的拨杆老化后在中位的时不一定为0
  */
#define rc_deadband_limit(input, output, dealine)        	\
{                                                    		\
        if ((input) > (dealine) || (input) < -(dealine)) {	\
            (output) = (input);                          	\
        } else {											\
            (output) = 0;                                	\
        }                                                	\
    }
	
/**
  * @brief          根据遥控器分解计算
  */
void RC_Gimbal_Diagram(void)
{
	// 输入遥控器值并做死区限制
	rc_deadband_limit(rc_ctrl.rc.ch[YAW_CHANNEL], RC_CH_FIR[YAW_CHANNEL].OriginData, GIMBAL_RC_DEADLINE);
    rc_deadband_limit(rc_ctrl.rc.ch[PITCH_CHANNEL], RC_CH_FIR[PITCH_CHANNEL].OriginData, GIMBAL_RC_DEADLINE);

    // 量纲转换
    RC_CH_FIR[YAW_CHANNEL].OriginData = RC_CH_FIR[YAW_CHANNEL].OriginData * RC_Yaw_Sen;
    RC_CH_FIR[PITCH_CHANNEL].OriginData = RC_CH_FIR[PITCH_CHANNEL].OriginData * RC_Pitch_Sen;
	
	// 一阶低通滤波代替斜波作为底盘速度输入
	LowPassFilterRC1st(&RC_CH_FIR[YAW_CHANNEL], 0.800f, RC_CH_FIR[YAW_CHANNEL].OriginData);
	LowPassFilterRC1st(&RC_CH_FIR[PITCH_CHANNEL], 0.800f, RC_CH_FIR[PITCH_CHANNEL].OriginData);
	
	// 不需要缓慢加减速 并防止累计误差
	if (fabsf(RC_CH_FIR[YAW_CHANNEL].FilterData) < fabsf(GIMBAL_RC_DEADLINE * RC_Yaw_Sen)) {
		RC_CH_FIR[YAW_CHANNEL].FilterData = 0.0f;
	}
	if (fabsf(RC_CH_FIR[PITCH_CHANNEL].FilterData) < fabsf(GIMBAL_RC_DEADLINE * RC_Pitch_Sen)) {
		RC_CH_FIR[PITCH_CHANNEL].FilterData = 0.0f;
	}
	
	// 输出
	PID_Mortor_Angle[AnglePID_GimbalPitch].EX_Val += RC_CH_FIR[PITCH_CHANNEL].FilterData;
//	PID_Mortor_Angle[AnglePID_GimbalYaw].EX_Val = RC_CH_FIR[YAW_CHANNEL].FilterData + PID_Mortor_Angle[AnglePID_GimbalYaw].Now_Val;
	PID_Mortor_Angle[AnglePID_GimbalYaw].EX_Val += RC_CH_FIR[YAW_CHANNEL].FilterData;
	PID_Mortor_Angle[AnglePID_GimbalPitch_Relative].EX_Val += RC_CH_FIR[PITCH_CHANNEL].FilterData;

	// 限幅
	LIMIT(PID_Mortor_Angle[AnglePID_GimbalPitch].EX_Val, GimbalMachine_Pitch.Min_Angle, GimbalMachine_Pitch.Max_Angle);
	LIMIT(PID_Mortor_Angle[AnglePID_GimbalYaw].EX_Val, GimbalMachine_Yaw.Min_Angle, GimbalMachine_Yaw.Max_Angle);
	LIMIT(PID_Mortor_Angle[AnglePID_GimbalPitch_Relative].EX_Val, GimbalMachine_Pitch.Min_Angle, GimbalMachine_Pitch.Max_Angle);
}

/**
  * @brief          底盘速度分解计算
  */
void RC_Chassis_Diagram(void)
{
	float vx_set = 0.000f, vy_set = 0.000f, wz_set = 0.000f;

	// 输入遥控器值并做死区限制
	rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL], vx_set, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL], vy_set, CHASSIS_RC_DEADLINE);

    // 量纲转换
	vx_set = vx_set *  CHASSIS_VX_RC_SEN;
	vy_set = vy_set * -CHASSIS_VY_RC_SEN;

	// 一阶低通滤波代替斜波作为底盘速度输入
	vx_set = LowPassFilterRC1st(&RC_CH_FIR[CHASSIS_X_CHANNEL], 0.800f, vx_set);
	vy_set = LowPassFilterRC1st(&RC_CH_FIR[CHASSIS_Y_CHANNEL], 0.800f, vy_set);

	// 不需要缓慢加减速 并防止累计误差
	if (fabsf(vx_set) < fabsf(CHASSIS_RC_DEADLINE *  CHASSIS_VX_RC_SEN)) {
		vx_set = 0;
	}
	if (fabsf(vy_set) < fabsf(CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)) {
		vy_set = 0;
	}

	// 遥控器Z轴在一些模式下对底盘的作用
	if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW || chassis_behaviour_mode == CHASSIS_OPEN) {
		// CHASSIS_NO_FOLLOW_YAW 模式
		rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL], wz_set, CHASSIS_RC_DEADLINE);
		wz_set = wz_set * -CHASSIS_WZ_RC_SEN;
		wz_set = LowPassFilterRC1st(&RC_CH_FIR[CHASSIS_WZ_CHANNEL], 0.800f, wz_set);
		if (fabsf(wz_set) < fabsf(CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN)) {
			wz_set = 0;
		}
		// CHASSIS_OPEN 模式
		if (chassis_behaviour_mode == CHASSIS_OPEN) {
			vx_set = vx_set *  CHASSIS_OPEN_RC_SEN;
			vy_set = vy_set * -CHASSIS_OPEN_RC_SEN;
			wz_set = wz_set * -CHASSIS_OPEN_RC_SEN;
		}
	}

	// 输出四个轮子运动分解后的期望速度
//	PID_Mortor_Speed[Chassis_M1].EX_Val = (vx_set + vy_set + wz_set); //前左轮
//	PID_Mortor_Speed[Chassis_M2].EX_Val = (vx_set - vy_set - wz_set); //前右轮
//	PID_Mortor_Speed[Chassis_M3].EX_Val = (vx_set + vy_set - wz_set); //后左轮
//	PID_Mortor_Speed[Chassis_M4].EX_Val = (vx_set - vy_set + wz_set); //后右轮
}

/**
  * @brief          重启遥控器
  */
void RC_Restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
}

/**
  * @brief          关闭遥控器
  */
void RC_Unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
