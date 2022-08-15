/*
 *  anotc.c
 *
 *  Created on: 2021年11月5日
 *      Author: LBQ
 */

#include "anotc.h"

#include "MadgwickAHRS.h"
#include "can_receive.h"

#define UserDataLen	4	// 数据长度

extern UART_HandleTypeDef huart1;

/*
 * @name:       void Wireless_Send(void)
 * @function:   Wireless_Send
 * @param:      none
 * @return:     none
 * @notion:     V7
 * @notion:     逐飞无线串口一次发送数据最好不要超过30字节
 */
void Wireless_Send(void)
{
    uint8_t waveform[6 + UserDataLen] = {0};// 数据帧缓存
    uint8_t _cnt = 0;

//	memset(waveform, 0, sizeof(waveform));

    waveform[_cnt ++ ] = 0xAA;        		// 帧头
    waveform[_cnt ++ ] = 0xFF;        		// 目标地址
    waveform[_cnt ++ ] = 0xF1;				// 功能码ID
//	waveform[_cnt ++ ] = sizeof(waveform) - 6;// 有效数据长度
//	waveform[_cnt ++ ] = UserDataLen;
    waveform[_cnt ++ ] = 0;

	int16_t UserData_1 = 0;
	int16_t UserData_2 = 0;
//	int16_t UserData_1 = motorData[0].speed_rpm;
//	int16_t UserData_2 = motorData[0].given_current;

    // 数据区
	// 使用小端模式, 低字节在前
    waveform[_cnt ++ ] = BYTE0(UserData_1);   // 数据内容
    waveform[_cnt ++ ] = BYTE1(UserData_1);
    waveform[_cnt ++ ] = BYTE0(UserData_2);
    waveform[_cnt ++ ] = BYTE1(UserData_2);

    waveform[3] = _cnt - 4; 				// 写入有效数据字节数

    uint8_t sumcheck = 0;   				// 和校验SC
    uint8_t addcheck = 0;   				// 附加校验AC
    for(uint8_t i = 0; i < waveform[3] + 4; i ++ ) {
      sumcheck += waveform[i];      		// 从帧头开始, 一直到 data 区结束, 对每一字节进行累加操作, 只取低8位
      addcheck += sumcheck;         		// 计算和校验时, 每进行一字节的加法运算, 同时进行一次 sumcheck 的累加操作, 只取低8位
    }
    waveform[_cnt ++ ] = sumcheck;
    waveform[_cnt ++ ] = addcheck;

    // 串口发送数据
//	for (uint8_t j = 0; j < (6 + UserDataLen); j ++ ) {
//		HAL_UART_Transmit(&huart1, &waveform[j], sizeof(uint8_t), 1);
//	}
	HAL_UART_Transmit(&huart1, (uint8_t*)waveform, _cnt, 2);
}
