/*
 *  ch100.h
 *
 *  Created on: 2021年11月5日
 *      Author: LBQ
 */

#include "ch100.h"

//extern UART_HandleTypeDef huart3;
//extern DMA_HandleTypeDef hdma_usart3_rx;

// Statement
#define CH100_DATA_PRE		(0x5A)
#define CH100_DATA_TYPE		(0xA5)
#define CH100_DATA_LABEL1	(0x91)
#define CH100_DATA_ID		1
#define CH100_DATA_HEADLEN	6
#define CH100_DATA_LEN		82

//static int16_t	CU2(uint8_t *p) {int16_t	u; memcpy(&u, p, 2); return u;}
//static int32_t	CU4(uint8_t *p) {int32_t	u; memcpy(&u, p, 4); return u;}
static float	CR4(uint8_t *p) {float		r; memcpy(&r, p, 4); return r;}

// Define
static uint8_t CH100_buf[CH100_DATA_LEN] = {0};	// 帧缓存
static float CH100_eul[3] = {0};				// Pitch Yaw Roll
static float CH100_acc[3] = {0};				// XYZ加速度 1G
static float CH100_gyro[3] = {0};				// XYZ角速度 deg/s

#if DEBUGMODE
	static Frequency_t CH100_Frequency;
	static float CH100_Freq = 0.000f;			// 有效接收数据频率
#endif

/**
  * @brief			CH100初始化
  */
void CH100_USART_Init(void)
{
	if (HAL_UART_Receive_DMA((UART_HandleTypeDef *)&huart3, (uint8_t *)CH100_buf, (uint16_t)CH100_DATA_LEN) != HAL_OK) {	// 串口DMA中断
		Error_Handler();
	}
}

/**
  * @brief			CRC校验
  */
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
	uint32_t crc = *currectCrc;
	uint32_t j;
	for (j = 0; j < lengthInBytes; ++j) {
		uint32_t i;
		uint32_t byte = src[j];
		crc ^= byte << 8;
		for (i = 0; i < 8; ++i) {
			uint32_t temp = crc << 1;
			if (crc & 0x8000) {
				temp ^= 0x1021;
			}
			crc = temp;
		}
	}
	*currectCrc = crc;
}

/**
  * @brief			CH100串口通讯协议解析
  * @param[in]		data
  */
void CH100_Proc(uint8_t data)
{
	static uint16_t ch100_nbyte = 0;	// 第几帧
    static uint16_t payload_len = 0;	// 数据帧长度
	static uint16_t ch100_crc0 = 0;		// 数据帧中CRC
	uint16_t ch100_crc = 0;				// 收到数据计算CRC

    if (ch100_nbyte == 0 && data == CH100_DATA_PRE) {
    	// 校验并记录帧头
    	CH100_buf[0] = CH100_DATA_PRE;
        ch100_nbyte = 1;
    } else if (ch100_nbyte == 1 && data == CH100_DATA_TYPE) {
    	// 校验并记录帧头
    	CH100_buf[1] = CH100_DATA_TYPE;
        ch100_nbyte = 2;
    } else if (ch100_nbyte == 2) {
    	CH100_buf[2] = data;
        ch100_nbyte = 3;
    } else if (ch100_nbyte == 3) {
    	CH100_buf[3] = data;
    	payload_len = CH100_buf[2] | (CH100_buf[3] << 8);
    	ch100_nbyte = 4;
    	if (payload_len != (CH100_DATA_LEN - CH100_DATA_HEADLEN)) {
    		// 校验数据包长度
            ch100_nbyte = 0;
            memset(&CH100_buf, 0, sizeof(CH100_buf));
    	}
    } else if (ch100_nbyte >= 4 && ch100_nbyte <= payload_len + 6) {
    	// 存储帧数据
    	CH100_buf[ch100_nbyte] = data;
    	if (ch100_nbyte == 5) {
    		// 取出帧中携带CRC
    		ch100_crc0 = CH100_buf[4] | (CH100_buf[5] << 8);
    	}
    	if (ch100_nbyte == 6 && CH100_buf[ch100_nbyte] != CH100_DATA_LABEL1) {
    		// 校验数据标签
            ch100_nbyte = 0;
            memset(&CH100_buf, 0, sizeof(CH100_buf));
    	}
    	if (ch100_nbyte == 7 && CH100_buf[ch100_nbyte] != CH100_DATA_ID) {
    		// 校验模块ID
            ch100_nbyte = 0;
            memset(&CH100_buf, 0, sizeof(CH100_buf));
    	}
    	if (ch100_nbyte != 0) {
    		ch100_nbyte++;
    	}
    } else if (ch100_nbyte == CH100_DATA_LEN + 1) {
    	// 计算CRC
    	crc16_update(&ch100_crc, CH100_buf, 4);
		crc16_update(&ch100_crc, CH100_buf + 6, payload_len);
		// CRC校验
		if (ch100_crc == ch100_crc0) {
			// 解析串口通讯协议
			CH100_eul[2] = CR4(CH100_buf + 54);		// Roll		横滚角
			CH100_eul[0] = CR4(CH100_buf + 58);		// Pitch	俯仰角
			CH100_eul[1] = CR4(CH100_buf + 62);		// Yaw		航向角/偏航角

			#if DEBUGMODE
				// 计算有效接收数据频率
				CH100_Freq = GetFrequency(&CH100_Frequency);
			#endif
		}
    	ch100_nbyte = 0;
    	memset(&CH100_buf, 0, sizeof(CH100_buf));
    } else {
        ch100_nbyte = 0;
        memset(&CH100_buf, 0, sizeof(CH100_buf));
    }
}

/**
  * @brief			CH100串口通讯协议解析
  * @param[in]		data
  */
uint8_t CH100_ProcAll(uint8_t *data)
{
	uint16_t ch100_crc0 = 0;		// 数据帧中CRC
	uint16_t ch100_crc = 0;			// 收到数据计算CRC

	// 取出帧中携带CRC
	ch100_crc0 =  CH100_buf[4] | (CH100_buf[5] << 8);
	// 计算CRC
	crc16_update(&ch100_crc, CH100_buf, 4);
	crc16_update(&ch100_crc, CH100_buf + 6, CH100_DATA_LEN - CH100_DATA_HEADLEN);
	// CRC校验
	if (ch100_crc == ch100_crc0) {
		// 解析串口通讯协议
		CH100_acc[0] =  CR4(CH100_buf + 18);	// 加速度XYZ
		CH100_acc[1] =  CR4(CH100_buf + 22);
		CH100_acc[2] =  CR4(CH100_buf + 26);
		CH100_gyro[0] = CR4(CH100_buf + 30);	// 角速度XYZ
		CH100_gyro[1] = CR4(CH100_buf + 34);
		CH100_gyro[2] = CR4(CH100_buf + 38);
		CH100_eul[2] = CR4(CH100_buf + 54);		// Roll		横滚角
		CH100_eul[0] = CR4(CH100_buf + 58);		// Pitch	俯仰角
		CH100_eul[1] = CR4(CH100_buf + 62);		// Yaw		航向角/偏航角

		#if DEBUGMODE
			// 计算有效接收数据频率
			CH100_Freq = GetFrequency(&CH100_Frequency);
		#endif

		return 1;
	} else {
		memset(&CH100_buf, 0, sizeof(CH100_buf));
		return 0;
	}
}

/**
  * @brief			接收中断服务回调函数
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;	// 不请求上下文切换
//
//	if (huart->Instance == USART3){
//		if (CH100_ProcAll(CH100_buf) == 1) {
//			// 向消息队列中填充数据
//			xQueueOverwriteFromISR(messageQueue[ANGLE], (void *)&CH100_eul, &xHigherPriorityTaskWoken);
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//		} else {
//			// 接收错位重启DMA
//			HAL_UART_DMAStop(&huart3);
//			HAL_Delay(3);
//			HAL_UART_Receive_DMA((UART_HandleTypeDef *)&huart3, (uint8_t *)CH100_buf, (uint16_t)CH100_DATA_LEN);
//		}
//	}
//}

