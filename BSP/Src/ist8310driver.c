/*
 * ist8310driver.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "ist8310driver.h"

#define IST8310_WHO_AM_I		0x00	// ist8310 "who am I "
#define IST8310_WHO_AM_I_VALUE	0x10	// device ID

#define IST8310_DATA_READY_BIT	2
#define IST8310_WRITE_REG_NUM	4

#define MAG_SEN					0.3f	// 原始整型数据转化成 单位ut

// 设置RSTN引脚为1
void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
}
// 设置RSTN引脚为0
void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
}

// 第一列:IST8310的寄存器
// 第二列:需要写入的寄存器值
const uint8_t ist8310_write_reg_data[IST8310_WRITE_REG_NUM][2] = {
        {0x0B, 0x08},	// 中断寄存器, 开启中断, 并且设置DRDY管脚中断时为低电平
//		{0x41, 0x09},	// 采样次数寄存器, 平均采样两次
		{0x41, 0x12},	// 平均采样4次 8次为0x1b 16次为0x24
        {0x42, 0xC0},	// 必须是0xC0
        {0x0A, 0x0B}	// 连续测量模式输出频率200Hz
};

void ist8310_init(void)
{
	const uint8_t wait_time = 150;
	const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    // 通过IST8310重启管脚进行重启
    ist8310_RST_L();
    HAL_Delay(sleepTime);
    ist8310_RST_H();
    HAL_Delay(sleepTime);

    // 取ID寄存器
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    // 确认ID正确 判断通信是否正常
    if (res != IST8310_WHO_AM_I_VALUE)
    {

    }

    // 设置IST8310工作状态
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum ++ ) {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data[writeNum][0], ist8310_write_reg_data[writeNum][1]);
        Delay_us(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data[writeNum][0]);
        Delay_us(wait_time);
        // 校验
        if (res != ist8310_write_reg_data[writeNum][1])
        {

        }
    }
}

/**
  * @brief          如果已经通过I2C的DMA方式读取到了从STAT1到DATAXL共7个数据，可以使用这个函数进行处理成单位是uT的磁场强度数据
  * @param[in]      status_buf:数据指针,从STAT1(0x02) 寄存器到 DATAXL(0x08)寄存器
  * @param[out]
  * @retval         none
  */
void ist8310_read_over(uint8_t *status_buf, fp32 mag[3], uint8_t magStatus)
{
	// 通过判断stat1寄存器值判断有没有新的数据产生
    if (status_buf[0] & 0x01) {
        int16_t temp_ist8310_data = 0;
        magStatus |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else {
    	magStatus &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

/**
  * @brief          通过读取磁场数据
  * @param[out]     磁场数组
  * @retval         none
  */
void ist8310_read_mag(fp32 mag[3])
{
    uint8_t buf[6] = {0};
    int16_t temp_ist8310_data = 0;
    // I2C读取多个字节
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}
