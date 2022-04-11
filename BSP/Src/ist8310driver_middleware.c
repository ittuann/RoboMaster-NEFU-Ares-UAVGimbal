/*
 * ist8310driver_middleWare.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "ist8310driver_middleWare.h"

extern I2C_HandleTypeDef hi2c3;

/**
  * @brief          读取IST8310的一个字节通过I2C
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    // I2C句柄 I2C从机地址 寄存器地址 寄存器地址增加大小 数据指针 数据长度 超时时间
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 2);	// 从I2C设备的寄存器读取数据
    return res;
}

/**
  * @brief          通过I2C写入一个字节到IST8310的寄存器中
  * @param[in]      寄存器地址
  * @param[in]      写入值
  * @retval         none
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
	// I2C句柄 I2C从机地址 寄存器地址 寄存器地址增加大小 数据指针 数据长度 超时时间
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 2);	// 向I2C设备的寄存器写入数据
}

/**
  * @brief          读取IST8310的多个字节通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  * @retval         none
  */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}

/**
  * @brief          写入多个字节到IST8310的寄存器通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  * @retval         none
  */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT, data, len, 10);
}
