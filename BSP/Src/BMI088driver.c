/*
 * BMI088driver.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "BMI088driver.h"
#include "BMI088reg.h"

/*************** define ***************/
#define BMI088_WRITE_ACCEL_REG_NUM	6
#define BMI088_WRITE_GYRO_REG_NUM	6


#define BMI088_ACCEL_3G_SEN			0.0008974358974f
#define BMI088_ACCEL_6G_SEN			0.00179443359375f
#define BMI088_ACCEL_12G_SEN		0.0035888671875f
#define BMI088_ACCEL_24G_SEN		0.007177734375f

#define BMI088_GYRO_2000_SEN		0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN		0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN			0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN			0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN			0.000066579027251980956150958662738366f

#define BMI088_TEMP_FACTOR			0.125f
#define BMI088_TEMP_OFFSET			23.0f

#define BMI088_LONG_DELAY_TIME		80
#define BMI088_COM_WAIT_SENSOR_TIME	150

//float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
//float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
float BMI088_ACCEL_SEN = BMI088_ACCEL_24G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

/*************** CS/SS 片选线 ***************/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/*************** 宏定义读取&写入陀螺仪和加速度计 ***************/
#define BMI088_accel_write_single_reg(reg, data) 		\
    {                                            		\
        BMI088_ACCEL_NS_L();                     		\
        BMI088_write_single_reg((reg), (data));  		\
        BMI088_ACCEL_NS_H();                     		\
    }
#define BMI088_accel_read_single_reg(reg, data) 		\
    {                                           		\
        BMI088_ACCEL_NS_L();                    		\
        BMI088_read_write_byte((reg) | 0x80);   		\
        BMI088_read_write_byte(0x55);           		\
        (data) = BMI088_read_write_byte(0x55);  		\
        BMI088_ACCEL_NS_H();                    		\
    }
#define BMI088_accel_read_muli_reg(reg, data, len) 		\
    {                                             		\
        BMI088_ACCEL_NS_L();                       		\
        BMI088_read_write_byte((reg) | 0x80);      		\
        BMI088_read_muli_reg(reg, data, len);      		\
        BMI088_ACCEL_NS_H();                       		\
    }
// 片选信号选中加速度计 (低电平有效)
// 然后用SPI将加速度计数据寄存器中的数据读入buf中
// 通信结束 主设备拉高SS/CS片选
#define BMI088_accel_write_muli_reg(reg, data, len)		\
	{ 													\
		BMI088_ACCEL_NS_L(); 							\
		BMI088_write_muli_reg(reg, data, len);			\
		BMI088_ACCEL_NS_H(); 							\
	}
#define BMI088_gyro_write_single_reg(reg, data) 		\
    {                                           		\
        BMI088_GYRO_NS_L();                     		\
        BMI088_write_single_reg((reg), (data)); 		\
        BMI088_GYRO_NS_H();                     		\
    }
#define BMI088_gyro_read_single_reg(reg, data)  		\
    {                                           		\
        BMI088_GYRO_NS_L();                     		\
        BMI088_read_single_reg((reg), &(data)); 		\
        BMI088_GYRO_NS_H();                     		\
    }
#define BMI088_gyro_read_muli_reg(reg, data, len)   	\
    {                                               	\
        BMI088_GYRO_NS_L();                         	\
        BMI088_read_muli_reg((reg), (data), (len)); 	\
        BMI088_GYRO_NS_H();                         	\
    }
// 片选信号选中陀螺仪 (低电平有效)
// 然后用SPI将角速度计的ID和数据寄存器中的数据读入buf
// 通信结束 主设备拉高SS/CS片选
#define BMI088_gyro_write_muli_reg(reg,  data, len)		\
	{													\
		BMI088_GYRO_NS_L();								\
		BMI088_write_muli_reg((reg), (data), (len));	\
		BMI088_GYRO_NS_H();								\
	}

/*************** 寄存器和寄存器值 ***************/
const uint8_t write_BMI088_accel_reg_data[BMI088_WRITE_ACCEL_REG_NUM][2] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE},
//		{BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set},
		{BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set},
//		{BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT}
};

const uint8_t write_BMI088_gyro_reg_data[BMI088_WRITE_GYRO_REG_NUM][2] = {
//		{BMI088_GYRO_RANGE, BMI088_GYRO_2000},
		{BMI088_GYRO_RANGE, BMI088_GYRO_2000},
//		{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set},
		{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3}
};

/*************** 初始化 ***************/
void bmi088_accel_init(void)
{
    uint8_t res = 0;

    // 首先读取ID寄存器
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 使用软件重置寄存器对加速度计所有寄存器的值进行恢复为默认值0
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // 重置后再读取ID寄存器
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 确认ID正确 判断通信是否正常
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        
    }

    // 设置传感器的工作状态
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data[write_reg_num][0], write_BMI088_accel_reg_data[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        // 校验
        if (res != write_BMI088_accel_reg_data[write_reg_num][1])
        {

        }
    }
}

void bmi088_gyro_init(void)
{
    uint8_t res = 0;

    // 首先读取ID寄存器
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 使用软件重置寄存器对加速度计所有寄存器的值进行恢复为默认值0
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // 重置后再读取ID寄存器
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 确认ID正确
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {

    }

    // 设置传感器的工作状态
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data[write_reg_num][0], write_BMI088_gyro_reg_data[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        // 校验
        if (res != write_BMI088_gyro_reg_data[write_reg_num][1])
        {

        }
    }
}

/*************** 数据读取 ***************/
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    uint8_t buf[8] = {0};
    int16_t bmi088_raw_temp = 0;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
	
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088_gyro_read_over(uint8_t *rx_buf, fp32 gyro[3])
{
    int16_t bmi088_raw_temp = 0;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}

void BMI088_accel_read_over(uint8_t *rx_buf, fp32 accel[3])
{
    int16_t bmi088_raw_temp = 0;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
}

void BMI088_temperature_read_over(uint8_t *rx_buf, fp32 *temperate)
{
    int16_t bmi088_raw_temp = 0;
    bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));
    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

uint32_t get_BMI088_sensor_time(void)
{
    uint32_t sensor_time = 0;
    uint8_t buf[3] = {0};
    BMI088_accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);

    sensor_time = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

    return sensor_time;
}

void BMI088_time_read_over(uint8_t *rx_buf, fp32 *time)
{
	uint32_t sensor_time = 0;
	sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
	*time = sensor_time * 39.0625f;
}

#if defined(BMI088_USE_IIC)

#endif
