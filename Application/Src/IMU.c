/*
 * IMU.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "IMU.h"
#include "kalman.h"
#include "arm_math.h"

extern TIM_HandleTypeDef htim10;
extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler = NULL;

static uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
static uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
static uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
static uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};;

static volatile uint8_t gyro_update_flag = 0;
static volatile uint8_t accel_update_flag = 0;
static volatile uint8_t accel_temp_update_flag = 0;
static uint8_t imu_start_dma_flag = 0;

static float INS_Angle[3] = {0.0f, 0.0f, 0.0f};	// roll翻滚角 pitch俯仰角 yaw航向角/偏航角 deg
static float mag[3] = {0};

extern void imu_cmd_spi_dma(void);

/**
  * @brief		IMU任务
  */
void IMUTask(void const * argument)
{
	// 挂起调度器
//	vTaskSuspendAll();

	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;			// 设置SPI频率
	HAL_SPI_Init(&hspi1);
	SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
	imu_start_dma_flag = 1;

	HAL_TIM_Base_Start(&htim10);									// 开启定时器
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);						// 使IMU温控电阻定时器对应通道PWM输出

//	kalman_filter_init();
	// 唤醒调度器
//	xTaskResumeAll();

    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));	// 获取当前任务的任务句柄

	while(1)
	{
		// 等待SPI的DMA传输
        while (ulTaskNotifyTake(pdTRUE, (TickType_t)(2UL / portTICK_RATE_MS)) != pdPASS)
        {
        }

        // 获取IMU数据
        if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, IMU_Gyro);
        }
        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, IMU_Accel);
        }
        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &IMU_Temperate);
        }
//		BMI088_read(IMU_Gyro, IMU_Accel, &IMU_Temperate);

		// 数据滤波
		IMUDataFilter();
//		kalman_filter_test();

		// VRU 姿态解算
		MadgwickAHRSupdateIMU(IMUGyroFilter[0][0], IMUGyroFilter[0][1], IMUGyroFilter[0][2], IMUAccelFilter[0][0], IMUAccelFilter[0][1], IMUAccelFilter[0][2]);
		q2eul(INS_Angle, 1);
//		q2YawPitchRoll(INS_Angle + 0, INS_Angle + 1, INS_Angle + 2);

		// 控制IMU温度
		PID_Calc(&PID_IMU_Temp, IMU_Temperate, 29);
		if (PID_IMU_Temp.Output < 0.0f) {
			PID_IMU_Temp.Output = 0;
		}
		__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, (uint16_t)(PID_IMU_Temp.Output));

		// 向消息队列中填充数据
		if (messageQueueCreateFlag) {
			xQueueOverwrite(messageQueue[IMUANGLE], (void *)&INS_Angle);
			float IMUGyroFilter_tmp[3] = {IMUGyroFilter[0][0], IMUGyroFilter[0][1], IMUGyroFilter[0][2]};
			xQueueOverwrite(messageQueue[IMUGYRO], (void *)&IMUGyroFilter_tmp);
		}
	}
}

void imu_cmd_spi_dma(void)
{
	const uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	const uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	const uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // 开启陀螺仪DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    // 开启加速度计DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//TODO: KEY
//	if (GPIO_Pin == KEY_Pin) {
//		aRGB_led_show(0xFF00FF00);
//	}

    if (GPIO_Pin == IST8310_DRDY_Pin) {
        ist8310_read_mag(mag);
    }

    if (GPIO_Pin == INT1_ACCEL_Pin) {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    }
	if (GPIO_Pin == INT1_GYRO_Pin) {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    }

	if (GPIO_Pin == GPIO_PIN_0) {
        // 唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        //accel read over
        if (accel_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
