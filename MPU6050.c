#include "MPU6050.h"
#include "stm32f10x_i2c.h"




void MPU6050_Init(void)
{
    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_SetSleepModeStatus(DISABLE);
}