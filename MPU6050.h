/**
 * @file MPU.h
 * @author Kyuubi0323 (nguyenvankhoi8d@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define AD0 0
#if AD0
#define MPU6050_ADDRESS 0x69 //device address when AD0 = 1
#else
#define MPU6050_ADDRESS 0x68 //device address when AD0 = 0
#endif 

#ifndef _MPU6050_H_
#define _MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif


#define SELF_TEST_X                 0x0D
#define SELF_TEST_Y                 0x0E
#define SELF_TEST_Z                 0x0F
#define SELF_TEST_A                 0x10
#define SMPLRT_DIV                  0x19
#define CONFIG                      0x1A
#define GYRO_CONFIG                 0x1B
#define ACCEL_CONFIG                0x1C
#define FIFO_EN                     0x23
#define I2C_MST_CTRL                0x24
#define I2C_SLV0_ADDR               0x25
#define I2C_SLV0_REG                0x26
#define I2C_SLV0_CTRL               0x27
#define I2C_SLV1_ADDR               0x28
#define I2C_SLV1_REG                0x29
#define I2C_SLV1_CTRL               0x2A
#define I2C_SLV2_ADDR               0x2B
#define I2C_SLV2_REG                0x2C
#define I2C_SLV2_CTRL               0x2D
#define I2C_SLV3_ADDR               0x2E
#define I2C_SLV3_REG                0x2F
#define I2C_SLV3_CTRL               0x30
#define I2C_SLV4_ADDR               0x31
#define I2C_SLV4_REG                0x32
#define I2C_SLV4_DO                 0x33
#define I2C_SLV4_CTRL               0x34
#define I2C_SLV4_DI                 0x35
#define I2C_MST_STATUS              0x36
#define INT_PIN_CFG                 0x37
#define INT_ENABLE                  0x38
#define INT_STATUS                  0x3A
#define ACCEL_XOUT_H                0x3B
#define ACCEL_XOUT_L                0x3C
#define ACCEL_YOUT_H                0x3D
#define ACCEL_YOUT_L                0x3E
#define ACCEL_ZOUT_H                0x3F
#define ACCEL_ZOUT_L                0x40
#define TEMP_OUT_H                  0x41
#define TEMP_OUT_L                  0x42
#define GYRO_XOUT_H                 0x43
#define GYRO_XOUT_L                 0x44
#define GYRO_YOUT_H                 0x45
#define GYRO_YOUT_L                 0x46
#define GYRO_ZOUT_H                 0x47
#define GYRO_ZOUT_L                 0x48
#define EXT_SENS_DATA_00            0x49
#define EXT_SENS_DATA_01            0x4A
#define EXT_SENS_DATA_02            0x4B
#define EXT_SENS_DATA_03            0x4C
#define EXT_SENS_DATA_04            0x4D
#define EXT_SENS_DATA_05            0x4E
#define EXT_SENS_DATA_06            0x4F
#define EXT_SENS_DATA_07            0x50
#define EXT_SENS_DATA_08            0x51
#define EXT_SENS_DATA_09            0x52
#define EXT_SENS_DATA_10            0x53
#define EXT_SENS_DATA_11            0x54
#define EXT_SENS_DATA_12            0x55
#define EXT_SENS_DATA_13            0x56
#define EXT_SENS_DATA_14            0x57
#define EXT_SENS_DATA_15            0x58
#define EXT_SENS_DATA_16            0x59
#define EXT_SENS_DATA_17            0x5A
#define EXT_SENS_DATA_18            0x5B
#define EXT_SENS_DATA_19            0x5C
#define EXT_SENS_DATA_20            0x5D
#define EXT_SENS_DATA_21            0x5E
#define EXT_SENS_DATA_22            0x5F
#define EXT_SENS_DATA_23            0x60
#define I2C_SLV0_DO                 0x63
#define I2C_SLV1_DO                 0x64
#define I2C_SLV2_DO                 0x65
#define I2C_SLV3_DO                 0x66
#define I2C_MST_DELAY_CTRL          0x67
#define SIGNAL_PATH_RESET           0x68
#define USER_CTRL                   0x6A
#define PWr_MGMT_1                  0x6B
#define PWR_MGMT_2                  0x6C
#define FIFO_COUNTH                 0x72
#define FIFO_COUNTL                 0x73
#define FIFO_R_W                    0x74
#define WHO_AM_I                    0x75

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07
//cfg accel
#define MPU6050_ACCEL_FS_2              0x00
#define MPU6050_ACCEL_FS_4              0x01
#define MPU6050_ACCEL_FS_8              0x02
#define MPU6050_ACCEL_FS_16             0x03
//cfg gyro
#define MPU6050_GYRO_FS_250             0x00
#define MPU6050_GYRO_FS_500             0x01
#define MPU6050_GYRO_FS_1000            0x02
#define MPU6050_GYRO_FS_2000            0x03

#define PWR1_DEVICE_RESET               7
#define PWR1_SLEEP                      6
#define PWR1_CYCLE                      5
#define PWR1_TEMP_DIS                   3
#define PWR1_CLKSEL                     2
#define PWR1_CLKSEL_LENGTH              3

#define PWR2_LP_WAKE_CTRL_BIT           7
#define PWR2_LP_WAKE_CTRL_LENGTH        2
#define PWR2_STBY_XA                    5
#define PWR2_STBY_YA                    4
#define PWR2_STBY_ZA                    3
#define PWR2_STBY_XG                    2
#define PWR2_STBY_YG                    1
#define PWR2_STBY_ZG                    0

#define MPU6050_TC_PWR_MODE_BIT         7
#define MPU6050_TC_OFFSET_BIT           6
#define MPU6050_TC_OFFSET_LENGTH        6
#define MPU6050_TC_OTP_BNK_VLD_BIT      0

#define MPU6050_VDDIO_LEVEL_VLOGIC      0
#define MPU6050_VDDIO_LEVEL_VDD         1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT        2
#define MPU6050_CFG_DLPF_CFG_LENGTH     3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256             0x00
#define MPU6050_DLPF_BW_188             0x01
#define MPU6050_DLPF_BW_98              0x02
#define MPU6050_DLPF_BW_42              0x03
#define MPU6050_DLPF_BW_20              0x04
#define MPU6050_DLPF_BW_10              0x05
#define MPU6050_DLPF_BW_5               0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define 
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


void MPU6050_Init(I2C_HandleTypeDef *hi2c);
bool MPU6050__TestConnection(void);

void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_Read_Temp(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

/*
uint8_t MPU6050_GetFullScaleGyroRange();
void MPU6050_SetFullScaleGyroRange(uint8_t range);

uint88_t MPU6050_GetFullScaleAccelRange();
void MPU6050_SetFullScaleAccelRange(uint8_t range);

// PWR_MGMT_1 register
bool MPU6050_GetSleepModeStatus();
void MPU6050_SetSleepModeStatus(FunctionalState NewState);
void MPU6050_SetClockSource(uint8_t source);

// WHO_AM_I register
uint8_t MPU6050_GetDeviceID();

void MPU6050_GetRawAccelGyro(s16* AccelGyro);

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t *pBuffer, uint8_t writeAddr);
void MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t *pBuffer, uint8_t readAddr, uint16_t NumBytetoRead);
*/

#ifdef __cplusplus
}
#endif


#endif /* __MPU6050_H */
