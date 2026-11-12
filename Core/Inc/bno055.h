#ifndef BNO055_H
#define BNO055_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

#define BNO055_ADDR       (0x28U << 1U)

#define BNO055_PI         3.14159265f
#define BNO055_RAD_TO_DEG 57.2957795f

typedef struct
{
    I2C_HandleTypeDef *hi2c;

    /* Accelerometer data [g] */
    float accX;
    float accY;
    float accZ;

    /* Gyroscope data [deg/s] */
    float gyroX;
    float gyroY;
    float gyroZ;

    /* Calibration offsets */
    float gyroXoffset;
    float gyroYoffset;
    float gyroZoffset;

    float accXoffset;
    float accYoffset;
    float accZoffset;

    /* Complementary filter */
    float comp_tau;
    float comp_alpha;

    float roll;
    float pitch;

} BNO055_t;

HAL_StatusTypeDef BNO055_Init(BNO055_t *imu,
                              I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef BNO055_FetchData(BNO055_t *imu);

HAL_StatusTypeDef BNO055_SetGyroBandwidth(BNO055_t *imu,
                                          uint8_t bandwidth);

HAL_StatusTypeDef BNO055_SetAccBandwidth(BNO055_t *imu,
                                         uint8_t bandwidth);

void BNO055_CompFilterInit(BNO055_t *imu,
                           float tau);

void BNO055_CompFilterUpdate(BNO055_t *imu,
                             float dt,
                             uint8_t accel_update);

HAL_StatusTypeDef BNO055_CalcOffsets(BNO055_t *imu,
                                     uint8_t calc_gyro,
                                     uint8_t calc_acc);

#endif /* BNO055_H */
