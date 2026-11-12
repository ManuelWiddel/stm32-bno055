
bno055.c.txt
100%
#include "bno055.h"
#include <math.h>
#include <stdint.h>

/* =========================================================
 * BNO055 REGISTER DEFINITIONS
 * ========================================================= */

/* I2C address: 7-bit address 0x28, shifted for STM32 HAL */
#define BNO055_ADDR             (0x28 << 1)

/* Page 0 registers */
#define BNO055_CHIP_ID           0x00
#define BNO055_ACC_DATA_X_LSB    0x08
#define BNO055_GYR_DATA_X_LSB    0x14
#define BNO055_PAGE_ID           0x07
#define BNO055_UNIT_SEL          0x3B
#define BNO055_OPR_MODE          0x3D
#define BNO055_PWR_MODE          0x3E

/* Page 1 registers */
#define BNO055_ACC_CONFIG        0x08
#define BNO055_GYR_CONFIG_0      0x0A

/* Operating modes */
#define BNO055_CONFIG_MODE       0x00
#define BNO055_ACCGYRO_MODE      0x05

/* Power modes */
#define BNO055_POWER_NORMAL      0x00

/* Unit selection:
 * ACC: m/s^2
 * GYR: degrees/second
 * EUL: degrees
 */
#define BNO055_UNIT_DEFAULT      0x00

/* Conversion constants */
#define BNO055_ACC_LSB_PER_MS2   100.0f
#define BNO055_GYR_LSB_PER_DPS   16.0f

#define BNO055_GRAVITY           9.80665f
#define RAD_TO_DEG               57.2957795f

#define BNO055_I2C_TIMEOUT       100

/* =========================================================
 * PRIVATE I2C FUNCTIONS
 * ========================================================= */

static HAL_StatusTypeDef BNO055_WriteReg(
    BNO055_t *imu,
    uint8_t reg,
    uint8_t value)
{
    return HAL_I2C_Mem_Write(
        imu->hi2c,
        BNO055_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        BNO055_I2C_TIMEOUT
    );
}


static HAL_StatusTypeDef BNO055_ReadReg(
    BNO055_t *imu,
    uint8_t reg,
    uint8_t *value)
{
    return HAL_I2C_Mem_Read(
        imu->hi2c,
        BNO055_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        BNO055_I2C_TIMEOUT
    );
}


static HAL_StatusTypeDef BNO055_ReadRegs(
    BNO055_t *imu,
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    return HAL_I2C_Mem_Read(
        imu->hi2c,
        BNO055_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        length,
        BNO055_I2C_TIMEOUT
    );
}


/* Convert two little-endian bytes to signed int16_t */
static int16_t BNO055_ToInt16(uint8_t lsb, uint8_t msb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}


/* =========================================================
 * INITIALIZATION
 * ========================================================= */

HAL_StatusTypeDef BNO055_Init(
    BNO055_t *imu,
    I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    if (imu == NULL || hi2c == NULL)
    {
        return HAL_ERROR;
    }

    imu->hi2c = hi2c;

    /* Reset software variables */
    imu->accX = 0.0f;
    imu->accY = 0.0f;
    imu->accZ = 0.0f;

    imu->gyroX = 0.0f;
    imu->gyroY = 0.0f;
    imu->gyroZ = 0.0f;

    imu->accXoffset = 0.0f;
    imu->accYoffset = 0.0f;
    imu->accZoffset = 0.0f;

    imu->gyroXoffset = 0.0f;
    imu->gyroYoffset = 0.0f;
    imu->gyroZoffset = 0.0f;

    imu->roll = 0.0f;
    imu->pitch = 0.0f;
    imu->comp_tau = 0.0f;
    imu->comp_alpha = 0.98f;

    /* Enter CONFIGMODE */
    status = BNO055_WriteReg(
        imu,
        BNO055_OPR_MODE,
        BNO055_CONFIG_MODE
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Datasheet: transition to CONFIGMODE takes about 19 ms */
    HAL_Delay(25);

    /* Set normal power mode */
    status = BNO055_WriteReg(
        imu,
        BNO055_PWR_MODE,
        BNO055_POWER_NORMAL
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(10);

    /* Select register page 0 */
    status = BNO055_WriteReg(
        imu,
        BNO055_PAGE_ID,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Select default units:
     * Accelerometer = m/s^2
     * Gyroscope = degrees/second
     */
    status = BNO055_WriteReg(
        imu,
        BNO055_UNIT_SEL,
        BNO055_UNIT_DEFAULT
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Select register page 1 */
    status = BNO055_WriteReg(
        imu,
        BNO055_PAGE_ID,
        0x01
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Accelerometer configuration: 0x15
     *
     * Bits [4:2] = 101 -> 250 Hz bandwidth
     * Bits [1:0] = 01  -> +/- 4 g range
     */
    status = BNO055_WriteReg(
        imu,
        BNO055_ACC_CONFIG,
        0x15
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Gyroscope configuration: 0x00
     *
     * Bits [5:3] = 000 -> 523 Hz bandwidth
     * Bits [2:0] = 000 -> +/- 2000 degrees/second
     */
    status = BNO055_WriteReg(
        imu,
        BNO055_GYR_CONFIG_0,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Return to register page 0 */
    status = BNO055_WriteReg(
        imu,
        BNO055_PAGE_ID,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Select ACCGYRO mode */
    status = BNO055_WriteReg(
        imu,
        BNO055_OPR_MODE,
        BNO055_ACCGYRO_MODE
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Transition to operating mode takes about 7 ms */
    HAL_Delay(10);

    return HAL_OK;
}


/* =========================================================
 * READ ACCELEROMETER AND GYROSCOPE
 * ========================================================= */

HAL_StatusTypeDef BNO055_FetchData(BNO055_t *imu)
{
    uint8_t buffer[18];

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    HAL_StatusTypeDef status;

    if (imu == NULL || imu->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * Read 18 bytes starting at ACC_DATA_X_LSB (0x08).
     *
     * 0x08 - 0x0D : Accelerometer X/Y/Z
     * 0x0E - 0x13 : Magnetometer X/Y/Z
     * 0x14 - 0x19 : Gyroscope X/Y/Z
     *
     * Magnetometer data is ignored.
     */

    status = BNO055_ReadRegs(
        imu,
        BNO055_ACC_DATA_X_LSB,
        buffer,
        sizeof(buffer)
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Accelerometer raw values */
    ax = BNO055_ToInt16(buffer[0], buffer[1]);
    ay = BNO055_ToInt16(buffer[2], buffer[3]);
    az = BNO055_ToInt16(buffer[4], buffer[5]);

    /* Gyroscope raw values */
    gx = BNO055_ToInt16(buffer[12], buffer[13]);
    gy = BNO055_ToInt16(buffer[14], buffer[15]);
    gz = BNO055_ToInt16(buffer[16], buffer[17]);

    /*
     * Accelerometer:
     * 1 LSB = 0.01 m/s^2
     *
     * Convert to g:
     * acceleration_g = raw / (100 * 9.80665)
     */
    imu->accX =
        ((float)ax / (BNO055_ACC_LSB_PER_MS2 * BNO055_GRAVITY))
        - imu->accXoffset;

    imu->accY =
        ((float)ay / (BNO055_ACC_LSB_PER_MS2 * BNO055_GRAVITY))
        - imu->accYoffset;

    imu->accZ =
        ((float)az / (BNO055_ACC_LSB_PER_MS2 * BNO055_GRAVITY))
        - imu->accZoffset;

    /*
     * Gyroscope:
     * 1 LSB = 1/16 degrees/second
     */
    imu->gyroX =
        ((float)gx / BNO055_GYR_LSB_PER_DPS)
        - imu->gyroXoffset;

    imu->gyroY =
        ((float)gy / BNO055_GYR_LSB_PER_DPS)
        - imu->gyroYoffset;

    imu->gyroZ =
        ((float)gz / BNO055_GYR_LSB_PER_DPS)
        - imu->gyroZoffset;

    return HAL_OK;
}


/* =========================================================
 * SET GYROSCOPE BANDWIDTH
 * ========================================================= */

HAL_StatusTypeDef BNO055_SetGyroBandwidth(
    BNO055_t *imu,
    uint8_t bandwidth)
{
    uint8_t reg;
    HAL_StatusTypeDef status;

    /*
     * Valid bandwidth values:
     *
     * 0 = 523 Hz
     * 1 = 230 Hz
     * 2 = 116 Hz
     * 3 =  47 Hz
     * 4 =  23 Hz
     * 5 =  12 Hz
     * 6 =  64 Hz
     * 7 =  32 Hz
     */

    if (imu == NULL || imu->hi2c == NULL || bandwidth > 7)
    {
        return HAL_ERROR;
    }

    /* Enter CONFIGMODE */
    status = BNO055_WriteReg(
        imu,
        BNO055_OPR_MODE,
        BNO055_CONFIG_MODE
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(25);

    /* Select page 1 */
    status = BNO055_WriteReg(imu, BNO055_PAGE_ID, 0x01);

    if (status != HAL_OK)
    {
        return status;
    }

    /* Read current gyroscope configuration */
    status = BNO055_ReadReg(
        imu,
        BNO055_GYR_CONFIG_0,
        &reg
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Bits [5:3] = bandwidth
     * Preserve all other bits.
     */
    reg = (reg & 0xC7) | (uint8_t)(bandwidth << 3);

    status = BNO055_WriteReg(
        imu,
        BNO055_GYR_CONFIG_0,
        reg
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Return to page 0 */
    status = BNO055_WriteReg(imu, BNO055_PAGE_ID, 0x00);

    if (status != HAL_OK)
    {
        return status;
    }

    /* Return to ACCGYRO mode */
    status = BNO055_WriteReg(
        imu,
        BNO055_OPR_MODE,
        BNO055_ACCGYRO_MODE
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(10);

    return HAL_OK;
}


/* =========================================================
 * SET ACCELEROMETER BANDWIDTH
 * ========================================================= */

HAL_StatusTypeDef BNO055_SetAccBandwidth(
    BNO055_t *imu,
    uint8_t bandwidth)
{
    uint8_t reg;
    HAL_StatusTypeDef status;

    /*
     * Valid bandwidth values:
     *
     * 0 =   7.81 Hz
     * 1 =  15.63 Hz
     * 2 =  31.25 Hz
     * 3 =  62.50 Hz
     * 4 = 125.00 Hz
     * 5 = 250.00 Hz
     * 6 = 500.00 Hz
     * 7 = 1000.0 Hz
     */

    if (imu == NULL || imu->hi2c == NULL || bandwidth > 7)
    {
        return HAL_ERROR;
    }

    /* Enter CONFIGMODE */
    status = BNO055_WriteReg(
        imu,
        BNO055_OPR_MODE,
        BNO055_CONFIG_MODE
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(25);

    /* Select page 1 */
    status = BNO055_WriteReg(imu, BNO055_PAGE_ID, 0x01);

    if (status != HAL_OK)
    {
        return status;
    }

    /* Read current accelerometer configuration */
    status = BNO055_ReadReg(
        imu,
        BNO055_ACC_CONFIG,
        &reg
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Bits [4:2] = bandwidth
     * Preserve range and power-mode bits.
     */
    reg = (reg & 0xE3) | (uint8_t)(bandwidth << 2);

    status = BNO055_WriteReg(
        imu,
        BNO055_ACC_CONFIG,
        reg
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Return to page 0 */
    status = BNO055_WriteReg(imu, BNO055_PAGE_ID, 0x00);

    if (status != HAL_OK)
    {
        return status;
    }

    /* Return to ACCGYRO mode */
    status = BNO055_WriteReg(
        imu,
        BNO055_OPR_MODE,
        BNO055_ACCGYRO_MODE
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(10);

    return HAL_OK;
}


/* =========================================================
 * COMPLEMENTARY FILTER INITIALIZATION
 * ========================================================= */

void BNO055_CompFilterInit(BNO055_t *imu, float tau)
{
    if (imu == NULL || tau <= 0.0f)
    {
        return;
    }

    imu->comp_tau = tau;

    imu->roll = atan2f(imu->accY, imu->accZ) * RAD_TO_DEG;

    imu->pitch = atan2f(
        -imu->accX,
        sqrtf(imu->accY * imu->accY + imu->accZ * imu->accZ)
    ) * RAD_TO_DEG;
}


/* =========================================================
 * COMPLEMENTARY FILTER UPDATE
 * ========================================================= */

void BNO055_CompFilterUpdate(
    BNO055_t *imu,
    float dt,
    uint8_t accel_update)
{
    float alpha;
    float roll_acc;
    float pitch_acc;

    if (imu == NULL || dt <= 0.0f || imu->comp_tau <= 0.0f)
    {
        return;
    }

    /* Calculate coefficient from time constant and actual dt */
    alpha = imu->comp_tau / (imu->comp_tau + dt);

    /* Integrate gyroscope */
    imu->roll += imu->gyroX * dt;
    imu->pitch += imu->gyroY * dt;

    /* Accelerometer correction */
    if (accel_update)
    {
        roll_acc = atan2f(
            imu->accY,
            imu->accZ
        ) * RAD_TO_DEG;

        pitch_acc = atan2f(
            -imu->accX,
            sqrtf(
                imu->accY * imu->accY +
                imu->accZ * imu->accZ
            )
        ) * RAD_TO_DEG;

        imu->roll =
            alpha * imu->roll +
            (1.0f - alpha) * roll_acc;

        imu->pitch =
            alpha * imu->pitch +
            (1.0f - alpha) * pitch_acc;
    }
}


/* =========================================================
 * GYROSCOPE AND ACCELEROMETER OFFSET CALIBRATION
 * ========================================================= */

HAL_StatusTypeDef BNO055_CalcOffsets(
    BNO055_t *imu,
    uint8_t calc_gyro,
    uint8_t calc_acc)
{
    const uint16_t samples = 1000;

    float gyroX_sum = 0.0f;
    float gyroY_sum = 0.0f;
    float gyroZ_sum = 0.0f;

    float accX_sum = 0.0f;
    float accY_sum = 0.0f;
    float accZ_sum = 0.0f;

    uint16_t valid_samples = 0;

    if (imu == NULL || imu->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    /* Clear offsets before calibration */
    imu->gyroXoffset = 0.0f;
    imu->gyroYoffset = 0.0f;
    imu->gyroZoffset = 0.0f;

    imu->accXoffset = 0.0f;
    imu->accYoffset = 0.0f;
    imu->accZoffset = 0.0f;

    /*
     * Keep the sensor completely stationary during
     * calibration.
     *
     * For accelerometer calibration, place the board
     * flat with its +Z axis pointing upward.
     */

    for (uint16_t i = 0; i < samples; i++)
    {
        if (BNO055_FetchData(imu) != HAL_OK)
        {
            HAL_Delay(2);
            continue;
        }

        valid_samples++;

        if (calc_gyro)
        {
            gyroX_sum += imu->gyroX;
            gyroY_sum += imu->gyroY;
            gyroZ_sum += imu->gyroZ;
        }

        if (calc_acc)
        {
            accX_sum += imu->accX;
            accY_sum += imu->accY;
            accZ_sum += imu->accZ;
        }

        HAL_Delay(2);
    }

    if (valid_samples == 0)
    {
        return HAL_ERROR;
    }

    if (calc_gyro)
    {
        imu->gyroXoffset = gyroX_sum / valid_samples;
        imu->gyroYoffset = gyroY_sum / valid_samples;
        imu->gyroZoffset = gyroZ_sum / valid_samples;
    }

    if (calc_acc)
    {
        /*
         * Assumes:
         * X = 0 g
         * Y = 0 g
         * Z = +1 g
         *
         * This is a simple flat-position calibration,
         * not a full multi-position accelerometer calibration.
         */
        imu->accXoffset = accX_sum / valid_samples;
        imu->accYoffset = accY_sum / valid_samples;
        imu->accZoffset = (accZ_sum / valid_samples) - 1.0f;
    }

    return HAL_OK;
}
Displaying bno055.h.txt.
