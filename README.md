# stm32-bno055

STM32 HAL driver for Bosch BNO055 accelerometer and gyroscope data acquisition, calibration, bandwidth configuration, and complementary-filter orientation estimation.

---

## Project Overview

This project provides a lightweight STM32 HAL driver for the Bosch BNO055 intelligent IMU.

The driver operates the sensor in **ACCGYRO mode**, allowing direct access to raw accelerometer and gyroscope measurements. Roll and pitch angles can be estimated using the included complementary filter implementation.

The library is designed for integration into STM32CubeIDE projects and follows a simple `Core/Inc` and `Core/Src` project structure.

---

## Key Features

- STM32 HAL I2C communication
- Accelerometer measurements in **g**
- Gyroscope measurements in **degrees per second**
- Configurable accelerometer bandwidth
- Configurable gyroscope bandwidth
- Sensor offset calibration
- Complementary filter for roll and pitch estimation
- Lightweight implementation with no external dependencies
- Compatible with STM32CubeIDE and STM32 HAL

---

## Hardware

| Component | Specification |
|------------|------------|
| Sensor | Bosch BNO055 |
| Interface | I2C |
| Address | 0x28 (default) |
| Microcontroller | STM32 with HAL I2C support |

---

## Software

- Programming Language: C
- IDE: STM32CubeIDE / STM32CubeMX
- Framework: STM32 HAL
- Communication: I2C
- Standard Library: math.h

---

## Repository Structure

```text
stm32-bno055/
├── Core/
│   ├── Inc/
│   │   └── bno055.h
│   └── Src/
│       └── bno055.c
├── .gitignore
├── LICENSE
└── README.md
```

---

## Installation

Copy the driver files into your STM32 project:

```text
Core/Inc/bno055.h
Core/Src/bno055.c
```

Include the driver in your application:

```c
#include "bno055.h"
```

---

## Initialization

Create an IMU object and initialize the sensor:

```c
BNO055_t imu;

if (BNO055_Init(&imu, &hi2c1) != HAL_OK)
{
    Error_Handler();
}
```

---

## Reading Sensor Data

Fetch the latest accelerometer and gyroscope values:

```c
if (BNO055_FetchData(&imu) == HAL_OK)
{
    float accX = imu.accX;
    float accY = imu.accY;
    float accZ = imu.accZ;

    float gyroX = imu.gyroX;
    float gyroY = imu.gyroY;
    float gyroZ = imu.gyroZ;
}
```

---

## Calibration

For best results, keep the sensor completely stationary during calibration.

For accelerometer calibration, place the board flat with the **positive Z-axis pointing upward**.

```c
if (BNO055_CalcOffsets(&imu, 1, 1) != HAL_OK)
{
    Error_Handler();
}
```

### Gyroscope Only

```c
BNO055_CalcOffsets(&imu, 1, 0);
```

### Accelerometer Only

```c
BNO055_CalcOffsets(&imu, 0, 1);
```

---

## Complementary Filter

Initialize the filter after reading valid accelerometer data:

```c
BNO055_FetchData(&imu);

BNO055_CompFilterInit(&imu, 0.5f);
```

Update the filter periodically:

```c
BNO055_FetchData(&imu);

BNO055_CompFilterUpdate(&imu, 0.01f, 1);
```

Access the estimated orientation:

```c
float roll  = imu.roll;
float pitch = imu.pitch;
```

---

## Bandwidth Configuration

### Set Gyroscope Bandwidth

```c
BNO055_SetGyroBandwidth(&imu, 0);
```

| Value | Bandwidth |
|---------|---------|
| 0 | 523 Hz |
| 1 | 230 Hz |
| 2 | 116 Hz |
| 3 | 47 Hz |
| 4 | 23 Hz |
| 5 | 12 Hz |
| 6 | 64 Hz |
| 7 | 32 Hz |

### Set Accelerometer Bandwidth

```c
BNO055_SetAccBandwidth(&imu, 5);
```

| Value | Bandwidth |
|---------|---------|
| 0 | 7.81 Hz |
| 1 | 15.63 Hz |
| 2 | 31.25 Hz |
| 3 | 62.50 Hz |
| 4 | 125 Hz |
| 5 | 250 Hz |
| 6 | 500 Hz |
| 7 | 1000 Hz |

---

## Notes

- The driver operates the BNO055 in ACCGYRO mode.
- Roll and pitch are estimated using a complementary filter running on the STM32.
- Yaw estimation is not provided.
- Magnetometer measurements are currently ignored.
- Offset calibration is a simple stationary calibration procedure.
- Accelerometer calibration assumes a flat sensor position (+Z upward).

---

## Tested Hardware

| Board | Status |
|---------|---------|
| STM32F411CEU6 Blackpill | ✅ Tested |

Additional hardware validation is welcome.

---

## Future Improvements

- Six-position accelerometer calibration
- Doxygen documentation
- Example project

---
