/* mpu9265.h — MPU-9265 9-axis IMU driver (I2C) for EFR32xG28
 *
 * Wiring (BRD2705A):
 *   MPU VCC  → VMCU (3.3V)
 *   MPU GND  → GND
 *   MPU SDA  → PC03 (I2C0_SDA)
 *   MPU SCL  → PC04 (I2C0_SCL)
 *   MPU AD0  → GND (I2C address = 0x68)
 */
#ifndef MPU9265_H
#define MPU9265_H

#include <stdint.h>
#include <stdbool.h>

/* I2C address (AD0 = GND) */
#define MPU9265_I2C_ADDR        0x68u

/* Register addresses */
#define MPU9265_REG_WHO_AM_I    0x75u
#define MPU9265_REG_PWR_MGMT_1  0x6Bu
#define MPU9265_REG_ACCEL_XOUT_H 0x3Bu
#define MPU9265_REG_GYRO_XOUT_H  0x43u

/* Expected WHO_AM_I values — board labeled MPU-9265 but chip reports
 * MPU-6500 ID (0x70). This is common on cheap breakout boards: the
 * magnetometer (AK8963) may be absent, but accel+gyro work normally. */
#define MPU9265_WHO_AM_I_MPU6500  0x70u
#define MPU9265_WHO_AM_I_MPU9250  0x71u
#define MPU9265_WHO_AM_I_MPU9255  0x73u

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
} MPU9265_Data;

/* Initialize sensor — wakes from sleep mode. Returns true on success. */
bool MPU9265_Init(void);

/* Read WHO_AM_I register — for connectivity test */
bool MPU9265_ReadWhoAmI(uint8_t *who_am_i);

/* AK8963 magnetometer (only present on real MPU-9250/9255) */
#define AK8963_I2C_ADDR          0x0Cu
#define AK8963_REG_WHO_AM_I      0x00u
#define AK8963_WHO_AM_I_VALUE    0x48u

#define MPU9265_REG_INT_PIN_CFG  0x37u
#define MPU9265_REG_USER_CTRL    0x6Au

/* Test for magnetometer presence via I2C bypass mode.
 * Returns true if AK8963 responds with correct WHO_AM_I (0x48). */
bool MPU9265_TestMagnetometer(void);

/* Read accelerometer + gyroscope raw data */
bool MPU9265_ReadData(MPU9265_Data *data);

#endif /* MPU9265_H */
