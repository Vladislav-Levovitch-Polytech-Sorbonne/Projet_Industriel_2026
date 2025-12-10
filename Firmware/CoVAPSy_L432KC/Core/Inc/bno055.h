/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055.h
  * @brief          : BNO055 9-axis IMU sensor driver header (IMU Mode)
  * @author         : CoVAPSy Team
  * @date           : 2025-12-10
  ******************************************************************************
  * @attention
  *
  * This driver supports the DFRobot SEN0253 (BNO055+BMP280) 10-axis sensor.
  * Configured for IMU mode (0x08) for indoor autonomous vehicle navigation.
  *
  * Features:
  *   - Operating Mode: IMU (0x08) - Accelerometer + Gyroscope only
  *   - Outputs: Quaternion, Euler angles, Linear acceleration, Angular velocity
  *   - I2C interface (400kHz Fast Mode)
  *   - Dependency injection design pattern
  *   - Fast calibration (approx. 1 second)
  *   - No magnetometer usage (avoids indoor magnetic interference)
  *
  * Sensor Specifications:
  *   - Chip: Bosch BNO055
  *   - Operating voltage: 3.3V-5V
  *   - I2C address: 0x28 (7-bit)
  *   - Data rate: up to 100Hz
  *
  * IMU Mode Characteristics:
  *   - Roll/Pitch: Stable and accurate
  *   - Yaw: Drifts over time (no magnetometer correction)
  *   - Best for short-term orientation tracking
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BNO055_H
#define BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#if BNO055_ENABLE

#include "stm32l4xx_hal.h"

/* ============================================ */
/* Data Structure Definitions                   */
/* ============================================ */

/**
 * @brief BNO055 IMU sensor complete data structure
 */
typedef struct {
    // Quaternion data (w, x, y, z)
    int16_t quat_w_raw;      // Raw quaternion W component
    int16_t quat_x_raw;      // Raw quaternion X component
    int16_t quat_y_raw;      // Raw quaternion Y component
    int16_t quat_z_raw;      // Raw quaternion Z component
    float quat_w;            // Quaternion W (normalized, 1 LSB = 1/16384)
    float quat_x;            // Quaternion X (normalized, 1 LSB = 1/16384)
    float quat_y;            // Quaternion Y (normalized, 1 LSB = 1/16384)
    float quat_z;            // Quaternion Z (normalized, 1 LSB = 1/16384)

    // Euler angles (heading, roll, pitch)
    int16_t euler_heading_raw;  // Raw heading (yaw) in degrees
    int16_t euler_roll_raw;     // Raw roll in degrees
    int16_t euler_pitch_raw;    // Raw pitch in degrees
    float euler_heading;         // Heading in degrees (0-360°, 1 LSB = 1/16 deg)
    float euler_roll;            // Roll in degrees (-180 to +180°, 1 LSB = 1/16 deg)
    float euler_pitch;           // Pitch in degrees (-90 to +90°, 1 LSB = 1/16 deg)

    // Linear acceleration (gravity-free)
    int16_t linear_accel_x_raw; // Raw X-axis linear acceleration
    int16_t linear_accel_y_raw; // Raw Y-axis linear acceleration
    int16_t linear_accel_z_raw; // Raw Z-axis linear acceleration
    float linear_accel_x;        // X-axis linear acceleration (m/s², 1 LSB = 1/100 m/s²)
    float linear_accel_y;        // Y-axis linear acceleration (m/s², 1 LSB = 1/100 m/s²)
    float linear_accel_z;        // Z-axis linear acceleration (m/s², 1 LSB = 1/100 m/s²)

    // Angular velocity (gyroscope)
    int16_t gyro_x_raw;         // Raw X-axis angular velocity
    int16_t gyro_y_raw;         // Raw Y-axis angular velocity
    int16_t gyro_z_raw;         // Raw Z-axis angular velocity
    float gyro_x;                // X-axis angular velocity (deg/s, 1 LSB = 1/16 deg/s)
    float gyro_y;                // Y-axis angular velocity (deg/s, 1 LSB = 1/16 deg/s)
    float gyro_z;                // Z-axis angular velocity (deg/s, 1 LSB = 1/16 deg/s)

    // Calibration status
    uint8_t calib_status;        // Calibration status register (0x35)
    uint8_t calib_sys;           // System calibration (0-3)
    uint8_t calib_gyro;          // Gyroscope calibration (0-3)
    uint8_t calib_accel;         // Accelerometer calibration (0-3)
    uint8_t calib_mag;           // Magnetometer calibration (not used in IMU mode)

    // System status
    uint8_t system_status;       // System status (0x39)
    uint8_t system_error;        // System error code (0x3A)
} BNO055_Data;

/* ============================================ */
/* I2C Address                                  */
/* ============================================ */

#define BNO055_ADDRESS          (0x28 << 1)  // 7-bit address 0x28, left shift for HAL

/* ============================================ */
/* Register Addresses - Page 0                  */
/* ============================================ */

// Chip identification
#define BNO055_REG_CHIP_ID      0x00  // Should read 0xA0
#define BNO055_REG_ACC_ID       0x01  // Accelerometer ID
#define BNO055_REG_MAG_ID       0x02  // Magnetometer ID
#define BNO055_REG_GYR_ID       0x03  // Gyroscope ID
#define BNO055_REG_SW_REV_ID_LSB 0x04 // Software revision LSB
#define BNO055_REG_SW_REV_ID_MSB 0x05 // Software revision MSB
#define BNO055_REG_BL_REV_ID    0x06  // Bootloader revision

// Page selection
#define BNO055_REG_PAGE_ID      0x07  // Page ID register (0 or 1)

// Gyroscope data
#define BNO055_REG_GYRO_X_LSB   0x14
#define BNO055_REG_GYRO_X_MSB   0x15
#define BNO055_REG_GYRO_Y_LSB   0x16
#define BNO055_REG_GYRO_Y_MSB   0x17
#define BNO055_REG_GYRO_Z_LSB   0x18
#define BNO055_REG_GYRO_Z_MSB   0x19

// Euler angles
#define BNO055_REG_EULER_H_LSB  0x1A  // Heading (yaw)
#define BNO055_REG_EULER_H_MSB  0x1B
#define BNO055_REG_EULER_R_LSB  0x1C  // Roll
#define BNO055_REG_EULER_R_MSB  0x1D
#define BNO055_REG_EULER_P_LSB  0x1E  // Pitch
#define BNO055_REG_EULER_P_MSB  0x1F

// Quaternion data
#define BNO055_REG_QUAT_W_LSB   0x20
#define BNO055_REG_QUAT_W_MSB   0x21
#define BNO055_REG_QUAT_X_LSB   0x22
#define BNO055_REG_QUAT_X_MSB   0x23
#define BNO055_REG_QUAT_Y_LSB   0x24
#define BNO055_REG_QUAT_Y_MSB   0x25
#define BNO055_REG_QUAT_Z_LSB   0x26
#define BNO055_REG_QUAT_Z_MSB   0x27

// Linear acceleration data
#define BNO055_REG_LIA_X_LSB    0x28
#define BNO055_REG_LIA_X_MSB    0x29
#define BNO055_REG_LIA_Y_LSB    0x2A
#define BNO055_REG_LIA_Y_MSB    0x2B
#define BNO055_REG_LIA_Z_LSB    0x2C
#define BNO055_REG_LIA_Z_MSB    0x2D

// Temperature
#define BNO055_REG_TEMP         0x34  // Temperature (1 LSB = 1°C)

// Calibration status
#define BNO055_REG_CALIB_STAT   0x35  // Calibration status

// System status
#define BNO055_REG_SYS_STATUS   0x39  // System status
#define BNO055_REG_SYS_ERR      0x3A  // System error
#define BNO055_REG_UNIT_SEL     0x3B  // Unit selection
#define BNO055_REG_OPR_MODE     0x3D  // Operation mode
#define BNO055_REG_PWR_MODE     0x3E  // Power mode
#define BNO055_REG_SYS_TRIGGER  0x3F  // System trigger (reset)
#define BNO055_REG_TEMP_SOURCE  0x40  // Temperature source

/* ============================================ */
/* Register Addresses - Page 1                  */
/* ============================================ */

#define BNO055_REG_ACC_CONFIG   0x08  // Accelerometer configuration
#define BNO055_REG_GYR_CONFIG_0 0x0A  // Gyroscope configuration 0
#define BNO055_REG_GYR_CONFIG_1 0x0B  // Gyroscope configuration 1
#define BNO055_REG_MAG_CONFIG   0x09  // Magnetometer configuration

/* ============================================ */
/* Operation Modes                              */
/* ============================================ */

#define BNO055_MODE_CONFIG      0x00  // Configuration mode (required for mode change)
#define BNO055_MODE_ACCONLY     0x01  // Accelerometer only
#define BNO055_MODE_MAGONLY     0x02  // Magnetometer only
#define BNO055_MODE_GYROONLY    0x03  // Gyroscope only
#define BNO055_MODE_ACCMAG      0x04  // Accelerometer + Magnetometer
#define BNO055_MODE_ACCGYRO     0x05  // Accelerometer + Gyroscope
#define BNO055_MODE_MAGGYRO     0x06  // Magnetometer + Gyroscope
#define BNO055_MODE_AMG         0x07  // Accelerometer + Magnetometer + Gyroscope
#define BNO055_MODE_IMU         0x08  // IMU mode (Accel + Gyro, no Mag) *** TARGET MODE ***
#define BNO055_MODE_COMPASS     0x09  // Compass mode
#define BNO055_MODE_M4G         0x0A  // M4G mode
#define BNO055_MODE_NDOF_FMC_OFF 0x0B // NDOF with Fast Mag Calibration OFF
#define BNO055_MODE_NDOF        0x0C  // NDOF mode (9DOF fusion)

/* ============================================ */
/* Power Modes                                  */
/* ============================================ */

#define BNO055_PWR_NORMAL       0x00  // Normal power mode
#define BNO055_PWR_LOW          0x01  // Low power mode
#define BNO055_PWR_SUSPEND      0x02  // Suspend mode

/* ============================================ */
/* Unit Selection (Register 0x3B)              */
/* ============================================ */

#define BNO055_UNIT_ACCEL_MS2   0x00  // Acceleration: m/s²
#define BNO055_UNIT_ACCEL_MG    0x01  // Acceleration: mg
#define BNO055_UNIT_ANGRATE_DPS 0x00  // Angular rate: deg/s (bit 1 = 0)
#define BNO055_UNIT_ANGRATE_RPS 0x02  // Angular rate: rad/s (bit 1 = 1)
#define BNO055_UNIT_EULER_DEG   0x00  // Euler angles: degrees (bit 2 = 0)
#define BNO055_UNIT_EULER_RAD   0x04  // Euler angles: radians (bit 2 = 1)
#define BNO055_UNIT_TEMP_C      0x00  // Temperature: Celsius (bit 4 = 0)
#define BNO055_UNIT_TEMP_F      0x10  // Temperature: Fahrenheit (bit 4 = 1)

// Default unit configuration: m/s², deg/s, degrees, Celsius
#define BNO055_UNIT_DEFAULT     0x00  // All SI units

/* ============================================ */
/* Chip ID Values                               */
/* ============================================ */

#define BNO055_CHIP_ID_VALUE    0xA0  // Expected CHIP_ID value

/* ============================================ */
/* Conversion Factors                           */
/* ============================================ */

#define BNO055_QUAT_SCALE       16384.0f    // 1 LSB = 1/16384 (quaternion)
#define BNO055_EULER_SCALE      16.0f       // 1 LSB = 1/16 degree
#define BNO055_GYRO_SCALE       16.0f       // 1 LSB = 1/16 deg/s
#define BNO055_ACCEL_SCALE      100.0f      // 1 LSB = 1/100 m/s² (linear accel)

/* ============================================ */
/* Calibration Status Macros                    */
/* ============================================ */

#define BNO055_CALIB_SYS(status)    (((status) >> 6) & 0x03)  // System calibration
#define BNO055_CALIB_GYRO(status)   (((status) >> 4) & 0x03)  // Gyro calibration
#define BNO055_CALIB_ACCEL(status)  (((status) >> 2) & 0x03)  // Accel calibration
#define BNO055_CALIB_MAG(status)    ((status) & 0x03)         // Mag calibration

/* ============================================ */
/* Configuration Values - Page 1                */
/* ============================================ */

// Accelerometer configuration (0x08)
// Range: 2g/4g/8g/16g, Bandwidth: 7.81Hz - 1000Hz
#define BNO055_ACC_CONFIG_DEFAULT   0x0D  // 4g range, 62.5Hz bandwidth

// Gyroscope configuration 0 (0x0A)
// Range: 2000/1000/500/250/125 dps, Bandwidth: 523Hz - 12Hz
#define BNO055_GYR_CONFIG_0_DEFAULT 0x38  // 2000dps range, 32Hz bandwidth

// Gyroscope configuration 1 (0x0B)
// Power mode: Normal/Fast powerup/Deep suspend/Suspend/Advanced powerup
#define BNO055_GYR_CONFIG_1_DEFAULT 0x00  // Normal power mode

/* ============================================ */
/* Public Function Prototypes                   */
/* ============================================ */

/**
 * @brief Initialize BNO055 sensor in IMU mode
 *
 * This function performs the following initialization sequence:
 * 1. Wait 650ms for sensor boot-up
 * 2. Verify CHIP_ID (should be 0xA0)
 * 3. Switch to CONFIG mode
 * 4. Reset system (if reset_on_init = 1)
 * 5. Switch to Page 1 for sensor configuration
 * 6. Configure accelerometer (range, bandwidth)
 * 7. Configure gyroscope (range, bandwidth, power mode)
 * 8. Switch back to Page 0
 * 9. Set unit selection (m/s², deg/s, degrees)
 * 10. Set power mode (Normal)
 * 11. Set operation mode to IMU (0x08)
 * 12. Wait for mode transition (20ms)
 *
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 * @param reset_on_init If 1, perform system reset; if 0, skip reset
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c, uint8_t reset_on_init);

/**
 * @brief Read quaternion data from BNO055
 *
 * Reads 8 bytes from registers 0x20-0x27 (W, X, Y, Z components).
 * Converts raw int16_t values to normalized float quaternion.
 *
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to BNO055_Data structure (quaternion fields updated)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_ReadQuaternion(I2C_HandleTypeDef *hi2c, BNO055_Data *data);

/**
 * @brief Read Euler angles from BNO055
 *
 * Reads 6 bytes from registers 0x1A-0x1F (Heading, Roll, Pitch).
 * Converts raw int16_t values to degrees.
 *
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to BNO055_Data structure (Euler fields updated)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, BNO055_Data *data);

/**
 * @brief Read linear acceleration data from BNO055
 *
 * Reads 6 bytes from registers 0x28-0x2D (X, Y, Z axes).
 * Linear acceleration is gravity-free (sensor fusion output).
 * Converts raw int16_t values to m/s².
 *
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to BNO055_Data structure (linear accel fields updated)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_ReadLinearAccel(I2C_HandleTypeDef *hi2c, BNO055_Data *data);

/**
 * @brief Read gyroscope data from BNO055
 *
 * Reads 6 bytes from registers 0x14-0x19 (X, Y, Z axes).
 * Converts raw int16_t values to deg/s.
 *
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to BNO055_Data structure (gyro fields updated)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_ReadGyro(I2C_HandleTypeDef *hi2c, BNO055_Data *data);

/**
 * @brief Read calibration status from BNO055
 *
 * Reads 1 byte from register 0x35.
 * Extracts individual calibration values for system, gyro, accel, mag.
 * Each value ranges from 0 (uncalibrated) to 3 (fully calibrated).
 *
 * In IMU mode:
 * - System: Should reach 3 within 1 second
 * - Gyro: Calibrates quickly (keep sensor still)
 * - Accel: Calibrates automatically during movement
 * - Mag: Not used (will remain 0)
 *
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to BNO055_Data structure (calib fields updated)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_ReadCalibStatus(I2C_HandleTypeDef *hi2c, BNO055_Data *data);

/**
 * @brief Read complete sensor data (all fields)
 *
 * This convenience function reads all sensor outputs in sequence:
 * 1. Quaternion (8 bytes)
 * 2. Euler angles (6 bytes)
 * 3. Linear acceleration (6 bytes)
 * 4. Gyroscope (6 bytes)
 * 5. Calibration status (1 byte)
 *
 * Total: 27 bytes read from sensor
 *
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to BNO055_Data structure (all fields updated)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_ReadAll(I2C_HandleTypeDef *hi2c, BNO055_Data *data);

/**
 * @brief Read system status and error code
 *
 * Reads system status (0x39) and error code (0x3A).
 * Useful for diagnostics and error handling.
 *
 * System Status values:
 * - 0: Idle
 * - 1: System Error
 * - 2: Initializing Peripherals
 * - 3: System Initialization
 * - 4: Executing Self-Test
 * - 5: Fusion Algorithm Running
 * - 6: System Running without Fusion
 *
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to BNO055_Data structure (system status fields updated)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef BNO055_ReadSystemStatus(I2C_HandleTypeDef *hi2c, BNO055_Data *data);

#if BNO055_TEST_ENABLE
/**
 * @brief Run BNO055 test program in IMU mode
 *
 * This function performs comprehensive testing of the BNO055 sensor:
 * 1. Initialize BNO055 in IMU mode (0x08)
 * 2. Display chip ID and firmware versions
 * 3. Wait for calibration to complete (system cal = 3)
 * 4. Enter infinite loop:
 *    - Read all sensor data (quaternion, Euler, linear accel, gyro)
 *    - Display formatted data via UART
 *    - Show calibration status
 *    - Update every 500ms
 *
 * Expected UART output format:
 * ========================================
 *   CoVAPSy BNO055 IMU Sensor Test
 *   Mode: IMU (0x08)
 * ========================================
 * [INFO] Initializing BNO055...
 * [OK] BNO055 detected! Chip ID: 0xA0
 * [OK] IMU mode activated (0x08)
 * [INFO] Waiting for calibration...
 * [OK] Calibration complete! (Sys:3 Gyro:3 Accel:3)
 * ----------------------------------------
 * [0001] Euler: H=180.5° R=-2.3° P=1.1°  |  Gyro: X=+0.12 Y=-0.05 Z=+0.01 deg/s
 *        Quat: W=+0.999 X=-0.012 Y=+0.003 Z=-0.001
 *        Accel: X=+0.05 Y=-0.02 Z=+0.01 m/s²  |  Calib: S:3 G:3 A:3
 * ----------------------------------------
 *
 * WARNING: This function contains an infinite loop and never returns!
 *
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 * @param huart Pointer to UART handle for debug output (e.g., &huart2)
 */
void BNO055_Test(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
#endif

#endif /* BNO055_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
