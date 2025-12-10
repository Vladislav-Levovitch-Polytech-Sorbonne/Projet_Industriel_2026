/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055.c
  * @brief          : BNO055 9-axis IMU sensor driver implementation
  * @author         : CoVAPSy Team
  * @date           : 2025-12-10
  ******************************************************************************
  * @attention
  *
  * This driver implements BNO055 sensor in IMU mode (0x08) for indoor
  * autonomous vehicle navigation. The implementation follows CoVAPSy project
  * standards with dependency injection pattern and comprehensive error handling.
  *
  * Key Features:
  * - IMU mode operation (no magnetometer)
  * - Quaternion and Euler angle output
  * - Linear acceleration and angular velocity
  * - Automatic calibration monitoring
  * - Detailed UART test output
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bno055.h"

#if BNO055_ENABLE

#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef BNO055_WriteByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BNO055_ReadByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef BNO055_ReadMultiBytes(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                                uint8_t *buffer, uint16_t length);
static HAL_StatusTypeDef BNO055_SetPage(I2C_HandleTypeDef *hi2c, uint8_t page);
static HAL_StatusTypeDef BNO055_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode);

/* ============================================ */
/* Private Helper Functions                     */
/* ============================================ */

/**
 * @brief Write single byte to BNO055 register
 */
static HAL_StatusTypeDef BNO055_WriteByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, BNO055_ADDRESS, reg, 1, &value, 1, HAL_MAX_DELAY);
}

/**
 * @brief Read single byte from BNO055 register
 */
static HAL_StatusTypeDef BNO055_ReadByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, BNO055_ADDRESS, reg, 1, value, 1, HAL_MAX_DELAY);
}

/**
 * @brief Read multiple bytes from BNO055 registers (consecutive)
 */
static HAL_StatusTypeDef BNO055_ReadMultiBytes(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                                uint8_t *buffer, uint16_t length)
{
    return HAL_I2C_Mem_Read(hi2c, BNO055_ADDRESS, reg, 1, buffer, length, HAL_MAX_DELAY);
}

/**
 * @brief Switch BNO055 register page (0 or 1)
 */
static HAL_StatusTypeDef BNO055_SetPage(I2C_HandleTypeDef *hi2c, uint8_t page)
{
    if (page > 1) return HAL_ERROR;
    return BNO055_WriteByte(hi2c, BNO055_REG_PAGE_ID, page);
}

/**
 * @brief Set BNO055 operation mode
 */
static HAL_StatusTypeDef BNO055_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
    HAL_StatusTypeDef status;

    // Switch to CONFIG mode first (required for mode change)
    status = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
    if (status != HAL_OK) return status;

    HAL_Delay(20);  // Wait for mode transition (datasheet: 19ms)

    // Set target mode
    status = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, mode);
    if (status != HAL_OK) return status;

    HAL_Delay(20);  // Wait for mode transition

    return HAL_OK;
}

/* ============================================ */
/* Public Function Implementations              */
/* ============================================ */

/**
 * @brief Initialize BNO055 sensor in IMU mode
 */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c, uint8_t reset_on_init)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id = 0;

    // Step 1: Wait for sensor boot-up (datasheet: 650ms minimum)
    HAL_Delay(700);

    // Step 2: Verify CHIP_ID
    status = BNO055_ReadByte(hi2c, BNO055_REG_CHIP_ID, &chip_id);
    if (status != HAL_OK) {
        return HAL_ERROR;  // I2C communication error
    }

    if (chip_id != BNO055_CHIP_ID_VALUE) {
        return HAL_ERROR;  // Wrong chip ID
    }

    // Step 3: Switch to CONFIG mode
    status = BNO055_SetMode(hi2c, BNO055_MODE_CONFIG);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 4: Reset system (optional)
    if (reset_on_init) {
        status = BNO055_WriteByte(hi2c, BNO055_REG_SYS_TRIGGER, 0x20);  // Bit 5: RST_SYS
        if (status != HAL_OK) return HAL_ERROR;

        HAL_Delay(700);  // Wait for reset to complete

        // Re-enter CONFIG mode after reset
        status = BNO055_SetMode(hi2c, BNO055_MODE_CONFIG);
        if (status != HAL_OK) return HAL_ERROR;
    }

    // Step 5: Switch to Page 1 for sensor configuration
    status = BNO055_SetPage(hi2c, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 6: Configure accelerometer (Page 1, Register 0x08)
    // Default: 4g range, 62.5Hz bandwidth
    status = BNO055_WriteByte(hi2c, BNO055_REG_ACC_CONFIG, BNO055_ACC_CONFIG_DEFAULT);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 7: Configure gyroscope (Page 1, Register 0x0A, 0x0B)
    // Config 0: 2000dps range, 32Hz bandwidth
    status = BNO055_WriteByte(hi2c, BNO055_REG_GYR_CONFIG_0, BNO055_GYR_CONFIG_0_DEFAULT);
    if (status != HAL_OK) return HAL_ERROR;

    // Config 1: Normal power mode
    status = BNO055_WriteByte(hi2c, BNO055_REG_GYR_CONFIG_1, BNO055_GYR_CONFIG_1_DEFAULT);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 8: Switch back to Page 0
    status = BNO055_SetPage(hi2c, 0);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 9: Set unit selection (m/s², deg/s, degrees, Celsius)
    status = BNO055_WriteByte(hi2c, BNO055_REG_UNIT_SEL, BNO055_UNIT_DEFAULT);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 10: Set power mode (Normal)
    status = BNO055_WriteByte(hi2c, BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 11: Set operation mode to IMU (0x08)
    status = BNO055_SetMode(hi2c, BNO055_MODE_IMU);
    if (status != HAL_OK) return HAL_ERROR;

    // Step 12: Wait for mode transition
    HAL_Delay(20);

    return HAL_OK;
}

/**
 * @brief Read quaternion data from BNO055
 */
HAL_StatusTypeDef BNO055_ReadQuaternion(I2C_HandleTypeDef *hi2c, BNO055_Data *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[8];  // 4 × 2 bytes (W, X, Y, Z)

    // Read 8 bytes from 0x20 (QUAT_W_LSB)
    status = BNO055_ReadMultiBytes(hi2c, BNO055_REG_QUAT_W_LSB, buffer, 8);
    if (status != HAL_OK) return status;

    // Parse raw values (little-endian)
    data->quat_w_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->quat_x_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->quat_y_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
    data->quat_z_raw = (int16_t)((buffer[7] << 8) | buffer[6]);

    // Convert to normalized quaternion (1 LSB = 1/16384)
    data->quat_w = (float)data->quat_w_raw / BNO055_QUAT_SCALE;
    data->quat_x = (float)data->quat_x_raw / BNO055_QUAT_SCALE;
    data->quat_y = (float)data->quat_y_raw / BNO055_QUAT_SCALE;
    data->quat_z = (float)data->quat_z_raw / BNO055_QUAT_SCALE;

    return HAL_OK;
}

/**
 * @brief Read Euler angles from BNO055
 */
HAL_StatusTypeDef BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, BNO055_Data *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];  // 3 × 2 bytes (Heading, Roll, Pitch)

    // Read 6 bytes from 0x1A (EULER_H_LSB)
    status = BNO055_ReadMultiBytes(hi2c, BNO055_REG_EULER_H_LSB, buffer, 6);
    if (status != HAL_OK) return status;

    // Parse raw values (little-endian)
    // Note: BNO055 register labels are swapped relative to standard aircraft convention
    // Register 0x1C is labeled "Roll" but represents Pitch motion
    // Register 0x1E is labeled "Pitch" but represents Roll motion
    data->euler_heading_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->euler_pitch_raw = (int16_t)((buffer[3] << 8) | buffer[2]);  // Swapped: 0x1C register
    data->euler_roll_raw = (int16_t)((buffer[5] << 8) | buffer[4]);   // Swapped: 0x1E register

    // Convert to degrees (1 LSB = 1/16 degree)
    data->euler_heading = (float)data->euler_heading_raw / BNO055_EULER_SCALE;
    data->euler_pitch = (float)data->euler_pitch_raw / BNO055_EULER_SCALE;  // Swapped
    data->euler_roll = (float)data->euler_roll_raw / BNO055_EULER_SCALE;    // Swapped

    return HAL_OK;
}

/**
 * @brief Read linear acceleration data from BNO055
 */
HAL_StatusTypeDef BNO055_ReadLinearAccel(I2C_HandleTypeDef *hi2c, BNO055_Data *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];  // 3 × 2 bytes (X, Y, Z)

    // Read 6 bytes from 0x28 (LIA_X_LSB)
    status = BNO055_ReadMultiBytes(hi2c, BNO055_REG_LIA_X_LSB, buffer, 6);
    if (status != HAL_OK) return status;

    // Parse raw values (little-endian)
    data->linear_accel_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->linear_accel_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->linear_accel_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Convert to m/s² (1 LSB = 1/100 m/s²)
    data->linear_accel_x = (float)data->linear_accel_x_raw / BNO055_ACCEL_SCALE;
    data->linear_accel_y = (float)data->linear_accel_y_raw / BNO055_ACCEL_SCALE;
    data->linear_accel_z = (float)data->linear_accel_z_raw / BNO055_ACCEL_SCALE;

    return HAL_OK;
}

/**
 * @brief Read gyroscope data from BNO055
 */
HAL_StatusTypeDef BNO055_ReadGyro(I2C_HandleTypeDef *hi2c, BNO055_Data *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];  // 3 × 2 bytes (X, Y, Z)

    // Read 6 bytes from 0x14 (GYRO_X_LSB)
    status = BNO055_ReadMultiBytes(hi2c, BNO055_REG_GYRO_X_LSB, buffer, 6);
    if (status != HAL_OK) return status;

    // Parse raw values (little-endian)
    data->gyro_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->gyro_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->gyro_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Convert to deg/s (1 LSB = 1/16 deg/s)
    data->gyro_x = (float)data->gyro_x_raw / BNO055_GYRO_SCALE;
    data->gyro_y = (float)data->gyro_y_raw / BNO055_GYRO_SCALE;
    data->gyro_z = (float)data->gyro_z_raw / BNO055_GYRO_SCALE;

    return HAL_OK;
}

/**
 * @brief Read calibration status from BNO055
 */
HAL_StatusTypeDef BNO055_ReadCalibStatus(I2C_HandleTypeDef *hi2c, BNO055_Data *data)
{
    HAL_StatusTypeDef status;

    // Read 1 byte from 0x35 (CALIB_STAT)
    status = BNO055_ReadByte(hi2c, BNO055_REG_CALIB_STAT, &data->calib_status);
    if (status != HAL_OK) return status;

    // Extract individual calibration values (each 2 bits)
    data->calib_sys = BNO055_CALIB_SYS(data->calib_status);
    data->calib_gyro = BNO055_CALIB_GYRO(data->calib_status);
    data->calib_accel = BNO055_CALIB_ACCEL(data->calib_status);
    data->calib_mag = BNO055_CALIB_MAG(data->calib_status);  // Will be 0 in IMU mode

    return HAL_OK;
}

/**
 * @brief Read complete sensor data (all fields)
 */
HAL_StatusTypeDef BNO055_ReadAll(I2C_HandleTypeDef *hi2c, BNO055_Data *data)
{
    HAL_StatusTypeDef status;

    // Read all data fields in sequence
    status = BNO055_ReadQuaternion(hi2c, data);
    if (status != HAL_OK) return status;

    status = BNO055_ReadEuler(hi2c, data);
    if (status != HAL_OK) return status;

    status = BNO055_ReadLinearAccel(hi2c, data);
    if (status != HAL_OK) return status;

    status = BNO055_ReadGyro(hi2c, data);
    if (status != HAL_OK) return status;

    status = BNO055_ReadCalibStatus(hi2c, data);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief Read system status and error code
 */
HAL_StatusTypeDef BNO055_ReadSystemStatus(I2C_HandleTypeDef *hi2c, BNO055_Data *data)
{
    HAL_StatusTypeDef status;

    // Read system status (0x39)
    status = BNO055_ReadByte(hi2c, BNO055_REG_SYS_STATUS, &data->system_status);
    if (status != HAL_OK) return status;

    // Read system error (0x3A)
    status = BNO055_ReadByte(hi2c, BNO055_REG_SYS_ERR, &data->system_error);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

#if BNO055_TEST_ENABLE
/**
 * @brief Run BNO055 test program in IMU mode
 */
void BNO055_Test(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    uint32_t sample_count = 0;
    BNO055_Data imu_data;
    char uart_tx_buffer[300];  // Large buffer for multi-line output
    uint8_t chip_id = 0;
    uint8_t sw_rev_lsb = 0, sw_rev_msb = 0;

    // Print header
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "  CoVAPSy BNO055 IMU Sensor Test\r\n"
             "  Mode: IMU (0x08)\r\n"
             "========================================\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Initialization message
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Initializing BNO055...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Initialize BNO055
    if (BNO055_Init(hi2c, 1) != HAL_OK) {  // Reset on init
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[FATAL] BNO055 initialization failed! Check hardware connection.\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Infinite loop to halt execution on error
        while (1) {
            HAL_Delay(1000);
        }
    }

    // Read and display chip information
    BNO055_ReadByte(hi2c, BNO055_REG_CHIP_ID, &chip_id);
    BNO055_ReadByte(hi2c, BNO055_REG_SW_REV_ID_LSB, &sw_rev_lsb);
    BNO055_ReadByte(hi2c, BNO055_REG_SW_REV_ID_MSB, &sw_rev_msb);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] BNO055 detected! Chip ID: 0x%02X\r\n"
             "[OK] Software Revision: %d.%02d\r\n"
             "[OK] IMU mode activated (0x08)\r\n",
             chip_id, sw_rev_msb, sw_rev_lsb);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Wait for calibration
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Waiting for gyro calibration (keep sensor still)...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Poll calibration status until gyro is calibrated
    uint8_t calib_complete = 0;
    uint32_t calib_timeout = 0;

    while (!calib_complete && calib_timeout < 10) {  // 10 second timeout (reduced from 30)
        BNO055_ReadCalibStatus(hi2c, &imu_data);

        if (imu_data.calib_gyro == 3) {  // Changed: only require Gyro calibration (was Sys==3)
            calib_complete = 1;
        } else {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "       Calibration: Sys:%d Gyro:%d Accel:%d Mag:%d\r\n",
                     imu_data.calib_sys, imu_data.calib_gyro,
                     imu_data.calib_accel, imu_data.calib_mag);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

            HAL_Delay(1000);
            calib_timeout++;
        }
    }

    if (calib_complete) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[OK] Calibration complete! (Sys:%d Gyro:%d Accel:%d)\r\n\r\n",
                 imu_data.calib_sys, imu_data.calib_gyro, imu_data.calib_accel);
    } else {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[WARN] Calibration timeout! Proceeding anyway...\r\n\r\n");
    }
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Starting continuous reading (every 500ms)...\r\n"
             "----------------------------------------\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Main loop: continuously read all sensor data
    while (1) {
        sample_count++;

        // Read all sensor data
        if (BNO055_ReadAll(hi2c, &imu_data) == HAL_OK) {
            // Line 1: Sample count + Euler angles + Gyroscope
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[%04lu] Euler: H=%6.1fdeg R=%+6.1fdeg P=%+6.1fdeg  |  "
                     "Gyro: X=%+6.2f Y=%+6.2f Z=%+6.2f deg/s\r\n",
                     sample_count,
                     imu_data.euler_heading, imu_data.euler_roll, imu_data.euler_pitch,
                     imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

            // Line 2: Quaternion
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "       Quat: W=%+6.3f X=%+6.3f Y=%+6.3f Z=%+6.3f\r\n",
                     imu_data.quat_w, imu_data.quat_x,
                     imu_data.quat_y, imu_data.quat_z);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

            // Line 3: Linear acceleration + Calibration status
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "       Accel: X=%+5.2f Y=%+5.2f Z=%+5.2f m/s^2  |  "
                     "Calib: S:%d G:%d A:%d\r\n",
                     imu_data.linear_accel_x, imu_data.linear_accel_y, imu_data.linear_accel_z,
                     imu_data.calib_sys, imu_data.calib_gyro, imu_data.calib_accel);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

            // Print separator every 5 samples
            if (sample_count % 5 == 0) {
                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                         "----------------------------------------\r\n");
                HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
            }
        } else {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[ERROR] Failed to read IMU data!\r\n");
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        }

        // Wait 500ms between readings
        HAL_Delay(500);
    }
}
#endif /* BNO055_TEST_ENABLE */

#endif /* BNO055_ENABLE */
