/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mma8451.c
  * @brief          : MMA8451 accelerometer driver implementation
  * @author         : CoVAPSy Team
  * @date           : 2025-12-01
  ******************************************************************************
  * @attention
  *
  * This driver implements the MMA8451 accelerometer functionality with
  * dependency injection pattern for maximum flexibility and testability.
  *
  * All functions accept peripheral handles as parameters instead of using
  * global variables, allowing for:
  * - Multiple sensor instances
  * - Better testability
  * - Cleaner code organization
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mma8451.h"

#if MMA8451_ENABLE

#include <stdio.h>
#include <string.h>

/* ============================================ */
/* Private Variables                            */
/* ============================================ */

// Store current range configuration (static = file scope only)
static uint8_t mma8451_range = MMA8451_RANGE_2G;

/* ============================================ */
/* Public Function Implementations              */
/* ============================================ */

/**
 * @brief Initialize MMA8451 accelerometer
 * @param hi2c Pointer to I2C handle
 * @param range Range selection (MMA8451_RANGE_2G/4G/8G)
 * @retval HAL status
 */
HAL_StatusTypeDef MMA8451_Init(I2C_HandleTypeDef *hi2c, uint8_t range)
{
    HAL_StatusTypeDef status;
    uint8_t reg_data;
    char uart_tx_buffer[150];

    // 1. Check device ID (WHO_AM_I register should return 0x1A)
    status = HAL_I2C_Mem_Read(hi2c, MMA8451_ADDRESS,
                              MMA8451_REG_WHOAMI, 1,
                              &reg_data, 1, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    if (reg_data != MMA8451_DEVICE_ID) {
        return HAL_ERROR;
    }

    // 2. Enter Standby mode (required before modifying configuration)
    reg_data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MMA8451_ADDRESS,
                               MMA8451_REG_CTRL_REG1, 1,
                               &reg_data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return HAL_ERROR;

    // 3. Configure range (XYZ_DATA_CFG register)
    mma8451_range = range;
    reg_data = range;
    status = HAL_I2C_Mem_Write(hi2c, MMA8451_ADDRESS,
                               MMA8451_REG_XYZ_DATA_CFG, 1,
                               &reg_data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return HAL_ERROR;

    // 4. Configure high resolution mode (CTRL_REG2)
    reg_data = MMA8451_MODE_HIGHRES;  // MODS[1:0] = 10 (High Resolution mode)
    status = HAL_I2C_Mem_Write(hi2c, MMA8451_ADDRESS,
                               MMA8451_REG_CTRL_REG2, 1,
                               &reg_data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return HAL_ERROR;

    // 5. Set data rate and start Active mode (CTRL_REG1)
    // DR[2:0] = 011 (100Hz), LNOISE = 1 (low noise), ACTIVE = 1
    reg_data = MMA8451_DATARATE_100HZ | MMA8451_MODE_LOWNOISE | MMA8451_ACTIVE_BIT;
    status = HAL_I2C_Mem_Write(hi2c, MMA8451_ADDRESS,
                               MMA8451_REG_CTRL_REG1, 1,
                               &reg_data, 1, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        HAL_Delay(2);  // Wait for sensor to switch to Active mode
    }

    return status;
}

/**
 * @brief Read MMA8451 acceleration data
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to data structure
 * @retval HAL status
 */
HAL_StatusTypeDef MMA8451_ReadAccel(I2C_HandleTypeDef *hi2c, MMA8451_Data *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[6];
    float divider;

    // Read 6 bytes continuously starting from OUT_X_MSB (X/Y/Z MSB+LSB)
    status = HAL_I2C_Mem_Read(hi2c, MMA8451_ADDRESS,
                              MMA8451_REG_OUT_X_MSB, 1,
                              buffer, 6, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        // MMA8451 is 14-bit data, left-aligned to 16-bit
        // Need to right shift 2 bits to get true 14-bit value
        data->x_raw = (int16_t)((buffer[0] << 8) | buffer[1]) >> 2;
        data->y_raw = (int16_t)((buffer[2] << 8) | buffer[3]) >> 2;
        data->z_raw = (int16_t)((buffer[4] << 8) | buffer[5]) >> 2;

        // Select conversion factor based on range
        switch (mma8451_range) {
            case MMA8451_RANGE_2G:
                divider = 4096.0f;  // ±2g: 4096 LSB/g
                break;
            case MMA8451_RANGE_4G:
                divider = 2048.0f;  // ±4g: 2048 LSB/g
                break;
            case MMA8451_RANGE_8G:
                divider = 1024.0f;  // ±8g: 1024 LSB/g
                break;
            default:
                divider = 4096.0f;
        }

        // Convert to g value
        data->x_g = (float)data->x_raw / divider;
        data->y_g = (float)data->y_raw / divider;
        data->z_g = (float)data->z_raw / divider;
    }

    return status;
}

/**
 * @brief Check if new data is available
 * @param hi2c Pointer to I2C handle
 * @retval 1=new data available, 0=no new data
 */
uint8_t MMA8451_DataReady(I2C_HandleTypeDef *hi2c)
{
    uint8_t status;

    if (HAL_I2C_Mem_Read(hi2c, MMA8451_ADDRESS,
                         MMA8451_REG_STATUS, 1,
                         &status, 1, 100) == HAL_OK) {
        return (status & MMA8451_STATUS_ZYXDR) ? 1 : 0;  // ZYXDR bit
    }

    return 0;
}

#if MMA8451_TEST_ENABLE
/**
 * @brief MMA8451 accelerometer complete test program
 * @param hi2c Pointer to I2C handle
 * @param huart Pointer to UART handle for debug output
 * @param led_port GPIO port for LED indicator
 * @param led_pin GPIO pin number for LED indicator
 */
void MMA8451_Test(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart,
                  GPIO_TypeDef *led_port, uint16_t led_pin)
{
    uint32_t sample_count = 0;
    MMA8451_Data accel_data;      // Local variable instead of global
    char uart_tx_buffer[150];     // Local variable instead of global

    // Print test start message
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "  CoVAPSy MMA8451 Accelerometer Test\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "========================================\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Initialize MMA8451
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Initializing MMA8451...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    if (MMA8451_Init(hi2c, MMA8451_RANGE_2G) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[FATAL] MMA8451 initialization failed! Check hardware connection.\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // LED fast blink indicates error
        while (1) {
            HAL_GPIO_TogglePin(led_port, led_pin);
            HAL_Delay(100);
        }
    }

    // Print device ID
    uint8_t device_id = 0;
    if (HAL_I2C_Mem_Read(hi2c, MMA8451_ADDRESS, MMA8451_REG_WHOAMI, 1,
                         &device_id, 1, HAL_MAX_DELAY) == HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[OK] MMA8451 detected! Device ID: 0x%02X\r\n", device_id);
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    }

    // Print configuration
    const char* range_str[] = {"±2g", "±4g", "±8g"};
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] Range configured: %s\r\n", range_str[MMA8451_RANGE_2G]);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] MMA8451 initialized! Sample rate: 100Hz\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n[INFO] Starting data collection (every 500ms)...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "----------------------------------------\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Main loop: continuously read acceleration data
    while (1) {
        // Check if data is ready
        if (MMA8451_DataReady(hi2c)) {
            // Read acceleration data
            if (MMA8451_ReadAccel(hi2c, &accel_data) == HAL_OK) {
                sample_count++;

                // Format and output data
                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                         "[%04lu] X=%+6.3fg  Y=%+6.3fg  Z=%+6.3fg  |  RAW: X=%5d Y=%5d Z=%5d\r\n",
                         sample_count,
                         accel_data.x_g, accel_data.y_g, accel_data.z_g,
                         accel_data.x_raw, accel_data.y_raw, accel_data.z_raw);

                HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                // LED blink indicates data is normal
                HAL_GPIO_TogglePin(led_port, led_pin);

                // Print separator line every 10 samples
                if (sample_count % 10 == 0) {
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                             "----------------------------------------\r\n");
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                }
            } else {
                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                         "[ERROR] Failed to read acceleration data!\r\n");
                HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
            }
        }

        HAL_Delay(500);  // Read every 500ms
    }
}
#endif /* MMA8451_TEST_ENABLE */

#endif /* MMA8451_ENABLE */
