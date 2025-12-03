/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mma8451.h
  * @brief          : MMA8451 accelerometer driver header
  * @author         : CoVAPSy Team
  * @date           : 2025-12-01
  ******************************************************************************
  * @attention
  *
  * This driver supports the Adafruit MMA8451 3-axis accelerometer.
  * Features:
  *   - 14-bit resolution
  *   - Configurable range: ±2g, ±4g, ±8g
  *   - Configurable data rate: 1.56Hz to 800Hz
  *   - I2C interface (400kHz Fast Mode)
  *   - Dependency injection design pattern
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MMA8451_H
#define MMA8451_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#if MMA8451_ENABLE

#include "stm32l4xx_hal.h"

/* ============================================ */
/* Data Structure Definitions                   */
/* ============================================ */

/**
 * @brief MMA8451 accelerometer data structure
 */
typedef struct {
    int16_t x_raw;   // 14-bit raw value (left-aligned to 16-bit)
    int16_t y_raw;
    int16_t z_raw;
    float x_g;       // Converted to g value
    float y_g;
    float z_g;
} MMA8451_Data;

/* ============================================ */
/* Register Address Definitions                 */
/* ============================================ */

// MMA8451 I2C address
#define MMA8451_ADDRESS         (0x1D << 1)  // Default address 0x1D, left shift 1 bit

// MMA8451 register addresses
#define MMA8451_REG_STATUS      0x00  // Data status register
#define MMA8451_REG_OUT_X_MSB   0x01  // X-axis data MSB (14-bit data starts here)
#define MMA8451_REG_OUT_X_LSB   0x02  // X-axis data LSB
#define MMA8451_REG_OUT_Y_MSB   0x03  // Y-axis data MSB
#define MMA8451_REG_OUT_Y_LSB   0x04  // Y-axis data LSB
#define MMA8451_REG_OUT_Z_MSB   0x05  // Z-axis data MSB
#define MMA8451_REG_OUT_Z_LSB   0x06  // Z-axis data LSB
#define MMA8451_REG_WHOAMI      0x0D  // Device ID (should read 0x1A)
#define MMA8451_REG_XYZ_DATA_CFG 0x0E // Range configuration
#define MMA8451_REG_CTRL_REG1   0x2A  // Control register 1 (power/data rate)
#define MMA8451_REG_CTRL_REG2   0x2B  // Control register 2 (mode)

// Device ID value
#define MMA8451_DEVICE_ID       0x1A  // Expected WHO_AM_I value

/* ============================================ */
/* Range Configuration Values                   */
/* ============================================ */

#define MMA8451_RANGE_2G        0x00  // ±2g (4096 LSB/g)
#define MMA8451_RANGE_4G        0x01  // ±4g (2048 LSB/g)
#define MMA8451_RANGE_8G        0x02  // ±8g (1024 LSB/g)

/* ============================================ */
/* Data Rate Configuration (CTRL_REG1 DR bits) */
/* ============================================ */

#define MMA8451_DATARATE_800HZ  0x00  // 800 Hz (fastest)
#define MMA8451_DATARATE_400HZ  0x08  // 400 Hz
#define MMA8451_DATARATE_200HZ  0x10  // 200 Hz
#define MMA8451_DATARATE_100HZ  0x18  // 100 Hz (recommended for most applications)
#define MMA8451_DATARATE_50HZ   0x20  // 50 Hz
#define MMA8451_DATARATE_12_5HZ 0x28  // 12.5 Hz
#define MMA8451_DATARATE_6_25HZ 0x30  // 6.25 Hz
#define MMA8451_DATARATE_1_56HZ 0x38  // 1.56 Hz (slowest, lowest power)

/* ============================================ */
/* Resolution Modes (CTRL_REG2 MODS bits)      */
/* ============================================ */

#define MMA8451_MODE_NORMAL     0x00  // Normal mode
#define MMA8451_MODE_LOWNOISE   0x04  // Low noise, low power mode
#define MMA8451_MODE_HIGHRES    0x02  // High resolution mode
#define MMA8451_MODE_LOWPOWER   0x03  // Low power mode

/* ============================================ */
/* Control Register Bit Definitions             */
/* ============================================ */

// CTRL_REG1 bits
#define MMA8451_ACTIVE_BIT      0x01  // Active mode bit
#define MMA8451_F_READ_BIT      0x02  // Fast read mode (8-bit instead of 14-bit)

// STATUS register bits
#define MMA8451_STATUS_ZYXDR    0x08  // XYZ data ready bit

/* ============================================ */
/* Public Function Prototypes                   */
/* ============================================ */

/**
 * @brief Initialize MMA8451 accelerometer
 *
 * This function performs the following steps:
 * 1. Check device ID (should be 0x1A)
 * 2. Reset device to standby mode
 * 3. Configure measurement range
 * 4. Configure data rate and resolution
 * 5. Activate the sensor
 *
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 * @param range Range configuration:
 *              - MMA8451_RANGE_2G: ±2g (4096 LSB/g)
 *              - MMA8451_RANGE_4G: ±4g (2048 LSB/g)
 *              - MMA8451_RANGE_8G: ±8g (1024 LSB/g)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef MMA8451_Init(I2C_HandleTypeDef *hi2c, uint8_t range);

/**
 * @brief Read acceleration data from MMA8451
 *
 * This function reads 6 bytes of data (X, Y, Z axes, 2 bytes each) from
 * the sensor and converts them to both raw 14-bit values and calibrated
 * g values based on the configured range.
 *
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 * @param data Pointer to MMA8451_Data structure to store results
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef MMA8451_ReadAccel(I2C_HandleTypeDef *hi2c, MMA8451_Data *data);

/**
 * @brief Check if new acceleration data is ready
 *
 * This function checks the STATUS register's ZYXDR bit to determine
 * if new data is available for all three axes.
 *
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 * @return 1 if data ready, 0 otherwise
 */
uint8_t MMA8451_DataReady(I2C_HandleTypeDef *hi2c);

#if MMA8451_TEST_ENABLE
/**
 * @brief Run MMA8451 test program
 *
 * This function performs a continuous test of the MMA8451 accelerometer:
 * 1. Initialize the sensor with ±2g range
 * 2. Enter infinite loop:
 *    - Wait for new data
 *    - Read acceleration values
 *    - Output to UART in human-readable format
 *    - Toggle LED to indicate activity
 *    - Delay 500ms between readings
 *
 * WARNING: This function contains an infinite loop and never returns!
 *
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 * @param huart Pointer to UART handle for debug output (e.g., &huart2)
 * @param led_port GPIO port for LED indicator (e.g., LD3_GPIO_Port)
 * @param led_pin GPIO pin number for LED indicator (e.g., LD3_Pin)
 */
void MMA8451_Test(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart,
                  GPIO_TypeDef *led_port, uint16_t led_pin);
#endif

#endif /* MMA8451_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* MMA8451_H */
