/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sharp_sensor.h
  * @brief          : SHARP GP2Y0A21YK0F distance sensor driver header
  * @author         : CoVAPSy Team
  * @date           : 2025-12-03
  ******************************************************************************
  * @attention
  *
  * This driver supports the SHARP GP2Y0A21YK0F infrared distance sensor.
  * Features:
  *   - Measurement range: 10-80cm
  *   - Analog output: 0.5V-2.5V
  *   - ADC interface (12-bit resolution)
  *   - DMA continuous mode support
  *   - Dependency injection design pattern
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SHARP_SENSOR_H
#define SHARP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#if SHARP_SENSOR_ENABLE

#include "stm32l4xx_hal.h"

/* ============================================ */
/* Data Structure Definitions                   */
/* ============================================ */

/**
 * @brief SHARP sensor data structure
 */
typedef struct {
    uint16_t adc_raw;        // Raw ADC value (0-4095)
    float voltage;           // Voltage value (V)
    float distance_cm;       // Distance (cm)
    uint8_t out_of_range;    // Out of range flag
} Sharp_Data;

/* ============================================ */
/* Constant Definitions                         */
/* ============================================ */

#define SHARP_ADC_RESOLUTION    4096.0f        // 12-bit ADC
#define SHARP_VREF              3.3f           // Reference voltage (V)
#define SHARP_DISTANCE_COEFF    27.86f         // GP2Y0A21YK0F distance coefficient
#define SHARP_VOLTAGE_OFFSET    0.42f          // Voltage offset for distance calculation
#define SHARP_MIN_DISTANCE      10.0f          // Minimum measurement distance (cm)
#define SHARP_MAX_DISTANCE      80.0f          // Maximum measurement distance (cm)
#define SHARP_MIN_VOLTAGE       0.5f           // Minimum valid voltage (V)
#define SHARP_MAX_VOLTAGE       2.5f           // Maximum valid voltage (V)

/* ============================================ */
/* Public Function Prototypes                   */
/* ============================================ */

/**
 * @brief Initialize SHARP sensor (ADC-DMA)
 *
 * This function performs the following steps:
 * 1. Calibrate ADC for improved accuracy
 * 2. Start ADC-DMA continuous conversion
 * 3. Wait for first conversion to complete
 *
 * @param hadc Pointer to ADC handle (e.g., &hadc1)
 * @param dma_buffer Pointer to DMA buffer array (uint16_t for HALFWORD DMA)
 * @param buffer_size Size of DMA buffer (number of channels)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef Sharp_Init(ADC_HandleTypeDef *hadc,
                              uint16_t *dma_buffer,
                              uint32_t buffer_size);

/**
 * @brief Read raw ADC value from DMA buffer
 *
 * @param dma_buffer Pointer to DMA buffer array (uint16_t for HALFWORD DMA)
 * @param channel_index Channel index in DMA buffer (0 for PA3/ADC1_IN8)
 * @return Raw ADC value (0-4095)
 */
uint16_t Sharp_ReadRaw(uint16_t *dma_buffer, uint32_t channel_index);

/**
 * @brief Convert ADC value to voltage
 *
 * @param adc_raw Raw ADC value
 * @return Voltage value (V)
 */
float Sharp_ConvertToVoltage(uint16_t adc_raw);

/**
 * @brief Convert voltage to distance
 *
 * This function uses the formula: Distance(cm) = 27.86 / (Voltage - 0.42)
 * Valid for GP2Y0A21YK0F sensor in the range 10-80cm.
 *
 * @param voltage Voltage value (V)
 * @param out_of_range Output parameter for out-of-range flag (0=valid, 1=invalid)
 * @return Distance (cm), returns 0.0 if out of range
 */
float Sharp_ConvertToDistance(float voltage, uint8_t *out_of_range);

/**
 * @brief Read complete sensor data (ADC + Voltage + Distance)
 *
 * This is a convenience function that combines all conversion steps:
 * 1. Read raw ADC value from DMA buffer
 * 2. Convert to voltage
 * 3. Convert to distance
 * 4. Check range validity
 *
 * @param dma_buffer Pointer to DMA buffer array (uint16_t for HALFWORD DMA)
 * @param channel_index Channel index in DMA buffer (0 for PA3/ADC1_IN8)
 * @param data Pointer to Sharp_Data structure to store results
 */
void Sharp_ReadData(uint16_t *dma_buffer, uint32_t channel_index, Sharp_Data *data);

#if SHARP_TEST_ENABLE
/**
 * @brief Run SHARP sensor test program (infinite loop)
 *
 * This function performs continuous testing of the SHARP distance sensor:
 * 1. Initialize ADC-DMA
 * 2. Enter infinite loop:
 *    - Read sensor data (ADC + Voltage + Distance)
 *    - Output to UART in human-readable format
 *    - Delay 500ms between readings
 *
 * Expected output format:
 * [0001] ADC=2048  V=1.650V  Distance= 22.6cm
 * [0002] ADC=2100  V=1.692V  Distance= 21.9cm
 *
 * WARNING: This function contains an infinite loop and never returns!
 *
 * @param hadc Pointer to ADC handle (e.g., &hadc1)
 * @param huart Pointer to UART handle for debug output (e.g., &huart2)
 */
void Sharp_Test(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart);
#endif

#endif /* SHARP_SENSOR_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* SHARP_SENSOR_H */
