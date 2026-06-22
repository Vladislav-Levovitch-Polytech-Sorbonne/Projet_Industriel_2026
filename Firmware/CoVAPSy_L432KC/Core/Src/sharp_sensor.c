/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sharp_sensor.c
  * @brief          : SHARP GP2Y0A21YK0F distance sensor driver implementation
  * @author         : CoVAPSy Team
  * @date           : 2025-12-03
  ******************************************************************************
  * @attention
  *
  * This driver implements the SHARP GP2Y0A21YK0F infrared distance sensor.
  * It uses ADC with DMA in continuous mode for efficient data acquisition.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sharp_sensor.h"

#if SHARP_SENSOR_ENABLE

#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* ============================================ */
/* Public Function Implementations             */
/* ============================================ */

/**
 * @brief Initialize SHARP sensor (ADC-DMA)
 */
HAL_StatusTypeDef Sharp_Init(ADC_HandleTypeDef *hadc,
                              uint16_t *dma_buffer,
                              uint32_t buffer_size)
{
    // Step 1: Calibrate ADC for improved accuracy
    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 2: Start ADC-DMA continuous conversion
    if (HAL_ADC_Start_DMA(hadc, (uint32_t*)dma_buffer, buffer_size) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 3: Wait for first conversion to complete
    if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Read raw ADC value from DMA buffer
 */
uint16_t Sharp_ReadRaw(uint16_t *dma_buffer, uint32_t channel_index)
{
    return dma_buffer[channel_index];
}

/**
 * @brief Convert ADC value to voltage
 */
float Sharp_ConvertToVoltage(uint16_t adc_raw)
{
    return ((float)adc_raw * SHARP_VREF) / SHARP_ADC_RESOLUTION;
}

/**
 * @brief Convert voltage to distance
 */
float Sharp_ConvertToDistance(float voltage, uint8_t *out_of_range)
{
    *out_of_range = 0;

    // Boundary check: voltage must be within valid range
    if (voltage < SHARP_MIN_VOLTAGE || voltage > SHARP_MAX_VOLTAGE) {
        *out_of_range = 1;
        return 0.0f;
    }

    // Calculate distance using formula: D(cm) = 27.86 * V^(-1.15)
    float distance = SHARP_DISTANCE_COEFF * powf(voltage, SHARP_POWER_EXPONENT);

    // Range validation: distance must be within sensor specification
    if (distance < SHARP_MIN_DISTANCE || distance > SHARP_MAX_DISTANCE) {
        *out_of_range = 1;
        return 0.0f;
    }

    return distance;
}

/**
 * @brief Read complete sensor data (ADC + Voltage + Distance)
 */
void Sharp_ReadData(uint16_t *dma_buffer, uint32_t channel_index, Sharp_Data *data)
{
    // Step 1: Read raw ADC value
    data->adc_raw = Sharp_ReadRaw(dma_buffer, channel_index);

    // Step 2: Convert to voltage
    data->voltage = Sharp_ConvertToVoltage(data->adc_raw);

    // Step 3: Convert to distance with range check
    data->distance_cm = Sharp_ConvertToDistance(data->voltage, &data->out_of_range);
}

#if SHARP_TEST_ENABLE
/**
 * @brief Run SHARP sensor test program (infinite loop)
 */
void Sharp_Test(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart)
{
    uint32_t sample_count = 0;
    Sharp_Data sensor_left;   // PA3 (ADC_IN8) - Rear Left
    Sharp_Data sensor_right;  // PA6 (ADC_IN11) - Rear Right
    char uart_tx_buffer[250];
    static uint16_t sharp_adc_buffer[2] = {0};  // DMA buffer for 2 ADC channels (PA3, PA6)

    // Print header
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "  CoVAPSy SHARP Sensor Test (Dual)\r\n"
             "  Model: GP2Y0A21YK0F (10-80cm)\r\n"
             "========================================\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Initialization message
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Initializing SHARP sensors...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Initialize ADC-DMA
    if (Sharp_Init(hadc, sharp_adc_buffer, 2) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[FATAL] ADC initialization failed! Check hardware connection.\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Infinite loop to halt execution on error
        while (1) {
            HAL_Delay(1000);
        }
    }

    // Success message
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] ADC1 initialized! DMA continuous mode active.\r\n"
             "[OK] Channel 0: PA3 (ADC_IN8) - Rear Left\r\n"
             "[OK] Channel 1: PA6 (ADC_IN11) - Rear Right\r\n"
             "[INFO] Starting continuous reading...\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Main loop
    while (1) {
        sample_count++;

        // Read both sensor data
        Sharp_ReadData(sharp_adc_buffer, 0, &sensor_left);   // PA3 (ADC_IN8, Channel 0)
        Sharp_ReadData(sharp_adc_buffer, 1, &sensor_right);  // PA6 (ADC_IN11, Channel 1)

        // Print sample header with DMA raw values
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[%04lu] DMA_RAW: [0]=%4u [1]=%4u\r\n",
                 sample_count,
                 sharp_adc_buffer[0], sharp_adc_buffer[1]);
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Print Left sensor data
        if (sensor_left.out_of_range) {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "  LEFT:  ADC=%4u V=%5.3fV Dist=OUT_OF_RANGE\r\n",
                     sensor_left.adc_raw, sensor_left.voltage);
        } else {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "  LEFT:  ADC=%4u V=%5.3fV Dist=%5.1fcm\r\n",
                     sensor_left.adc_raw, sensor_left.voltage, sensor_left.distance_cm);
        }
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Print Right sensor data
        if (sensor_right.out_of_range) {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "  RIGHT: ADC=%4u V=%5.3fV Dist=OUT_OF_RANGE\r\n",
                     sensor_right.adc_raw, sensor_right.voltage);
        } else {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "  RIGHT: ADC=%4u V=%5.3fV Dist=%5.1fcm\r\n",
                     sensor_right.adc_raw, sensor_right.voltage, sensor_right.distance_cm);
        }
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Print separator every 10 samples
        if (sample_count % 10 == 0) {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "========================================\r\n");
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        }

        // Wait 500ms between readings
        HAL_Delay(500);
    }
}
#endif

#endif /* SHARP_SENSOR_ENABLE */
