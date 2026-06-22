/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : servo.c
  * @brief          : Reely RS-610WP MG Servo Driver Implementation
  * @author         : CoVAPSy Team
  * @date           : 2025-12-17
  ******************************************************************************
  */
/* USER CODE END Header */

#include "servo.h"

#if SERVO_ENABLE

#include <stdio.h>
#include <string.h>

/* ============================================ */
/* Private Function Prototypes                  */
/* ============================================ */

static float Servo_ClampAngle(float angle_deg);
static uint16_t Servo_AngleToPulse(float angle_deg);

/* ============================================ */
/* Private Functions                            */
/* ============================================ */

/**
 * @brief Clamp angle to valid range
 * @param angle_deg Input angle
 * @return Clamped angle (-90 ~ +90)
 */
static float Servo_ClampAngle(float angle_deg)
{
    if (angle_deg < SERVO_ANGLE_MIN) {
        return SERVO_ANGLE_MIN;
    } else if (angle_deg > SERVO_ANGLE_MAX) {
        return SERVO_ANGLE_MAX;
    }
    return angle_deg;
}

/**
 * @brief Convert angle to PWM pulse width
 * @param angle_deg Angle (°)
 * @return PWM pulse width (μs)
 */
static uint16_t Servo_AngleToPulse(float angle_deg)
{
    // Linear mapping: angle [-90, +90] → pulse [1160, 1690]
    // Center: 1400μs, Range: ±265μs
    // Formula: pulse_us = 1400 + (angle_deg / 90.0) * 265
    float pulse_float = SERVO_PULSE_CENTER + (angle_deg / 90.0f) * 265.0f;

    // Round to nearest integer
    uint16_t pulse_us = (uint16_t)(pulse_float + 0.5f);

    // Safety limit check
    if (pulse_us < SERVO_PULSE_SAFE_MIN) {
        pulse_us = SERVO_PULSE_SAFE_MIN;
    } else if (pulse_us > SERVO_PULSE_SAFE_MAX) {
        pulse_us = SERVO_PULSE_SAFE_MAX;
    }

    return pulse_us;
}

/* ============================================ */
/* Public Functions                             */
/* ============================================ */

/**
 * @brief Initialize servo motor (start PWM output, set to center position)
 */
HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim, Servo_Data *data)
{
    // Step 1: Parameter check
    if (htim == NULL || data == NULL) {
        return HAL_ERROR;
    }

    // Step 2: Set initial PWM pulse width to center (1500μs)
    __HAL_TIM_SET_COMPARE(htim, SERVO_TIM_CHANNEL, SERVO_PULSE_CENTER);

    // Step 3: Start PWM output
    if (HAL_TIM_PWM_Start(htim, SERVO_TIM_CHANNEL) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 4: Wait 20ms to ensure servo receives signal
    HAL_Delay(20);

    // Step 5: Initialize data structure
    data->angle_deg = SERVO_ANGLE_CENTER;
    data->pulse_us = SERVO_PULSE_CENTER;
    data->initialized = 1;

    return HAL_OK;
}

/**
 * @brief Set servo angle (with limit protection)
 */
HAL_StatusTypeDef Servo_SetAngle(TIM_HandleTypeDef *htim, float angle_deg, Servo_Data *data)
{
    // Step 1: Parameter check
    if (htim == NULL || data == NULL) {
        return HAL_ERROR;
    }

    // Step 2: Check initialization status
    if (!data->initialized) {
        return HAL_ERROR;
    }

    // Step 3: Angle limit check
    angle_deg = Servo_ClampAngle(angle_deg);

    // Step 4: Convert angle to PWM pulse width
    uint16_t pulse_us = Servo_AngleToPulse(angle_deg);

    // Step 5: Set PWM compare value
    __HAL_TIM_SET_COMPARE(htim, SERVO_TIM_CHANNEL, pulse_us);

    // Step 6: Update data structure
    data->angle_deg = angle_deg;
    data->pulse_us = pulse_us;

    return HAL_OK;
}

/* ============================================ */
/* Test Function                                */
/* ============================================ */

#if SERVO_TEST_ENABLE
/**
 * @brief Servo test program (basic function + full range scan)
 */
void Servo_Test(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart)
{
    Servo_Data servo_data;
    char uart_tx_buffer[150];

    // ========================================
    // Print test banner
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "========================================\r\n"
             "  CoVAPSy Servo Motor Test\r\n"
             "  Model: Reely RS-610WP MG\r\n"
             "========================================\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 1000);

    // ========================================
    // PWM Configuration Diagnostics
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[DIAG] TIM1 Configuration:\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "  Prescaler: %lu\r\n", htim->Init.Prescaler);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "  Period (ARR): %lu\r\n", htim->Init.Period);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "  Current CCR4: %lu\r\n\r\n", __HAL_TIM_GET_COMPARE(htim, SERVO_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // ========================================
    // Initialize servo
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Initializing servo...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    if (Servo_Init(htim, &servo_data) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] Servo initialization failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        while (1);  // Stop execution
    }

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] Servo initialized! Angle: %.1fdeg, Pulse: %uus\r\n",
             servo_data.angle_deg, servo_data.pulse_us);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] CCR4 after init: %lu\r\n\r\n", __HAL_TIM_GET_COMPARE(htim, SERVO_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // ========================================
    // Phase 1: Basic function test (one-time)
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[TEST] Basic function test...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Test -90°
    Servo_SetAngle(htim, -90.0f, &servo_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0001] Angle= -90.0deg -> Pulse=%uus, CCR4=%lu (Left limit, hold 2s)\r\n",
             servo_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, SERVO_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(2000);

    // Test 0°
    Servo_SetAngle(htim, 0.0f, &servo_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0002] Angle=   0.0deg -> Pulse=%uus, CCR4=%lu (Center, hold 2s)\r\n",
             servo_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, SERVO_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(2000);

    // Test +90°
    Servo_SetAngle(htim, +90.0f, &servo_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0003] Angle= +90.0deg -> Pulse=%uus, CCR4=%lu (Right limit, hold 2s)\r\n",
             servo_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, SERVO_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(2000);

    // Return to center
    Servo_SetAngle(htim, 0.0f, &servo_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0004] Angle=   0.0deg -> Pulse=%uus, CCR4=%lu (Back to center)\r\n",
             servo_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, SERVO_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(1000);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] Basic function test completed!\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // ========================================
    // Phase 2: Full range scan test (loop)
    // ========================================
    uint32_t scan_count = 0;

    while (1)
    {
        scan_count++;

        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "----------------------------------------\r\n"
                 "[Scan #%lu] 1160us -> 1690us (Step: 50us, Calibrated)\r\n"
                 "----------------------------------------\r\n",
                 scan_count);
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        uint32_t sample = 0;

        // Scan from 1000μs to 2000μs, step 50μs
        for (uint16_t pulse_us = SERVO_PULSE_MIN; pulse_us <= SERVO_PULSE_MAX; pulse_us += 50)
        {
            sample++;

            // Set PWM pulse width
            __HAL_TIM_SET_COMPARE(htim, SERVO_TIM_CHANNEL, pulse_us);

            // Calculate corresponding angle using calibrated values
            // angle = (pulse - 1400) * 90 / 265
            float angle = (float)(pulse_us - SERVO_PULSE_CENTER) * 90.0f / 265.0f;

            // UART output
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[%04lu] Pulse=%uus -> Angle=%+6.1fdeg\r\n",
                     sample, pulse_us, angle);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

            HAL_Delay(500);  // Hold for 500ms
        }

        // Return to center
        __HAL_TIM_SET_COMPARE(htim, SERVO_TIM_CHANNEL, SERVO_PULSE_CENTER);
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[INFO] Scan completed, back to center (1400us)\r\n"
                 "[INFO] Waiting 3 seconds before next scan...\r\n\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        HAL_Delay(3000);  // Wait 3 seconds
    }
}
#endif

#endif // SERVO_ENABLE
