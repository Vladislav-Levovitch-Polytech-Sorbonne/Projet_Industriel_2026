/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : esc.c
  * @brief          : Tamiya TBLE-04S ESC Driver Implementation
  * @author         : CoVAPSy Team
  * @date           : 2025-12-17
  ******************************************************************************
  */
/* USER CODE END Header */

#include "esc.h"

#if ESC_ENABLE

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============================================ */
/* Private Function Prototypes                  */
/* ============================================ */

static float ESC_ClampThrottle(float throttle_percent);
static uint16_t ESC_ThrottleToPulse(float throttle_percent);
static uint8_t ESC_IsInDeadZone(float throttle_percent);

/* ============================================ */
/* Private Functions                            */
/* ============================================ */

/**
 * @brief Clamp throttle to valid range
 * @param throttle_percent Input throttle
 * @return Clamped throttle (-100 ~ +100 or 0 ~ +100)
 */
static float ESC_ClampThrottle(float throttle_percent)
{
#if ESC_BIDIRECTIONAL_ENABLE
    // Bidirectional mode: -100% ~ +100%
    if (throttle_percent < ESC_THROTTLE_MIN) {
        return ESC_THROTTLE_MIN;
    } else if (throttle_percent > ESC_THROTTLE_MAX) {
        return ESC_THROTTLE_MAX;
    }
#else
    // Forward only mode: 0% ~ +100%
    if (throttle_percent < 0.0f) {
        return 0.0f;
    } else if (throttle_percent > ESC_THROTTLE_MAX) {
        return ESC_THROTTLE_MAX;
    }
#endif
    return throttle_percent;
}

/**
 * @brief Check if throttle is in dead zone (near neutral)
 * @param throttle_percent Throttle value
 * @return 1 if in dead zone, 0 otherwise
 */
static uint8_t ESC_IsInDeadZone(float throttle_percent)
{
    return (fabsf(throttle_percent) <= ESC_DEADZONE_PERCENT) ? 1 : 0;
}

/**
 * @brief Convert throttle percentage to PWM pulse width
 * @param throttle_percent Throttle (%)
 * @return PWM pulse width (μs)
 */
static uint16_t ESC_ThrottleToPulse(float throttle_percent)
{
    // REVERSED mapping: throttle [-100, +100] → pulse [2000, 1000]
    // +100% (forward) → 1000μs
    //    0% (neutral) → 1500μs
    // -100% (reverse) → 2000μs
    // Formula: pulse_us = 1500 - (throttle_percent / 100.0) * 500

    float pulse_float = ESC_PULSE_NEUTRAL - (throttle_percent / 100.0f) * 500.0f;

    // Round to nearest integer
    uint16_t pulse_us = (uint16_t)(pulse_float + 0.5f);

    // Safety limit check
    if (pulse_us < ESC_PULSE_MIN) {
        pulse_us = ESC_PULSE_MIN;
    } else if (pulse_us > ESC_PULSE_MAX) {
        pulse_us = ESC_PULSE_MAX;
    }

    return pulse_us;
}

/* ============================================ */
/* Public Functions                             */
/* ============================================ */

/**
 * @brief Initialize ESC (start PWM output, set to neutral position)
 */
HAL_StatusTypeDef ESC_Init(TIM_HandleTypeDef *htim, ESC_Data *data)
{
    // Step 1: Parameter check
    if (htim == NULL || data == NULL) {
        return HAL_ERROR;
    }

    // Step 2: Set initial PWM pulse width to neutral (1500μs)
    __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, ESC_PULSE_NEUTRAL);

    // Step 3: Start PWM output
    if (HAL_TIM_PWM_Start(htim, ESC_TIM_CHANNEL) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 4: Wait 20ms to ensure ESC receives signal
    HAL_Delay(20);

    // Step 5: Initialize data structure (DISARMED state)
    data->throttle_percent = ESC_THROTTLE_NEUTRAL;
    data->pulse_us = ESC_PULSE_NEUTRAL;
    data->initialized = 1;
    data->armed = 0;  // NOT armed yet - must call ESC_Arm()

    return HAL_OK;
}

/**
 * @brief Arm ESC (execute arming sequence)
 */
HAL_StatusTypeDef ESC_Arm(TIM_HandleTypeDef *htim, ESC_Data *data, UART_HandleTypeDef *huart)
{
    char uart_tx_buffer[100];

    // Step 1: Parameter check
    if (htim == NULL || data == NULL) {
        return HAL_ERROR;
    }

    // Step 2: Check initialization status
    if (!data->initialized) {
        return HAL_ERROR;
    }

    // Step 3: Already armed check
    if (data->armed) {
        if (huart != NULL) {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[WARN] ESC already armed!\r\n");
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        }
        return HAL_OK;
    }

    // Step 4: Execute arming sequence
    if (huart != NULL) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[INFO] Arming ESC... (Neutral pulse for %dms)\r\n", ESC_ARMING_DURATION_MS);
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    }

    // Send neutral pulse (1500μs) for arming duration
    __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, ESC_ARMING_PULSE_US);
    HAL_Delay(ESC_ARMING_DURATION_MS);

    // Step 5: Set armed flag
    data->armed = 1;

    if (huart != NULL) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[OK] ESC armed! Ready for throttle control.\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    }

    return HAL_OK;
}

/**
 * @brief Set ESC throttle (with arming check and limit protection)
 */
HAL_StatusTypeDef ESC_SetThrottle(TIM_HandleTypeDef *htim, float throttle_percent, ESC_Data *data)
{
    // Step 1: Parameter check
    if (htim == NULL || data == NULL) {
        return HAL_ERROR;
    }

    // Step 2: Check initialization status
    if (!data->initialized) {
        return HAL_ERROR;
    }

    // Step 3: Check arming status (CRITICAL SAFETY CHECK)
    if (!data->armed) {
        return HAL_ERROR;  // Refuse to set throttle if not armed
    }

    // Step 4: Throttle limit check and clamping
    throttle_percent = ESC_ClampThrottle(throttle_percent);

    // Step 5: Convert throttle to PWM pulse width
    uint16_t pulse_us = ESC_ThrottleToPulse(throttle_percent);

    // Step 6: Set PWM compare value
    __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, pulse_us);

    // Step 7: Update data structure
    data->throttle_percent = throttle_percent;
    data->pulse_us = pulse_us;

    return HAL_OK;
}

/**
 * @brief Emergency stop - immediately set throttle to neutral
 */
HAL_StatusTypeDef ESC_Stop(TIM_HandleTypeDef *htim, ESC_Data *data)
{
    // Step 1: Parameter check
    if (htim == NULL || data == NULL) {
        return HAL_ERROR;
    }

    // Step 2: Set to neutral immediately (bypass arming check for safety)
    __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, ESC_PULSE_NEUTRAL);

    // Step 3: Update data structure
    data->throttle_percent = ESC_THROTTLE_NEUTRAL;
    data->pulse_us = ESC_PULSE_NEUTRAL;

    return HAL_OK;
}

/**
 * @brief Disarm ESC (set to neutral and clear armed flag)
 */
HAL_StatusTypeDef ESC_Disarm(TIM_HandleTypeDef *htim, ESC_Data *data)
{
    // Step 1: Stop motor first
    if (ESC_Stop(htim, data) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 2: Clear armed flag
    data->armed = 0;

    return HAL_OK;
}

#if ESC_BIDIRECTIONAL_ENABLE
/**
 * @brief Execute reverse sequence (brake + reverse) for bidirectional ESC
 */
HAL_StatusTypeDef ESC_ReverseSequence(TIM_HandleTypeDef *htim, float throttle_percent, ESC_Data *data)
{
    // Step 1: Parameter and state checks
    if (htim == NULL || data == NULL) {
        return HAL_ERROR;
    }

    if (!data->initialized || !data->armed) {
        return HAL_ERROR;
    }

    // Step 2: Ensure throttle is negative (reverse direction)
    if (throttle_percent >= 0.0f) {
        throttle_percent = -25.0f;  // Default to -25% if positive value given
    }

    // Step 3: Clamp to valid reverse range
    throttle_percent = ESC_ClampThrottle(throttle_percent);

    // Step 4: First reverse pulse → Brake (not actual reverse)
    uint16_t pulse_us = ESC_ThrottleToPulse(throttle_percent);
    __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, pulse_us);
    data->pulse_us = pulse_us;
    data->throttle_percent = throttle_percent;
    HAL_Delay(1000);  // Hold brake for 1 second

    // Step 5: Return to neutral (required between brake and reverse)
    __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, ESC_PULSE_NEUTRAL);
    data->pulse_us = ESC_PULSE_NEUTRAL;
    data->throttle_percent = ESC_THROTTLE_NEUTRAL;
    HAL_Delay(500);  // Hold neutral for 500ms

    // Step 6: Second reverse pulse → Actual reverse motion
    pulse_us = ESC_ThrottleToPulse(throttle_percent);
    __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, pulse_us);
    data->pulse_us = pulse_us;
    data->throttle_percent = throttle_percent;

    // Note: Motor is now in reverse, caller should manage duration and return to neutral

    return HAL_OK;
}
#endif

/* ============================================ */
/* Test Function                                */
/* ============================================ */

#if ESC_TEST_ENABLE
/**
 * @brief ESC test program (arming + basic throttle + full range scan)
 */
void ESC_Test(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart)
{
    ESC_Data esc_data;
    char uart_tx_buffer[200];

    // ========================================
    // Print test banner
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "========================================\r\n"
             "  CoVAPSy ESC Test Program\r\n"
             "  Model: Tamiya TBLE-04S\r\n"
             "  Mode: %s\r\n"
             "========================================\r\n\r\n",
             ESC_MODE_STRING);
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
             "  Current CCR1: %lu\r\n\r\n", __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // ========================================
    // Initialize ESC
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Initializing ESC...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    if (ESC_Init(htim, &esc_data) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] ESC initialization failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        while (1);  // Stop execution
    }

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] ESC initialized! Throttle: %.1f%%, Pulse: %uus, Armed: %s\r\n",
             esc_data.throttle_percent, esc_data.pulse_us, esc_data.armed ? "YES" : "NO");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] CCR1 after init: %lu\r\n\r\n", __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // ========================================
    // Arming Sequence
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "========================================\r\n"
             "[SAFETY] ESC Arming Sequence\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    if (ESC_Arm(htim, &esc_data, huart) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] ESC arming failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        while (1);
    }

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] ESC is now ARMED! Armed status: %s\r\n\r\n",
             esc_data.armed ? "YES" : "NO");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    HAL_Delay(1000);

    // ========================================
    // Phase 1: Basic throttle test (one-time)
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "========================================\r\n"
             "[TEST] Basic Throttle Test\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Test 0% (Neutral)
    ESC_SetThrottle(htim, 0.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0001] Throttle=   0.0%% -> Pulse=%uus, CCR1=%lu (Neutral, hold 2s)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(2000);

    // Test 25% Forward
    ESC_SetThrottle(htim, 25.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0002] Throttle= +25.0%% -> Pulse=%uus, CCR1=%lu (25%% Forward, hold 3s)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(3000);

    // Test 50% Forward
    ESC_SetThrottle(htim, 50.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0003] Throttle= +50.0%% -> Pulse=%uus, CCR1=%lu (50%% Forward, hold 3s)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(3000);

    // Test 75% Forward
    ESC_SetThrottle(htim, 75.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0004] Throttle= +75.0%% -> Pulse=%uus, CCR1=%lu (75%% Forward, hold 3s)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(3000);

    // Test 100% Forward
    ESC_SetThrottle(htim, 100.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0005] Throttle=+100.0%% -> Pulse=%uus, CCR1=%lu (FULL Forward, hold 3s)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(3000);

    // Return to neutral
    ESC_SetThrottle(htim, 0.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0006] Throttle=   0.0%% -> Pulse=%uus, CCR1=%lu (Back to neutral)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(2000);

#if ESC_BIDIRECTIONAL_ENABLE
    // Reverse sequence: Forward → Neutral → Brake (1st reverse) → Neutral → Reverse (2nd reverse)
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Testing reverse sequence (requires 2-step process)...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Step 1: First reverse pull → Brake/Stop (not actual reverse)
    ESC_SetThrottle(htim, -25.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0007] Throttle= -25.0%% -> Pulse=%uus, CCR1=%lu (1st pull: BRAKE, hold 1s)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(1000);

    // Step 2: Return to neutral (required between brake and reverse)
    ESC_SetThrottle(htim, 0.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0008] Throttle=   0.0%% -> Pulse=%uus, CCR1=%lu (Back to neutral, hold 500ms)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(500);

    // Step 3: Second reverse pull → Actual reverse motion
    ESC_SetThrottle(htim, -25.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0009] Throttle= -25.0%% -> Pulse=%uus, CCR1=%lu (2nd pull: ACTUAL REVERSE, hold 3s)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(3000);

    // Step 4: Return to neutral
    ESC_SetThrottle(htim, 0.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[0010] Throttle=   0.0%% -> Pulse=%uus, CCR1=%lu (Back to neutral)\r\n",
             esc_data.pulse_us, __HAL_TIM_GET_COMPARE(htim, ESC_TIM_CHANNEL));
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(2000);
#endif

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] Basic throttle test completed!\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // ========================================
    // Test emergency stop
    // ========================================
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "========================================\r\n"
             "[TEST] Emergency Stop Test\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Set 50% throttle
    ESC_SetThrottle(htim, 50.0f, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Set to 50%% throttle...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(1000);

    // Emergency stop
    ESC_Stop(htim, &esc_data);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] Emergency stop executed! Throttle: %.1f%%, Pulse: %uus\r\n\r\n",
             esc_data.throttle_percent, esc_data.pulse_us);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    HAL_Delay(2000);

    // ========================================
    // Phase 2: Full range scan test (loop)
    // ========================================
    uint32_t scan_count = 0;

    while (1)
    {
        scan_count++;

        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "========================================\r\n"
                 "[Scan #%lu] Full Range Test\r\n"
                 "========================================\r\n",
                 scan_count);
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        uint32_t sample = 0;

#if ESC_BIDIRECTIONAL_ENABLE
        // Bidirectional scan: 1000μs to 2000μs (step 50μs)
        for (uint16_t pulse_us = ESC_PULSE_MIN; pulse_us <= ESC_PULSE_MAX; pulse_us += 50)
#else
        // Forward only scan: 1500μs to 2000μs (step 50μs)
        for (uint16_t pulse_us = ESC_PULSE_NEUTRAL; pulse_us <= ESC_PULSE_MAX; pulse_us += 50)
#endif
        {
            sample++;

            // Set PWM pulse width directly
            __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, pulse_us);

            // Calculate corresponding throttle percentage (REVERSED)
            // 1000μs = +100% (forward), 1500μs = 0%, 2000μs = -100% (reverse)
            float throttle = (float)(ESC_PULSE_NEUTRAL - pulse_us) * 100.0f / 500.0f;

            // UART output
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[%04lu] Pulse=%uus -> Throttle=%+6.1f%%\r\n",
                     sample, pulse_us, throttle);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

            HAL_Delay(500);  // Hold for 500ms
        }

        // Return to neutral
        __HAL_TIM_SET_COMPARE(htim, ESC_TIM_CHANNEL, ESC_PULSE_NEUTRAL);
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[INFO] Scan completed, back to neutral (1500us)\r\n"
                 "[INFO] Waiting 3 seconds before next scan...\r\n\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        HAL_Delay(3000);  // Wait 3 seconds
    }
}
#endif

#endif // ESC_ENABLE
