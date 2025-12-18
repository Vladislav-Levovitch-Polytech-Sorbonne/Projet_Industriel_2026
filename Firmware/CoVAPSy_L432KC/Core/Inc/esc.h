/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : esc.h
  * @brief          : Tamiya TBLE-04S ESC Driver Header
  * @author         : CoVAPSy Team
  * @date           : 2025-12-17
  ******************************************************************************
  * @attention
  *
  * This driver supports the Tamiya TBLE-04S Brushless ESC.
  * Features:
  *   - Throttle control: -100% (Full Reverse) to +100% (Full Forward)
  *   - PWM interface: 50Hz (20ms period)
  *   - Pulse width: 1000μs ~ 2000μs (1500μs = Neutral)
  *   - TIM1 Channel 1 (PA8)
  *   - Bidirectional mode (Forward/Brake/Reverse)
  *   - Arming sequence for safety
  *   - Dependency injection design pattern
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ESC_H
#define ESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#if ESC_ENABLE

#include "stm32l4xx_hal.h"

/* ============================================ */
/* Data Structure Definitions                   */
/* ============================================ */

/**
 * @brief ESC (Electronic Speed Controller) data structure
 */
typedef struct {
    float throttle_percent;     // Current throttle (%) range: -100 ~ +100
                               // +100% = Full Forward  (1000μs)
                               //    0% = Neutral/Stop  (1500μs)
                               // -100% = Full Reverse  (2000μs)
    uint16_t pulse_us;         // PWM pulse width (μs) range: 1000 ~ 2000
    uint8_t initialized;       // Initialization flag (0=Not initialized, 1=Initialized)
    uint8_t armed;            // Arming flag (0=Disarmed, 1=Armed)
} ESC_Data;

/* ============================================ */
/* PWM Configuration                            */
/* ============================================ */

#define ESC_PWM_FREQ_HZ         50                // PWM frequency 50Hz (20ms period)
#define ESC_PWM_PERIOD_US       20000             // PWM period 20000μs = 20ms
#define ESC_TIM_CHANNEL         TIM_CHANNEL_1     // TIM1 Channel 1 (PA8)

/* ============================================ */
/* ESC Pulse Width Range (μs)                  */
/* ============================================ */

#define ESC_PULSE_MIN           1000              // Minimum pulse width 1000μs → Full Forward
#define ESC_PULSE_NEUTRAL       1500              // Neutral pulse width 1500μs → Stop
#define ESC_PULSE_MAX           2000              // Maximum pulse width 2000μs → Full Reverse/Brake

/* ============================================ */
/* ESC Throttle Range (percent)                */
/* ============================================ */

#define ESC_THROTTLE_MIN        -100.0f           // Full Reverse -100%
#define ESC_THROTTLE_NEUTRAL    0.0f              // Neutral 0%
#define ESC_THROTTLE_MAX        +100.0f           // Full Forward +100%

/* ============================================ */
/* Safety and Arming Configuration             */
/* ============================================ */

#define ESC_ARMING_PULSE_US     1500              // Arming pulse (neutral position)
#define ESC_ARMING_DURATION_MS  2000              // Arming duration 2000ms (2 seconds)
#define ESC_DEADZONE_PERCENT    5.0f              // Dead zone ±5% around neutral

/* ============================================ */
/* Bidirectional Mode Configuration            */
/* ============================================ */

#if ESC_BIDIRECTIONAL_ENABLE
    #define ESC_MODE_STRING     "Bidirectional (Forward/Brake/Reverse)"
#else
    #define ESC_MODE_STRING     "Forward Only"
    // Forward only mode: 1000μs = Stop, 2000μs = Full Forward
#endif

/* ============================================ */
/* Function Prototypes                          */
/* ============================================ */

/**
 * @brief Initialize ESC (start PWM output, set to neutral position)
 * @param htim Timer handle pointer (e.g. &htim1)
 * @param data ESC data structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef ESC_Init(TIM_HandleTypeDef *htim, ESC_Data *data);

/**
 * @brief Arm ESC (execute arming sequence - REQUIRED before throttle control)
 * @param htim Timer handle pointer
 * @param data ESC data structure pointer
 * @param huart UART handle for debug output (can be NULL)
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note This function sends neutral pulse (1500μs) for 2 seconds to arm the ESC.
 *       Must be called after ESC_Init() and before ESC_SetThrottle().
 */
HAL_StatusTypeDef ESC_Arm(TIM_HandleTypeDef *htim, ESC_Data *data, UART_HandleTypeDef *huart);

/**
 * @brief Set ESC throttle (with limit protection and arming check)
 * @param htim Timer handle pointer
 * @param throttle_percent Target throttle (%), range -100 ~ +100
 *                         -100% = Full Reverse (if bidirectional enabled)
 *                            0% = Neutral (Stop)
 *                         +100% = Full Forward
 * @param data ESC data structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure (e.g. not armed)
 */
HAL_StatusTypeDef ESC_SetThrottle(TIM_HandleTypeDef *htim, float throttle_percent, ESC_Data *data);

/**
 * @brief Emergency stop - immediately set throttle to neutral
 * @param htim Timer handle pointer
 * @param data ESC data structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef ESC_Stop(TIM_HandleTypeDef *htim, ESC_Data *data);

/**
 * @brief Disarm ESC (set to neutral and disarm)
 * @param htim Timer handle pointer
 * @param data ESC data structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef ESC_Disarm(TIM_HandleTypeDef *htim, ESC_Data *data);

#if ESC_BIDIRECTIONAL_ENABLE
/**
 * @brief Execute reverse sequence (brake + reverse) for bidirectional ESC
 * @param htim Timer handle pointer
 * @param throttle_percent Target reverse throttle (-100 ~ -1), will be clamped
 * @param data ESC data structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note This function implements the 2-step reverse sequence:
 *       1. First reverse pulse → Brake (1 second)
 *       2. Return to neutral (500ms)
 *       3. Second reverse pulse → Actual reverse motion
 *       Required after forward motion to prevent gear damage.
 */
HAL_StatusTypeDef ESC_ReverseSequence(TIM_HandleTypeDef *htim, float throttle_percent, ESC_Data *data);
#endif

#if ESC_TEST_ENABLE
/**
 * @brief ESC test program (arming + throttle test + full range scan)
 * @param htim Timer handle (&htim1)
 * @param huart UART handle (&huart2)
 */
void ESC_Test(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart);
#endif

#endif // ESC_ENABLE

#ifdef __cplusplus
}
#endif

#endif /* ESC_H */
