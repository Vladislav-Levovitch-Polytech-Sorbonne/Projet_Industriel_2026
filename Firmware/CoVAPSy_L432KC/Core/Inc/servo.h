/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : servo.h
  * @brief          : Reely RS-610WP MG Servo Driver Header
  * @author         : CoVAPSy Team
  * @date           : 2025-12-17
  ******************************************************************************
  * @attention
  *
  * This driver supports the Reely RS-610WP MG waterproof servo motor.
  * Features:
  *   - Angle control: -90° to +90°
  *   - PWM interface: 50Hz (20ms period)
  *   - Pulse width: 1000μs ~ 2000μs
  *   - TIM1 Channel 4 (PA11)
  *   - Dependency injection design pattern
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#if SERVO_ENABLE

#include "stm32l4xx_hal.h"

/* ============================================ */
/* Data Structure Definitions                   */
/* ============================================ */

/**
 * @brief Servo motor data structure
 */
typedef struct {
    float angle_deg;        // Current angle (°) range: -90 ~ +90
    uint16_t pulse_us;      // PWM pulse width (μs) range: 1000 ~ 2000
    uint8_t initialized;    // Initialization flag (0=Not initialized, 1=Initialized)
} Servo_Data;

/* ============================================ */
/* PWM Configuration                            */
/* ============================================ */

#define SERVO_PWM_FREQ_HZ       50                // PWM frequency 50Hz (20ms period)
#define SERVO_PWM_PERIOD_US     20000             // PWM period 20000μs = 20ms
#define SERVO_TIM_CHANNEL       TIM_CHANNEL_4     // TIM1 Channel 4 (PA11)

/* ============================================ */
/* Servo Pulse Width Range (μs)                */
/* ============================================ */

#define SERVO_PULSE_MIN         1160              // Minimum pulse width 1160μs → -90° (measured)
#define SERVO_PULSE_CENTER      1400              // Center pulse width 1400μs → 0° (measured)
#define SERVO_PULSE_MAX         1690              // Maximum pulse width 1690μs → +90° (measured)

/* ============================================ */
/* Servo Angle Range (degrees)                 */
/* ============================================ */

#define SERVO_ANGLE_MIN         -90.0f            // Minimum angle -90°
#define SERVO_ANGLE_CENTER      0.0f              // Center angle 0°
#define SERVO_ANGLE_MAX         +90.0f            // Maximum angle +90°

/* ============================================ */
/* Safety Limits                                */
/* ============================================ */

#define SERVO_PULSE_SAFE_MIN    1100              // Safe minimum (60μs margin from measured 1160μs)
#define SERVO_PULSE_SAFE_MAX    1750              // Safe maximum (60μs margin from measured 1690μs)

/* ============================================ */
/* Function Prototypes                          */
/* ============================================ */

/**
 * @brief Initialize servo motor (start PWM output, set to center position)
 * @param htim Timer handle pointer (e.g. &htim1)
 * @param data Servo data structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Servo_Init(TIM_HandleTypeDef *htim, Servo_Data *data);

/**
 * @brief Set servo angle (with limit protection)
 * @param htim Timer handle pointer
 * @param angle_deg Target angle (°), range -90 ~ +90
 * @param data Servo data structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Servo_SetAngle(TIM_HandleTypeDef *htim, float angle_deg, Servo_Data *data);

#if SERVO_TEST_ENABLE
/**
 * @brief Servo test program (basic function + full range scan)
 * @param htim Timer handle (&htim1)
 * @param huart UART handle (&huart2)
 */
void Servo_Test(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart);
#endif

#endif // SERVO_ENABLE

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
