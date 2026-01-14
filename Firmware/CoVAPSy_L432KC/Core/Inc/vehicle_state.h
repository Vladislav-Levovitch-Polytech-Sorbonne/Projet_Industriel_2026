/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vehicle_state.h
  * @brief          : Vehicle Control Layer Header
  * @author         : CoVAPSy Team
  * @date           : 2025-01-07
  ******************************************************************************
  * @attention
  *
  * Vehicle control layer integrating all sensors and actuators.
  * Features:
  *   - Sensor data collection (SHARP rear sensors, BNO055 IMU)
  *   - Basic motion control (steering, throttle)
  *   - Reverse safety protection (SHARP obstacle detection)
  *   - Debug status monitoring (50Hz periodic output)
  *   - Three-layer safety protection (watchdog, rear obstacle, attitude check)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#if VEHICLE_ENABLE

#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// Include all driver headers
#include "servo.h"
#include "esc.h"
#include "sharp_sensor.h"
#include "bno055.h"

/* ============================================ */
/* Vehicle Mode Enumeration                     */
/* ============================================ */

/**
 * @brief Vehicle operation modes
 */
typedef enum {
    VEHICLE_MODE_IDLE = 0,      // Idle mode (default on power-up)
    VEHICLE_MODE_REMOTE,        // Remote control mode (receive Raspberry Pi commands)
    VEHICLE_MODE_MANUAL,        // Manual test mode (Phase 1 testing)
    VEHICLE_MODE_EMERGENCY      // Emergency stop mode (safety triggered)
} Vehicle_Mode;

/* ============================================ */
/* Vehicle State Data Structure                 */
/* ============================================ */

/**
 * @brief Vehicle state data structure - integrates all sensors and actuators
 */
typedef struct {
    /* === Sensor Data === */
    // SHARP rear distance sensors (only used during reversing)
    float sharp_left_distance_cm;      // Rear-left distance (cm)
    float sharp_right_distance_cm;     // Rear-right distance (cm)
    uint8_t sharp_left_valid;          // Data validity flag
    uint8_t sharp_right_valid;

    // BNO055 IMU data
    float roll_deg;                    // Roll angle (deg)
    float pitch_deg;                   // Pitch angle (deg)
    float yaw_deg;                     // Yaw angle (deg)
    float accel_x_mps2;                // X-axis acceleration (m/s²)
    float accel_y_mps2;                // Y-axis acceleration (m/s²)
    float accel_z_mps2;                // Z-axis acceleration (m/s²)
    uint8_t imu_valid;                 // IMU data validity

    /* === Actuator Current State === */
    float current_steering_deg;        // Current steering angle (deg) [-30, +30]
    float current_throttle_percent;    // Current throttle (%) [-100, +100]

    /* === Target Control Values (from upper layer or manual setting) === */
    float target_steering_deg;         // Target steering angle (deg)
    float target_throttle_percent;     // Target throttle (%)

    /* === Vehicle Running State === */
    Vehicle_Mode mode;                 // Current operation mode
    uint8_t is_reversing;              // Reversing flag (0=forward/stop, 1=reversing)
    uint8_t safety_stop_triggered;     // Safety stop triggered flag
    uint32_t control_loop_counter;     // Control loop counter
    uint32_t last_command_timestamp;   // Last command timestamp (ms) - watchdog

    /* === Hardware Handles (dependency injection) === */
    TIM_HandleTypeDef *htim_servo_esc; // TIM1 (for servo and ESC)
    I2C_HandleTypeDef *hi2c_imu;       // I2C1 (for BNO055)
    ADC_HandleTypeDef *hadc_sharp;     // ADC1 (for SHARP sensors)
    UART_HandleTypeDef *huart_debug;   // UART2 (for debug output)

    /* === ADC DMA Buffer for SHARP Sensors === */
    uint16_t *sharp_dma_buffer;        // DMA buffer for ADC (2 channels: PA3, PA4)
    uint32_t sharp_dma_buffer_size;    // DMA buffer size (typically 2)

    /* === Driver Data Structures === */
    Servo_Data servo_data;             // Servo data
    ESC_Data esc_data;                 // ESC data
    BNO055_Data bno055_data;           // IMU data
    Sharp_Data sharp_left_data;        // Left SHARP data
    Sharp_Data sharp_right_data;       // Right SHARP data

} Vehicle_State;

/* ============================================ */
/* Safety Configuration Constants              */
/* ============================================ */

#define VEHICLE_WATCHDOG_TIMEOUT_MS     500     // Watchdog timeout (ms)
#define VEHICLE_REAR_SAFE_DISTANCE_CM   15.0f   // Rear safe distance (cm)
#define VEHICLE_MAX_ROLL_DEG            30.0f   // Maximum roll angle (deg)
#define VEHICLE_MAX_PITCH_DEG           30.0f   // Maximum pitch angle (deg)

/* ============================================ */
/* Function Prototypes                          */
/* ============================================ */

/**
 * @brief Initialize vehicle (all sensors and actuators, set to idle mode)
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note Before calling this function, you must set the hardware handles:
 *       - vehicle->htim_servo_esc = &htim1;
 *       - vehicle->hi2c_imu = &hi2c1;
 *       - vehicle->hadc_sharp = &hadc1;
 *       - vehicle->huart_debug = &huart2;
 *       - vehicle->sharp_dma_buffer = <pointer to DMA buffer>;
 *       - vehicle->sharp_dma_buffer_size = 2;
 */
HAL_StatusTypeDef Vehicle_Init(Vehicle_State *vehicle);

/**
 * @brief Update all sensor data (SHARP, IMU)
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note This function should be called at 50Hz in the control loop
 */
HAL_StatusTypeDef Vehicle_UpdateSensors(Vehicle_State *vehicle);

/**
 * @brief Set target steering angle
 * @param vehicle Vehicle state structure pointer
 * @param steering_deg Target steering angle (deg), range: [-30, +30]
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note Automatically clamps input to valid range
 *       Updates watchdog timestamp
 */
HAL_StatusTypeDef Vehicle_SetTargetSteering(Vehicle_State *vehicle, float steering_deg);

/**
 * @brief Set target throttle
 * @param vehicle Vehicle state structure pointer
 * @param throttle_percent Target throttle (%), range: [-100, +100]
 *                         -100% = Full Reverse
 *                            0% = Stop
 *                         +100% = Full Forward
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note Automatically clamps input to valid range
 *       Updates watchdog timestamp
 *       Updates is_reversing flag based on throttle value
 */
HAL_StatusTypeDef Vehicle_SetTargetThrottle(Vehicle_State *vehicle, float throttle_percent);

/**
 * @brief Set vehicle operation mode
 * @param vehicle Vehicle state structure pointer
 * @param mode Target mode (IDLE, REMOTE, MANUAL, EMERGENCY)
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Vehicle_SetMode(Vehicle_State *vehicle, Vehicle_Mode mode);

/**
 * @brief Vehicle control loop (50Hz periodic execution)
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure (safety check failed)
 *
 * @note This function performs:
 *       1. Safety checks (priority)
 *       2. Servo steering control
 *       3. ESC throttle control (with auto reverse sequence detection)
 *
 * @note If safety check fails, it automatically calls Vehicle_EmergencyStop()
 */
HAL_StatusTypeDef Vehicle_ControlLoop(Vehicle_State *vehicle);

/**
 * @brief Multi-layer safety checks
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK if all checks pass | HAL_ERROR if any check fails
 *
 * @note Safety layers:
 *       1. Watchdog timeout (500ms no command)
 *       2. Rear obstacle detection during reversing (SHARP < 15cm)
 *       3. Attitude abnormal detection (Roll/Pitch > 30°)
 */
HAL_StatusTypeDef Vehicle_CheckSafety(Vehicle_State *vehicle);

/**
 * @brief Emergency stop - immediately stop ESC and center servo
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note Sets mode to VEHICLE_MODE_EMERGENCY
 *       Clears all target values
 */
HAL_StatusTypeDef Vehicle_EmergencyStop(Vehicle_State *vehicle);

/**
 * @brief Print vehicle status to UART (debug monitoring)
 * @param vehicle Vehicle state structure pointer
 *
 * @note Output format:
 *       [count] Mode | Steer | Throttle | Rev | IMU(R P Y) | SHARP(L R)
 */
void Vehicle_PrintStatus(Vehicle_State *vehicle);

#if VEHICLE_TEST_ENABLE
/**
 * @brief Vehicle test program (Phase 1 whole vehicle testing)
 * @param htim Timer handle (&htim1)
 * @param hi2c I2C handle (&hi2c1)
 * @param hadc ADC handle (&hadc1)
 * @param huart UART handle (&huart2)
 *
 * @note Test sequence:
 *       1. Initialize vehicle
 *       2. Arm ESC
 *       3. Forward straight (3s)
 *       4. Forward left turn (2s)
 *       5. Forward right turn (2s)
 *       6. Reverse with SHARP safety (auto-stop if obstacle < 15cm)
 */
void Vehicle_Test(TIM_HandleTypeDef *htim, I2C_HandleTypeDef *hi2c,
                  ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart);
#endif

#endif // VEHICLE_ENABLE

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_STATE_H */
