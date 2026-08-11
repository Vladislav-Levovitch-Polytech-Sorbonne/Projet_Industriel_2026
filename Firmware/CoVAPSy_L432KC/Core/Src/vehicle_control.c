/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vehicle_control.c
  * @brief          : Vehicle Control Layer Implementation
  * @author         : CoVAPSy Team
  * @date           : 2025-01-07
  ******************************************************************************
  * @attention
  *
  * Vehicle control layer integrating all sensors and actuators.
  * Implements:
  *   - Sensor data collection (SHARP rear sensors, BNO055 IMU)
  *   - Basic motion control (steering angle, throttle speed)
  *   - Reverse safety protection (SHARP obstacle detection)
  *   - Debug status monitoring (50Hz periodic output)
  *   - Three-layer safety protection system
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "vehicle_state.h"

#if VEHICLE_ENABLE

/* ============================================ */
/* Vehicle_Init - Initialize Vehicle            */
/* ============================================ */

/**
 * @brief Initialize vehicle - all sensors and actuators
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Vehicle_Init(Vehicle_State *vehicle)
{
    // 1. Parameter check
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // 2. Hardware handles (must be set by caller before calling this function)
    // vehicle->htim_servo_esc, hi2c_imu, hadc_sharp, huart_debug

    // 3. Initialize Servo (TIM1_CH4)
    if (Servo_Init(vehicle->htim_servo_esc, &vehicle->servo_data) != HAL_OK) {
        char msg[] = "[Vehicle] ERROR: Servo init failed!\r\n";
        HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
        return HAL_ERROR;
    }

    // 4. Initialize ESC (TIM1_CH1)
    if (ESC_Init(vehicle->htim_servo_esc, &vehicle->esc_data) != HAL_OK) {
        char msg[] = "[Vehicle] ERROR: ESC init failed!\r\n";
        HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
        return HAL_ERROR;
    }

    // 5. Initialize BNO055 IMU
    if (BNO055_Init(vehicle->hi2c_imu, 1) != HAL_OK) {  // 1 = perform reset on init
        char msg[] = "[Vehicle] WARNING: BNO055 init failed! IMU data will be invalid.\r\n";
        HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
        // Continue even if IMU fails (non-critical for basic testing)
    }

    // 6. Initialize SHARP sensors using ADC-DMA
    if (Sharp_Init(vehicle->hadc_sharp, vehicle->sharp_dma_buffer, vehicle->sharp_dma_buffer_size) != HAL_OK) {
        char msg[] = "[Vehicle] WARNING: SHARP sensors init failed!\r\n";
        HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
    }

    // 7. Initialize vehicle state variables
    vehicle->mode = VEHICLE_MODE_IDLE;
    vehicle->is_reversing = 0;
    vehicle->safety_stop_triggered = 0;
    vehicle->control_loop_counter = 0;
    vehicle->target_steering_deg = 0.0f;
    vehicle->target_throttle_percent = 0.0f;
    vehicle->current_steering_deg = 0.0f;
    vehicle->current_throttle_percent = 0.0f;
    vehicle->last_command_timestamp = HAL_GetTick();

    // 8. Clear sensor data
    vehicle->sharp_left_distance_cm = 0.0f;
    vehicle->sharp_right_distance_cm = 0.0f;
    vehicle->sharp_left_valid = 0;
    vehicle->sharp_right_valid = 0;
    vehicle->roll_deg = 0.0f;
    vehicle->pitch_deg = 0.0f;
    vehicle->yaw_deg = 0.0f;
    vehicle->accel_x_mps2 = 0.0f;
    vehicle->accel_y_mps2 = 0.0f;
    vehicle->accel_z_mps2 = 0.0f;
    vehicle->imu_valid = 0;

    // 9. UART output initialization success message
    char msg[] = "[Vehicle] Initialization complete!\r\n";
    HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);

    return HAL_OK;
}

/* ============================================ */
/* Vehicle_UpdateSensors - Update Sensor Data   */
/* ============================================ */

/**
 * @brief Update all sensor data (SHARP, IMU)
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Vehicle_UpdateSensors(Vehicle_State *vehicle)
{
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // 1. Update BNO055 IMU (attitude + acceleration)
    if (BNO055_ReadAll(vehicle->hi2c_imu, &vehicle->bno055_data) == HAL_OK) {
        vehicle->roll_deg = vehicle->bno055_data.euler_roll;
        vehicle->pitch_deg = vehicle->bno055_data.euler_pitch;
        vehicle->yaw_deg = vehicle->bno055_data.euler_heading;  // Note: heading, not yaw
        vehicle->accel_x_mps2 = vehicle->bno055_data.linear_accel_x;
        vehicle->accel_y_mps2 = vehicle->bno055_data.linear_accel_y;
        vehicle->accel_z_mps2 = vehicle->bno055_data.linear_accel_z;
        vehicle->imu_valid = 1;
    } else {
        vehicle->imu_valid = 0;
    }

    // 2. Update SHARP sensors (rear left and right)
    // Note: Only meaningful during reversing, but read every time
    // Sharp_ReadData is void type, always succeeds if DMA buffer is valid
    Sharp_ReadData(vehicle->sharp_dma_buffer, 0, &vehicle->sharp_left_data);  // Channel 0 = PA3/ADC_IN8
    vehicle->sharp_left_distance_cm = vehicle->sharp_left_data.distance_cm;
    vehicle->sharp_left_valid = (vehicle->sharp_left_data.out_of_range == 0) ? 1 : 0;

    Sharp_ReadData(vehicle->sharp_dma_buffer, 1, &vehicle->sharp_right_data);  // Channel 1 = PA6/ADC_IN11
    vehicle->sharp_right_distance_cm = vehicle->sharp_right_data.distance_cm;
    vehicle->sharp_right_valid = (vehicle->sharp_right_data.out_of_range == 0) ? 1 : 0;

    return HAL_OK;
}

/* ============================================ */
/* Vehicle_SetTargetSteering - Set Steering     */
/* ============================================ */

/**
 * @brief Set target steering angle
 * @param vehicle Vehicle state structure pointer
 * @param steering_deg Target steering angle (deg), range: [-30, +30]
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Vehicle_SetTargetSteering(Vehicle_State *vehicle, float steering_deg)
{
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // Clamp to valid range: [-30, +30] deg
    if (steering_deg < -30.0f) steering_deg = -30.0f;
    if (steering_deg > +30.0f) steering_deg = +30.0f;

    vehicle->target_steering_deg = steering_deg;
    vehicle->last_command_timestamp = HAL_GetTick();  // Update watchdog timestamp

    return HAL_OK;
}

/* ============================================ */
/* Vehicle_SetTargetThrottle - Set Throttle     */
/* ============================================ */

/**
 * @brief Set target throttle
 * @param vehicle Vehicle state structure pointer
 * @param throttle_percent Target throttle (%), range: [-100, +100]
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Vehicle_SetTargetThrottle(Vehicle_State *vehicle, float throttle_percent)
{
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // Clamp to valid range: [-100, +100] %
    if (throttle_percent < -100.0f) throttle_percent = -100.0f;
    if (throttle_percent > +100.0f) throttle_percent = +100.0f;

    vehicle->target_throttle_percent = throttle_percent;
    vehicle->last_command_timestamp = HAL_GetTick();  // Update watchdog timestamp

    // Update reversing flag (with deadzone protection)
    if (throttle_percent < -5.0f) {  // Deadzone: ignore small negative values
        vehicle->is_reversing = 1;
    } else {
        vehicle->is_reversing = 0;
    }

    return HAL_OK;
}

/* ============================================ */
/* Vehicle_SetMode - Set Operation Mode         */
/* ============================================ */

/**
 * @brief Set vehicle operation mode
 * @param vehicle Vehicle state structure pointer
 * @param mode Target mode (IDLE, REMOTE, MANUAL, EMERGENCY)
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Vehicle_SetMode(Vehicle_State *vehicle, Vehicle_Mode mode)
{
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // Recovery from EMERGENCY mode - only allow switching to IDLE first
    if (vehicle->mode == VEHICLE_MODE_EMERGENCY) {
        if (mode == VEHICLE_MODE_IDLE) {
            // Clear safety flag when recovering to IDLE
            vehicle->safety_stop_triggered = 0;
            vehicle->target_steering_deg = 0.0f;
            vehicle->target_throttle_percent = 0.0f;
            vehicle->is_reversing = 0;

            char msg[] = "[Vehicle] Recovery from EMERGENCY - safety flag cleared\r\n";
            HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
        } else if (mode != VEHICLE_MODE_EMERGENCY) {
            // Must go through IDLE first
            char msg[] = "[Vehicle] ERROR: Must switch to IDLE before other modes\r\n";
            HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
            return HAL_ERROR;
        }
    }

    vehicle->mode = mode;

    // Update watchdog timestamp when entering REMOTE mode
    if (mode == VEHICLE_MODE_REMOTE) {
        vehicle->last_command_timestamp = HAL_GetTick();
    }

    // UART output mode change
    char msg[80];
    snprintf(msg, sizeof(msg), "[Vehicle] Mode changed to: %d\r\n", mode);
    HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);

    return HAL_OK;
}

/* ============================================ */
/* Vehicle_CheckSafety - Multi-Layer Safety     */
/* ============================================ */

/**
 * @brief Multi-layer safety checks
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK if all checks pass | HAL_ERROR if any check fails
 */
HAL_StatusTypeDef Vehicle_CheckSafety(Vehicle_State *vehicle)
{
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // === Safety Check 1: Watchdog Timeout (500ms no command) ===
    // Note: Only enabled in REMOTE mode (waiting for Raspberry Pi commands)
    // In MANUAL mode, watchdog is disabled for testing purposes
    // Configurable via VEHICLE_WATCHDOG_ENABLE in config.h
#if VEHICLE_WATCHDOG_ENABLE
    if (vehicle->mode == VEHICLE_MODE_REMOTE) {
        uint32_t time_since_last_cmd = HAL_GetTick() - vehicle->last_command_timestamp;
        if (time_since_last_cmd > VEHICLE_WATCHDOG_TIMEOUT_MS) {
            vehicle->safety_stop_triggered = 1;
            char msg[100];
            snprintf(msg, sizeof(msg),
                     "[SAFETY] Watchdog timeout! No command for >%u ms\r\n",
                     (unsigned int)VEHICLE_WATCHDOG_TIMEOUT_MS);
            HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
            return HAL_ERROR;
        }
    }
#endif /* VEHICLE_WATCHDOG_ENABLE */

    // === Safety Check 2: Rear Obstacle Detection During Reversing (SHARP) ===
    if (vehicle->is_reversing) {
        float min_safe_distance = VEHICLE_REAR_SAFE_DISTANCE_CM;

        // Check rear-left
        if (vehicle->sharp_left_valid &&
            vehicle->sharp_left_distance_cm < min_safe_distance) {
            vehicle->safety_stop_triggered = 1;
            char msg[120];
            snprintf(msg, sizeof(msg),
                     "[SAFETY] Rear-left obstacle detected! Distance: %.1f cm (min: %.1f cm)\r\n",
                     vehicle->sharp_left_distance_cm, min_safe_distance);
            HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
            return HAL_ERROR;
        }

        // Check rear-right
        if (vehicle->sharp_right_valid &&
            vehicle->sharp_right_distance_cm < min_safe_distance) {
            vehicle->safety_stop_triggered = 1;
            char msg[120];
            snprintf(msg, sizeof(msg),
                     "[SAFETY] Rear-right obstacle detected! Distance: %.1f cm (min: %.1f cm)\r\n",
                     vehicle->sharp_right_distance_cm, min_safe_distance);
            HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
            return HAL_ERROR;
        }
    }

    // === Safety Check 3: IMU Attitude Abnormal (Rollover Detection) ===
    if (vehicle->imu_valid) {
        float max_roll = VEHICLE_MAX_ROLL_DEG;
        float max_pitch = VEHICLE_MAX_PITCH_DEG;

        if (fabs(vehicle->roll_deg) > max_roll ||
            fabs(vehicle->pitch_deg) > max_pitch) {
            vehicle->safety_stop_triggered = 1;
            char msg[150];
            snprintf(msg, sizeof(msg),
                     "[SAFETY] Attitude abnormal! Roll: %.1fdeg (max: %.1fdeg), Pitch: %.1fdeg (max: %.1fdeg)\r\n",
                     vehicle->roll_deg, max_roll, vehicle->pitch_deg, max_pitch);
            HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);
            return HAL_ERROR;
        }
    }

    // All checks passed
    return HAL_OK;
}

/* ============================================ */
/* Vehicle_ControlLoop - Main Control Loop      */
/* ============================================ */

/**
 * @brief Vehicle control loop (50Hz periodic execution)
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure (safety check failed)
 */
HAL_StatusTypeDef Vehicle_ControlLoop(Vehicle_State *vehicle)
{
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // 1. Increment control loop counter
    vehicle->control_loop_counter++;

    // 2. Safety check (highest priority)
    if (Vehicle_CheckSafety(vehicle) != HAL_OK) {
        // Safety check failed → Emergency stop
        Vehicle_EmergencyStop(vehicle);
        return HAL_ERROR;
    }

    // 3. Check operation mode
    if (vehicle->mode == VEHICLE_MODE_IDLE ||
        vehicle->mode == VEHICLE_MODE_EMERGENCY) {
        // Idle/Emergency mode: do not execute control
        return HAL_OK;
    }

    // 4. Actuator control: Servo (steering)
    if (Servo_SetAngle(vehicle->htim_servo_esc,
                       vehicle->target_steering_deg,
                       &vehicle->servo_data) == HAL_OK) {
        vehicle->current_steering_deg = vehicle->servo_data.angle_deg;
    }

    // 5. Actuator control: ESC (throttle)
    // Note: If switching to reverse, need to execute 2-step reverse sequence
    float current_throttle = vehicle->current_throttle_percent;
    float target_throttle = vehicle->target_throttle_percent;

    // Detect if reverse sequence is needed
    // Trigger conditions:
    //   1. Forward → Reverse: current > 5% AND target < -5%
    //   2. Neutral/Stop → Reverse: current >= -5% AND target < -5% (new case)
    if (target_throttle < -5.0f && current_throttle >= -5.0f) {
        // Entering reverse mode → Execute 2-step reverse sequence
        if (ESC_ReverseSequence(vehicle->htim_servo_esc,
                                target_throttle,
                                &vehicle->esc_data) == HAL_OK) {
            vehicle->current_throttle_percent = vehicle->esc_data.throttle_percent;
        }
    } else {
        // Normal throttle control (forward, maintain reverse, or stop)
        if (ESC_SetThrottle(vehicle->htim_servo_esc,
                            target_throttle,
                            &vehicle->esc_data) == HAL_OK) {
            vehicle->current_throttle_percent = vehicle->esc_data.throttle_percent;
        }
    }

    return HAL_OK;
}

/* ============================================ */
/* Vehicle_EmergencyStop - Emergency Stop       */
/* ============================================ */

/**
 * @brief Emergency stop - immediately stop ESC and center servo
 * @param vehicle Vehicle state structure pointer
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef Vehicle_EmergencyStop(Vehicle_State *vehicle)
{
    if (vehicle == NULL) {
        return HAL_ERROR;
    }

    // 1. Immediately stop ESC
    ESC_Stop(vehicle->htim_servo_esc, &vehicle->esc_data);

    // 2. Center servo
    Servo_SetAngle(vehicle->htim_servo_esc, 0.0f, &vehicle->servo_data);

    // 3. Update state
    vehicle->mode = VEHICLE_MODE_EMERGENCY;
    vehicle->current_throttle_percent = 0.0f;
    vehicle->target_throttle_percent = 0.0f;
    vehicle->is_reversing = 0;

    // 4. UART output
    char msg[] = "[Vehicle] *** EMERGENCY STOP ACTIVATED! ***\r\n";
    HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)msg, strlen(msg), 100);

    return HAL_OK;
}

/* ============================================ */
/* Vehicle_PrintStatus - Status Monitoring      */
/* ============================================ */

/**
 * @brief Print vehicle status to UART (debug monitoring)
 * @param vehicle Vehicle state structure pointer
 */
void Vehicle_PrintStatus(Vehicle_State *vehicle)
{
    if (vehicle == NULL) {
        return;
    }

    char buffer[350];
    char sharp_left_str[20];
    char sharp_right_str[20];

    // Format left SHARP sensor data
    if (vehicle->sharp_left_valid && vehicle->sharp_left_distance_cm > 0.0f) {
        snprintf(sharp_left_str, sizeof(sharp_left_str), "%.1fcm", vehicle->sharp_left_distance_cm);
    } else {
        snprintf(sharp_left_str, sizeof(sharp_left_str), "OUT_OF_RANGE");
    }

    // Format right SHARP sensor data
    if (vehicle->sharp_right_valid && vehicle->sharp_right_distance_cm > 0.0f) {
        snprintf(sharp_right_str, sizeof(sharp_right_str), "%.1fcm", vehicle->sharp_right_distance_cm);
    } else {
        snprintf(sharp_right_str, sizeof(sharp_right_str), "OUT_OF_RANGE");
    }

    snprintf(buffer, sizeof(buffer),
             "[%05lu] Mode:%d | Steer:%.1fdeg | Throttle:%.1f%% | Rev:%d | "
             "IMU(R:%.1fdeg P:%.1fdeg Y:%.1fdeg) | SHARP(L:%s R:%s)\r\n",
             vehicle->control_loop_counter,
             vehicle->mode,
             vehicle->current_steering_deg,
             vehicle->current_throttle_percent,
             vehicle->is_reversing,
             vehicle->roll_deg,
             vehicle->pitch_deg,
             vehicle->yaw_deg,
             sharp_left_str,
             sharp_right_str);

    HAL_UART_Transmit(vehicle->huart_debug, (uint8_t*)buffer, strlen(buffer), 100);
}

/* ============================================ */
/* Vehicle_Test - Phase 1 Test Program          */
/* ============================================ */

#if VEHICLE_TEST_ENABLE

/**
 * @brief Vehicle test program (Phase 1 whole vehicle testing)
 * @param htim Timer handle (&htim1)
 * @param hi2c I2C handle (&hi2c1)
 * @param hadc ADC handle (&hadc1)
 * @param huart UART handle (&huart2)
 */
void Vehicle_Test(TIM_HandleTypeDef *htim, I2C_HandleTypeDef *hi2c,
                  ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart)
{
    char uart_tx_buffer[250];
    Vehicle_State vehicle = {0};  // Zero-initialize

    // DMA buffer for SHARP sensors (2 channels: PA3, PA6)
    static uint16_t sharp_adc_dma_buffer[2];

    // === Step 1: Initialize Vehicle ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "  CoVAPSy Phase 1 Vehicle Test\r\n"
             "  Control Frequency: 50Hz (20ms)\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Set hardware handles
    vehicle.htim_servo_esc = htim;
    vehicle.hi2c_imu = hi2c;
    vehicle.hadc_sharp = hadc;
    vehicle.huart_debug = huart;
    vehicle.sharp_dma_buffer = sharp_adc_dma_buffer;
    vehicle.sharp_dma_buffer_size = 2;

    // Initialize
    if (Vehicle_Init(&vehicle) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] Vehicle initialization failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        return;
    }

    // === Step 2: Arm ESC ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n[INFO] Arming ESC (2 seconds neutral pulse)...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    if (ESC_Arm(htim, &vehicle.esc_data, huart) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] ESC arming failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        return;
    }

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] ESC armed successfully!\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // === Step 3: Set to manual test mode ===
    Vehicle_SetMode(&vehicle, VEHICLE_MODE_MANUAL);

    // === Step 4: Test Sequence ===

    // --- Test 4.1: Forward Straight (3 seconds) ---
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "[TEST 1] Forward Straight\r\n"
             "  Throttle: 30%%\r\n"
             "  Steering: 0deg\r\n"
             "  Duration: 3 seconds\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    Vehicle_SetTargetSteering(&vehicle, 0.0f);    // Straight
    Vehicle_SetTargetThrottle(&vehicle, 30.0f);   // 30% forward

    for (int i = 0; i < 150; i++) {  // 3s @ 50Hz = 150 cycles
        Vehicle_UpdateSensors(&vehicle);
        Vehicle_ControlLoop(&vehicle);

        if (i % 25 == 0) {  // Print status every 0.5s
            Vehicle_PrintStatus(&vehicle);
        }

        HAL_Delay(20);  // 50Hz = 20ms
    }

    // Stop
    Vehicle_SetTargetThrottle(&vehicle, 0.0f);
    Vehicle_UpdateSensors(&vehicle);
    Vehicle_ControlLoop(&vehicle);
    HAL_Delay(1000);

    // --- Test 4.2: Forward Left Turn (2 seconds) ---
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "[TEST 2] Forward Left Turn\r\n"
             "  Throttle: 30%%\r\n"
             "  Steering: -20deg (left)\r\n"
             "  Duration: 2 seconds\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    Vehicle_SetTargetSteering(&vehicle, -20.0f);  // Left turn
    Vehicle_SetTargetThrottle(&vehicle, 30.0f);   // 30% forward

    for (int i = 0; i < 100; i++) {  // 2s @ 50Hz
        Vehicle_UpdateSensors(&vehicle);
        Vehicle_ControlLoop(&vehicle);

        if (i % 25 == 0) {
            Vehicle_PrintStatus(&vehicle);
        }

        HAL_Delay(20);
    }

    // Stop
    Vehicle_SetTargetThrottle(&vehicle, 0.0f);
    Vehicle_SetTargetSteering(&vehicle, 0.0f);
    Vehicle_UpdateSensors(&vehicle);
    Vehicle_ControlLoop(&vehicle);
    HAL_Delay(1000);

    // --- Test 4.3: Forward Right Turn (2 seconds) ---
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "[TEST 3] Forward Right Turn\r\n"
             "  Throttle: 30%%\r\n"
             "  Steering: +20deg (right)\r\n"
             "  Duration: 2 seconds\r\n"
             "========================================\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    Vehicle_SetTargetSteering(&vehicle, +20.0f);  // Right turn
    Vehicle_SetTargetThrottle(&vehicle, 30.0f);   // 30% forward

    for (int i = 0; i < 100; i++) {  // 2s @ 50Hz
        Vehicle_UpdateSensors(&vehicle);
        Vehicle_ControlLoop(&vehicle);

        if (i % 25 == 0) {
            Vehicle_PrintStatus(&vehicle);
        }

        HAL_Delay(20);
    }

    // Stop
    Vehicle_SetTargetThrottle(&vehicle, 0.0f);
    Vehicle_SetTargetSteering(&vehicle, 0.0f);
    Vehicle_UpdateSensors(&vehicle);
    Vehicle_ControlLoop(&vehicle);
    HAL_Delay(1000);

    // --- Test 4.4: Reverse Test (with SHARP Safety Detection) ---
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "[TEST 4] Reverse with Safety Detection\r\n"
             "  Throttle: -25%%\r\n"
             "  Steering: 0deg\r\n"
             "  Duration: Max 3 seconds\r\n"
             "  Note: Will auto-stop if obstacle < %.1f cm\r\n"
             "========================================\r\n",
             VEHICLE_REAR_SAFE_DISTANCE_CM);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    Vehicle_SetTargetSteering(&vehicle, 0.0f);
    Vehicle_SetTargetThrottle(&vehicle, -25.0f);  // Reverse (will auto-execute 2-step sequence)

    // Note: Vehicle_ControlLoop() will auto-detect forward→reverse transition
    // and execute ESC_ReverseSequence()

    for (int i = 0; i < 150; i++) {  // Max 3s @ 50Hz
        Vehicle_UpdateSensors(&vehicle);

        if (Vehicle_ControlLoop(&vehicle) != HAL_OK) {
            // Safety check failed (rear obstacle or other)
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "\r\n[INFO] Safety check triggered, stopped at cycle %d (%.1f seconds)\r\n",
                     i, i * 0.02f);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
            break;
        }

        if (i % 25 == 0) {
            Vehicle_PrintStatus(&vehicle);
        }

        HAL_Delay(20);
    }

    // === Step 5: Test Complete ===
    Vehicle_EmergencyStop(&vehicle);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "  Phase 1 Test Complete!\r\n"
             "  Total Control Cycles: %lu\r\n"
             "========================================\r\n",
             vehicle.control_loop_counter);
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Enter idle state
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INFO] Test complete. Entering idle state...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    while (1) {
        HAL_Delay(1000);
    }
}

#endif // VEHICLE_TEST_ENABLE

#endif // VEHICLE_ENABLE
