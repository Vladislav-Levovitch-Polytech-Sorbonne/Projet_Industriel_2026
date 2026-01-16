/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : config.h
  * @brief          : Feature configuration flags for CoVAPSy project
  * @author         : CoVAPSy Team
  * @date           : 2025-12-01
  ******************************************************************************
  * @attention
  *
  * This file contains feature flags to enable/disable specific modules.
  * Modify the values (1=Enable, 0=Disable) to control which features are
  * compiled into the firmware.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================ */
/* Feature Configuration Flags                  */
/* ============================================ */

/* MMA8451 Accelerometer Driver */
#define MMA8451_ENABLE          0    // 1=Enable, 0=Disable

/* MMA8451 Test Function */
#define MMA8451_TEST_ENABLE     0    // 1=Enable, 0=Disable

/* SHARP Distance Sensor Driver */
#define SHARP_SENSOR_ENABLE     1    // 1=Enable, 0=Disable

/* SHARP Test Function */
#define SHARP_TEST_ENABLE       0    // 1=Enable, 0=Disable (Dual sensor test)

/* BNO055 IMU Sensor Driver */
#define BNO055_ENABLE           1    // 1=Enable, 0=Disable

/* BNO055 Test Function */
#define BNO055_TEST_ENABLE      0    // 1=Enable, 0=Disable

/* Servo Driver */
#define SERVO_ENABLE            1    // 1=Enable, 0=Disable

/* Servo Test Function */
#define SERVO_TEST_ENABLE       0    // 1=Enable, 0=Disable (set to 0 for vehicle test)

/* ESC (Electronic Speed Controller) Driver */
#define ESC_ENABLE              1    // 1=Enable, 0=Disable

/* ESC Test Function */
#define ESC_TEST_ENABLE         0    // 1=Enable, 0=Disable (set to 0 for vehicle test)

/* ESC Bidirectional Mode (Forward/Brake/Reverse) */
#define ESC_BIDIRECTIONAL_ENABLE 1   // 1=Bidirectional, 0=Forward Only

/* Vehicle Control Layer */
#define VEHICLE_ENABLE          1    // 1=Enable, 0=Disable

/* Vehicle Test Function */
#define VEHICLE_TEST_ENABLE     0    // 1=Enable, 0=Disable

/* SPI Communication Module (Phase 2) */
#define SPI_COMM_ENABLE         1    // 1=Enable, 0=Disable

/* SPI Communication Test Function (protocol-only, no vehicle control) */
#define SPI_COMM_TEST_ENABLE    0    // 1=Enable, 0=Disable (UART simulation mode)

/* SPI Communication Hardware Mode */
#define SPI_COMM_USE_HARDWARE   1    // 0=UART simulation, 1=Real SPI3 hardware

/* SPI + Vehicle Integration Test (UART commands control real vehicle) */
#define SPI_VEHICLE_TEST_ENABLE 0    // 1=Enable, 0=Disable
                                     // Uses UART commands to control real servo/ESC
                                     // Simulates Raspberry Pi SPI commands

/* Vehicle Watchdog Timeout (REMOTE mode only) */
#define VEHICLE_WATCHDOG_ENABLE 0    // 1=Enable, 0=Disable
                                     // When enabled, triggers EMERGENCY_STOP if no command
                                     // received for >500ms in REMOTE mode
                                     // Set to 0 for manual testing (slow UART input)

/* SPI Hardware Debug Mode (Real SPI + Vehicle + UART debug output) */
#define SPI_HARDWARE_DEBUG_ENABLE 1  // 1=Enable, 0=Disable
                                     // Runs real SPI communication with Raspberry Pi
                                     // Prints received commands to UART for debugging

/* SPI Production Mode (Real SPI + Vehicle, NO debug output) */
#define SPI_PRODUCTION_ENABLE     0  // 1=Enable, 0=Disable
                                     // Same as DEBUG mode but without UART output
                                     // Use this for final deployment

/* Future sensor modules can be added here */
// #define LIDAR_ENABLE         1
// #define SERVO_ENABLE         1
// #define MOTOR_ENABLE         1

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
