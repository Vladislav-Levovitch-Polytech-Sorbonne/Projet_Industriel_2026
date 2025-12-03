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
#define MMA8451_ENABLE          1    // 1=Enable, 0=Disable

/* MMA8451 Test Function */
#define MMA8451_TEST_ENABLE     0    // 1=Enable, 0=Disable

/* SHARP Distance Sensor Driver */
#define SHARP_SENSOR_ENABLE     1    // 1=Enable, 0=Disable

/* SHARP Test Function */
#define SHARP_TEST_ENABLE       1    // 1=Enable, 0=Disable

/* Future sensor modules can be added here */
// #define LIDAR_ENABLE         1
// #define SERVO_ENABLE         1
// #define MOTOR_ENABLE         1

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
