/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : spi_comm.h
  * @brief          : SPI Communication Protocol Header (Phase 2)
  * @author         : CoVAPSy Team
  * @date           : 2025-01-14
  ******************************************************************************
  * @attention
  *
  * Phase 2 SPI Communication System - STM32L432KC Slave Mode
  *
  * Features:
  *   - Fixed 32-byte frame structure with CRC-16/MODBUS
  *   - Command set: GET_STATUS, SET_CONTROL, GET_SENSORS, SET_MODE, etc.
  *   - Dual buffer mechanism (20Hz SPI comm vs 50Hz control loop)
  *   - UART simulation mode for testing without Raspberry Pi
  *   - Seamless hardware switchover via SPI_COMM_USE_HARDWARE flag
  *   - Integration with Phase 1 vehicle_control system
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef SPI_COMM_H
#define SPI_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

#if SPI_COMM_ENABLE

#include "stm32l4xx_hal.h"
#include "vehicle_state.h"
#include <stdint.h>
#include <string.h>

/* ============================================ */
/* Protocol Constants                           */
/* ============================================ */

// Frame structure constants
#define SPI_FRAME_SIZE              32      // Total frame size (bytes)
#define SPI_FRAME_HEADER            0xAA    // Frame header magic byte
#define SPI_FRAME_FOOTER            0x55    // Frame footer magic byte
#define SPI_PAYLOAD_MAX_SIZE        26      // Maximum payload size (bytes)

// Command codes (Master → Slave: 0x01-0x7F)
#define SPI_CMD_GET_STATUS          0x01    // Get vehicle status
#define SPI_CMD_SET_CONTROL         0x02    // Set steering and throttle
#define SPI_CMD_GET_SENSORS         0x03    // Get sensor data
#define SPI_CMD_SET_MODE            0x04    // Set vehicle operation mode
#define SPI_CMD_EMERGENCY_STOP      0x05    // Emergency stop
#define SPI_CMD_HEARTBEAT           0x10    // Heartbeat (watchdog reset)

// Response codes (Slave → Master: 0x80-0xFF)
#define SPI_RESP_ACK_OK             0x80    // Command acknowledged successfully
#define SPI_RESP_ACK_ERROR          0x81    // Command failed
#define SPI_RESP_DATA               0x82    // Data response

// Error codes
#define SPI_ERROR_NONE              0x00    // No error
#define SPI_ERROR_CRC               0x01    // CRC check failed
#define SPI_ERROR_HEADER            0x02    // Invalid header
#define SPI_ERROR_FOOTER            0x03    // Invalid footer
#define SPI_ERROR_LENGTH            0x04    // Invalid payload length
#define SPI_ERROR_UNKNOWN_CMD       0x05    // Unknown command code
#define SPI_ERROR_TIMEOUT           0x06    // Communication timeout

// Communication timing
#define SPI_COMM_FREQUENCY_HZ       20      // SPI communication frequency (20Hz)
#define SPI_COMM_PERIOD_MS          50      // Period = 1000ms / 20Hz
#define SPI_COMM_TIMEOUT_MS         500     // Watchdog timeout (same as vehicle)

/* ============================================ */
/* Data Structure Definitions                   */
/* ============================================ */

/**
 * @brief SPI frame structure (32 bytes fixed)
 *
 * Frame layout:
 * [0]    Header (0xAA)
 * [1]    Command code
 * [2]    Payload length (0-26)
 * [3-28] Payload data (26 bytes max)
 * [29-30] CRC16 (Big-Endian)
 * [31]   Footer (0x55)
 */
typedef struct __attribute__((packed)) {
    uint8_t  header;                        // [0] Frame header (0xAA)
    uint8_t  command;                       // [1] Command code
    uint8_t  length;                        // [2] Payload length (0-26)
    uint8_t  payload[SPI_PAYLOAD_MAX_SIZE]; // [3-28] Payload data
    uint8_t  crc16[2];                      // [29-30] CRC-16/MODBUS (Big-Endian: [0]=MSB, [1]=LSB)
    uint8_t  footer;                        // [31] Frame footer (0x55)
} SPI_Frame;

/**
 * @brief Payload structure for SET_CONTROL command (8 bytes)
 */
typedef struct __attribute__((packed)) {
    float steering_deg;     // Target steering angle [-30, +30] deg (4 bytes, Big-Endian)
    float throttle_percent; // Target throttle [-100, +100] % (4 bytes, Big-Endian)
} Payload_SetControl;

/**
 * @brief Payload structure for SET_MODE command (1 byte)
 */
typedef struct __attribute__((packed)) {
    uint8_t mode;           // Vehicle_Mode enum value
} Payload_SetMode;

/**
 * @brief Payload structure for GET_STATUS response (variable length)
 */
typedef struct __attribute__((packed)) {
    uint8_t  mode;                  // Current vehicle mode
    float    current_steering_deg;  // Current steering angle (deg)
    float    current_throttle_percent; // Current throttle (%)
    uint8_t  is_reversing;          // Reversing flag
    uint8_t  safety_stop_triggered; // Safety stop flag
    uint32_t control_loop_counter;  // Control loop counter
} Payload_GetStatus;

/**
 * @brief Payload structure for GET_SENSORS response (variable length)
 */
typedef struct __attribute__((packed)) {
    // SHARP sensors
    float    sharp_left_distance_cm;    // Rear-left distance (cm)
    float    sharp_right_distance_cm;   // Rear-right distance (cm)
    uint8_t  sharp_left_valid;          // Left sensor validity
    uint8_t  sharp_right_valid;         // Right sensor validity

    // IMU data
    float    roll_deg;                  // Roll angle (deg)
    float    pitch_deg;                 // Pitch angle (deg)
    float    yaw_deg;                   // Yaw angle (deg)
    uint8_t  imu_valid;                 // IMU validity
} Payload_GetSensors;

/**
 * @brief Payload structure for ACK_ERROR response (1 byte)
 */
typedef struct __attribute__((packed)) {
    uint8_t error_code;     // Error code (SPI_ERROR_xxx)
} Payload_AckError;

/**
 * @brief SPI communication state management
 */
typedef struct {
    /* === Dual Buffer (solve 20Hz comm vs 50Hz control beat frequency) === */
    SPI_Frame rx_buffer_primary;        // Primary RX buffer (SPI DMA writes here)
    SPI_Frame rx_buffer_secondary;      // Secondary RX buffer (control loop reads here)
    SPI_Frame tx_buffer;                // TX buffer for responses
    volatile uint8_t buffer_swap_flag;  // Buffer swap flag (1=new data available)

    /* === Communication Statistics === */
    uint32_t frame_received_count;      // Total frames received
    uint32_t crc_error_count;           // CRC error count
    uint32_t frame_error_count;         // Frame format error count
    uint32_t valid_command_count;       // Valid command count

    /* === Control Data Cache (extracted from latest valid frame) === */
    float target_steering_deg;          // Target steering [-30, +30] deg
    float target_throttle_percent;      // Target throttle [-100, +100] %
    Vehicle_Mode target_mode;           // Target vehicle mode

    /* === Watchdog Timestamp === */
    uint32_t last_valid_command_time;   // Last valid command timestamp (ms)

    /* === Hardware Handles (dependency injection) === */
    SPI_HandleTypeDef *hspi;            // SPI3 handle (for real hardware mode)
    UART_HandleTypeDef *huart_debug;    // UART2 handle (for debug output)

    /* === Vehicle State Reference (for GET_STATUS/GET_SENSORS) === */
    Vehicle_State *vehicle;             // Pointer to vehicle state (can be NULL in test mode)

} SPI_Comm_State;

/* ============================================ */
/* Global Variables                             */
/* ============================================ */

/**
 * @brief Global SPI communication state pointer
 *
 * This pointer is set during SPI_Comm_Init() and used by:
 * - HAL_SPI_TxRxCpltCallback() in interrupt context
 * - Vehicle_ControlLoop() in main context via SPI_Comm_UpdateVehicleControl()
 *
 * @note Must be initialized before use. Check for NULL before dereferencing.
 */
extern SPI_Comm_State *g_spi_comm_ptr;

/* ============================================ */
/* Public Function Prototypes                   */
/* ============================================ */

/**
 * @brief Initialize SPI communication module
 *
 * This function initializes the SPI communication state and hardware.
 * In UART simulation mode (SPI_COMM_USE_HARDWARE=0), it only initializes
 * the state structure. In real hardware mode, it also starts SPI DMA.
 *
 * @param comm Pointer to SPI_Comm_State structure
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note Before calling this function, set hardware handles:
 *       - comm->hspi = &hspi3;
 *       - comm->huart_debug = &huart2;
 */
HAL_StatusTypeDef SPI_Comm_Init(SPI_Comm_State *comm);

/**
 * @brief Validate SPI frame (Header/Footer/CRC check)
 *
 * Performs four-layer validation:
 * 1. Header check (must be 0xAA)
 * 2. Footer check (must be 0x55)
 * 3. Length check (must be <= 26)
 * 4. CRC-16/MODBUS check (must match calculated CRC)
 *
 * @param frame Pointer to SPI_Frame to validate
 * @return 1 if valid | 0 if invalid
 *
 * @note CRC is calculated over fixed 29 bytes (Header + Command + Length + Payload)
 *       regardless of the Length field value. This protects against Length field
 *       corruption and ensures Padding area is also verified.
 */
uint8_t SPI_Comm_ValidateFrame(const SPI_Frame *frame);

/**
 * @brief Process received SPI frame (parse and execute command)
 *
 * This function dispatches the command to appropriate handler functions.
 * It updates the communication statistics and control data cache.
 *
 * @param comm Pointer to SPI_Comm_State structure
 * @param rx_frame Pointer to received frame
 * @param tx_frame Pointer to response frame (can be NULL if no response needed)
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef SPI_Comm_ProcessFrame(SPI_Comm_State *comm,
                                        const SPI_Frame *rx_frame,
                                        SPI_Frame *tx_frame);

/**
 * @brief Build response frame
 *
 * @param tx_frame Pointer to response frame buffer
 * @param response_code Response code (SPI_RESP_xxx)
 * @param payload Pointer to payload data (can be NULL if length=0)
 * @param payload_length Payload length in bytes (0-26)
 * @return HAL_OK on success | HAL_ERROR on failure
 */
HAL_StatusTypeDef SPI_Comm_BuildResponse(SPI_Frame *tx_frame,
                                         uint8_t response_code,
                                         const void *payload,
                                         uint8_t payload_length);

/**
 * @brief Calculate CRC-16/MODBUS checksum
 *
 * Polynomial: 0x8005 (reversed: 0xA001)
 * Initial value: 0xFFFF
 *
 * @param data Pointer to data buffer
 * @param length Data length in bytes
 * @return CRC-16 value (16-bit)
 *
 * @note For SPI_Frame validation and generation, always use length=29
 *       (Header + Command + Length + Payload area, regardless of actual payload size)
 *
 * @note Test vector:
 *       Input:  {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A}
 *       Output: 0xC5CD
 */
uint16_t SPI_Comm_CalculateCRC16(const uint8_t *data, uint16_t length);

/**
 * @brief Update vehicle control from SPI communication data (50Hz call in control loop)
 *
 * This function is called in the 50Hz control loop. It checks for new SPI data
 * and applies control values to the vehicle. Uses dual buffer mechanism to
 * avoid race conditions with SPI interrupt.
 *
 * @param comm Pointer to SPI_Comm_State structure
 * @param vehicle Pointer to Vehicle_State structure
 * @return HAL_OK on success | HAL_ERROR on failure
 *
 * @note This function should be called at the beginning of Vehicle_ControlLoop()
 *       when vehicle->mode == VEHICLE_MODE_REMOTE
 */
HAL_StatusTypeDef SPI_Comm_UpdateVehicleControl(SPI_Comm_State *comm,
                                                 Vehicle_State *vehicle);

/**
 * @brief Check if SPI communication has timed out
 *
 * @param comm Pointer to SPI_Comm_State structure
 * @return 1 if timeout | 0 if OK
 *
 * @note Timeout threshold: SPI_COMM_TIMEOUT_MS (500ms, same as vehicle watchdog)
 */
uint8_t SPI_Comm_IsTimeout(const SPI_Comm_State *comm);

/**
 * @brief Print SPI communication statistics to UART (debug)
 *
 * @param comm Pointer to SPI_Comm_State structure
 */
void SPI_Comm_PrintStats(const SPI_Comm_State *comm);

/* ============================================ */
/* Command Handler Function Prototypes         */
/* ============================================ */

/**
 * @brief Handle GET_STATUS command
 */
HAL_StatusTypeDef HandleCmd_GetStatus(SPI_Comm_State *comm,
                                      const SPI_Frame *rx_frame,
                                      SPI_Frame *tx_frame);

/**
 * @brief Handle SET_CONTROL command
 */
HAL_StatusTypeDef HandleCmd_SetControl(SPI_Comm_State *comm,
                                       const SPI_Frame *rx_frame,
                                       SPI_Frame *tx_frame);

/**
 * @brief Handle GET_SENSORS command
 */
HAL_StatusTypeDef HandleCmd_GetSensors(SPI_Comm_State *comm,
                                       const SPI_Frame *rx_frame,
                                       SPI_Frame *tx_frame);

/**
 * @brief Handle SET_MODE command
 */
HAL_StatusTypeDef HandleCmd_SetMode(SPI_Comm_State *comm,
                                    const SPI_Frame *rx_frame,
                                    SPI_Frame *tx_frame);

/**
 * @brief Handle EMERGENCY_STOP command
 */
HAL_StatusTypeDef HandleCmd_EmergencyStop(SPI_Comm_State *comm,
                                          const SPI_Frame *rx_frame,
                                          SPI_Frame *tx_frame);

/**
 * @brief Handle HEARTBEAT command
 */
HAL_StatusTypeDef HandleCmd_Heartbeat(SPI_Comm_State *comm,
                                      const SPI_Frame *rx_frame,
                                      SPI_Frame *tx_frame);

/* ============================================ */
/* Byte Order Conversion Inline Functions      */
/* ============================================ */

/**
 * @brief Convert float to Big-Endian byte array
 *
 * @param value Float value
 * @param buffer Pointer to 4-byte buffer (output)
 */
static inline void Float_ToBigEndian(float value, uint8_t *buffer)
{
    union {
        float f;
        uint32_t u;
    } converter;

    converter.f = value;
    buffer[0] = (converter.u >> 24) & 0xFF;  // MSB first
    buffer[1] = (converter.u >> 16) & 0xFF;
    buffer[2] = (converter.u >> 8) & 0xFF;
    buffer[3] = converter.u & 0xFF;          // LSB last
}

/**
 * @brief Convert Big-Endian byte array to float
 *
 * @param buffer Pointer to 4-byte buffer (Big-Endian)
 * @return Float value
 */
static inline float Float_FromBigEndian(const uint8_t *buffer)
{
    union {
        float f;
        uint32_t u;
    } converter;

    converter.u = ((uint32_t)buffer[0] << 24) |  // MSB first
                  ((uint32_t)buffer[1] << 16) |
                  ((uint32_t)buffer[2] << 8)  |
                  ((uint32_t)buffer[3]);         // LSB last

    return converter.f;
}

/**
 * @brief Convert uint16_t to Big-Endian byte array
 *
 * @param value 16-bit value
 * @param buffer Pointer to 2-byte buffer (output)
 */
static inline void Uint16_ToBigEndian(uint16_t value, uint8_t *buffer)
{
    buffer[0] = (value >> 8) & 0xFF;  // MSB first
    buffer[1] = value & 0xFF;         // LSB last
}

/**
 * @brief Convert Big-Endian byte array to uint16_t
 *
 * @param buffer Pointer to 2-byte buffer (Big-Endian)
 * @return 16-bit value
 */
static inline uint16_t Uint16_FromBigEndian(const uint8_t *buffer)
{
    return ((uint16_t)buffer[0] << 8) | buffer[1];
}

/**
 * @brief Convert uint32_t to Big-Endian byte array
 *
 * @param value 32-bit value
 * @param buffer Pointer to 4-byte buffer (output)
 */
static inline void Uint32_ToBigEndian(uint32_t value, uint8_t *buffer)
{
    buffer[0] = (value >> 24) & 0xFF;  // MSB first
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;          // LSB last
}

/**
 * @brief Convert Big-Endian byte array to uint32_t
 *
 * @param buffer Pointer to 4-byte buffer (Big-Endian)
 * @return 32-bit value
 */
static inline uint32_t Uint32_FromBigEndian(const uint8_t *buffer)
{
    return ((uint32_t)buffer[0] << 24) |
           ((uint32_t)buffer[1] << 16) |
           ((uint32_t)buffer[2] << 8)  |
           ((uint32_t)buffer[3]);
}

/* ============================================ */
/* UART Test Interface (for testing without Pi)*/
/* ============================================ */

#if SPI_COMM_TEST_ENABLE

/**
 * @brief Parse UART test command and generate SPI frame
 *
 * Supported command formats:
 *   "SET_CONTROL 15.0 30.0"   -> Steering 15°, Throttle 30%
 *   "SET_MODE REMOTE"         -> Switch to REMOTE mode
 *   "GET_STATUS"              -> Get status
 *   "GET_SENSORS"             -> Get sensors
 *   "EMERGENCY_STOP"          -> Emergency stop
 *   "HEARTBEAT"               -> Heartbeat
 *
 * @param uart_buffer UART input string (null-terminated)
 * @param frame Pointer to SPI_Frame to generate
 * @return HAL_OK if parsed successfully | HAL_ERROR if invalid format
 */
HAL_StatusTypeDef UART_ParseTestCommand(const char *uart_buffer, SPI_Frame *frame);

/**
 * @brief SPI communication test program (UART simulation mode)
 *
 * This function runs an interactive test loop via UART:
 * 1. User inputs test command via UART (e.g., "SET_CONTROL 15.0 30.0")
 * 2. Parse command and generate SPI frame
 * 3. Process frame (validate CRC, execute command)
 * 4. Print response and statistics to UART
 *
 * @param hspi SPI handle (unused in UART mode, pass &hspi3)
 * @param huart UART handle for debug output (&huart2)
 *
 * @note This function contains an infinite loop and never returns!
 */
void SPI_Comm_Test(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

#endif /* SPI_COMM_TEST_ENABLE */

/* ============================================ */
/* SPI + Vehicle Integration Test               */
/* ============================================ */

#if SPI_VEHICLE_TEST_ENABLE

/**
 * @brief SPI + Vehicle integration test (UART commands control real vehicle)
 *
 * This function combines SPI communication testing with real vehicle control:
 * 1. Initialize vehicle (Servo, ESC, IMU, SHARP sensors)
 * 2. Arm ESC (2-second neutral pulse)
 * 3. Enter interactive UART command loop
 * 4. UART commands → SPI frame → Vehicle control (real servo/ESC movement)
 * 5. Run 50Hz control loop in background
 *
 * Supported commands:
 *   SET_CONTROL <steering> <throttle>  - Control vehicle (e.g., "SET_CONTROL 15.0 30.0")
 *   SET_MODE <mode>                    - Change mode (IDLE/REMOTE/MANUAL/EMERGENCY)
 *   GET_STATUS                         - Get vehicle status
 *   GET_SENSORS                        - Get sensor data
 *   EMERGENCY_STOP                     - Emergency stop
 *   HEARTBEAT                          - Reset watchdog
 *   STOP                               - Set throttle to 0 (shortcut)
 *   HELP                               - Show command help
 *
 * @param htim Timer handle for servo/ESC (&htim1)
 * @param hi2c I2C handle for IMU (&hi2c1)
 * @param hadc ADC handle for SHARP sensors (&hadc1)
 * @param huart UART handle for debug/command input (&huart2)
 *
 * @note This function contains an infinite loop and never returns!
 * @warning This test controls REAL actuators! Ensure vehicle is safely positioned.
 */
void SPI_Vehicle_IntegrationTest(TIM_HandleTypeDef *htim,
                                  I2C_HandleTypeDef *hi2c,
                                  ADC_HandleTypeDef *hadc,
                                  UART_HandleTypeDef *huart);

#endif /* SPI_VEHICLE_TEST_ENABLE */

/* ============================================ */
/* SPI Hardware Debug Mode (Real Pi + Debug)    */
/* ============================================ */

#if SPI_HARDWARE_DEBUG_ENABLE

/**
 * @brief SPI Hardware Debug Mode (Real SPI + Vehicle + UART debug)
 *
 * This function runs real SPI communication with Raspberry Pi while
 * printing debug information to UART:
 * 1. Initialize vehicle (Servo, ESC, IMU, SHARP sensors)
 * 2. Initialize SPI3 in slave mode with DMA
 * 3. Set vehicle to REMOTE mode
 * 4. Main loop: 50Hz control + debug output
 *    - Receives SPI commands from Raspberry Pi
 *    - Prints received commands to UART for debugging
 *    - Controls vehicle based on SPI commands
 *    - Sends sensor data back to Raspberry Pi
 *
 * @param hspi SPI handle (&hspi3)
 * @param htim Timer handle for servo/ESC (&htim1)
 * @param hi2c I2C handle for IMU (&hi2c1)
 * @param hadc ADC handle for SHARP sensors (&hadc1)
 * @param huart UART handle for debug output (&huart2)
 *
 * @note This function contains an infinite loop and never returns!
 * @warning This test controls REAL actuators! Ensure vehicle is safely positioned.
 */
void SPI_Hardware_Debug(SPI_HandleTypeDef *hspi,
                        TIM_HandleTypeDef *htim,
                        I2C_HandleTypeDef *hi2c,
                        ADC_HandleTypeDef *hadc,
                        UART_HandleTypeDef *huart);

#endif /* SPI_HARDWARE_DEBUG_ENABLE */

/* ============================================ */
/* SPI Production Mode (Real Pi, No Debug)      */
/* ============================================ */

#if SPI_PRODUCTION_ENABLE

/**
 * @brief SPI Production Mode (Real SPI + Vehicle, NO debug output)
 *
 * This function runs real SPI communication with Raspberry Pi without
 * any UART debug output for maximum performance:
 * 1. Initialize vehicle (Servo, ESC, IMU, SHARP sensors)
 * 2. Initialize SPI3 in slave mode with DMA
 * 3. Set vehicle to REMOTE mode
 * 4. Main loop: 50Hz control loop only
 *    - Receives SPI commands from Raspberry Pi
 *    - Controls vehicle based on SPI commands
 *    - Sends sensor data back to Raspberry Pi
 *
 * @param hspi SPI handle (&hspi3)
 * @param htim Timer handle for servo/ESC (&htim1)
 * @param hi2c I2C handle for IMU (&hi2c1)
 * @param hadc ADC handle for SHARP sensors (&hadc1)
 *
 * @note This function contains an infinite loop and never returns!
 * @warning This function controls REAL actuators! Ensure vehicle is safely positioned.
 */
void SPI_Production(SPI_HandleTypeDef *hspi,
                    TIM_HandleTypeDef *htim,
                    I2C_HandleTypeDef *hi2c,
                    ADC_HandleTypeDef *hadc);

#endif /* SPI_PRODUCTION_ENABLE */

#endif /* SPI_COMM_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* SPI_COMM_H */
