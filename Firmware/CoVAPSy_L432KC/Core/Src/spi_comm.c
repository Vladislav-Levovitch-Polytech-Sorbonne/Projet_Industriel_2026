/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : spi_comm.c
  * @brief          : SPI Communication Protocol Implementation (Phase 2)
  * @author         : CoVAPSy Team
  * @date           : 2025-01-14
  ******************************************************************************
  * @attention
  *
  * Phase 2 SPI Communication System - STM32L432KC Slave Mode
  * This file implements the complete SPI communication protocol stack.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "spi_comm.h"
#include "main.h"

#if SPI_COMM_ENABLE

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern osThreadId_t SPICommTaskHandle;

/* Global variables ----------------------------------------------------------*/
/* Global SPI communication state pointer
 * - Used by DMA callback (interrupt context) and Vehicle_ControlLoop (main context)
 * - Declared extern in spi_comm.h for access from vehicle_control.c
 */
SPI_Comm_State *g_spi_comm_ptr = NULL;


/* Private function prototypes -----------------------------------------------*/
static void SPI_Comm_SetReady(GPIO_PinState state);
static void SPI_Comm_NotifyTaskFromISR(void);
static HAL_StatusTypeDef SPI_Comm_ArmDMA(SPI_Comm_State *comm);
static HAL_StatusTypeDef SPI_Comm_Recover(SPI_Comm_State *comm,
                                          uint8_t error_code);
static uint8_t SPI_Comm_ValidateFrameDetailed(const SPI_Frame *frame);
static uint8_t SPI_Comm_QueueVehicleCommand(
    SPI_Comm_State *comm,
    const SPI_PendingVehicleCommand *command);

/* ============================================ */
/* SPI DMA Callback Functions (for hardware mode) */
/* ============================================ */

#if SPI_COMM_USE_HARDWARE

/**
 * @brief SPI transmit-receive complete callback (called by HAL)
 *
 * This callback is triggered when a full-duplex SPI frame transfer completes.
 *
 * Key points:
 * - This function runs in INTERRUPT context - keep it fast!
 * - The frame we just received was transmitted WHILE we sent the previous response
 * - The response we prepare here will be sent DURING the next command reception
 *
 * Timeline:
 *   T0: Master sends Frame N, STM32 sends Response (N-1)  [DMA in progress]
 *   T1: Transfer complete, this callback fires
 *   T2: Process Frame N, prepare Response N
 *   T3: Master sends Frame N+1, STM32 sends Response N    [DMA restarts]
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI3 || g_spi_comm_ptr == NULL) {
        return;
    }

    SPI_Comm_State *comm = g_spi_comm_ptr;

    SPI_Comm_SetReady(GPIO_PIN_RESET);
    comm->dma_complete = 1U;
    comm->run_state = SPI_COMM_STATE_PROCESSING;
    SPI_Comm_NotifyTaskFromISR();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI3 || g_spi_comm_ptr == NULL) {
        return;
    }

    SPI_Comm_SetReady(GPIO_PIN_RESET);
    g_spi_comm_ptr->spi_error = 1U;
    g_spi_comm_ptr->dma_error_count++;
    g_spi_comm_ptr->run_state = SPI_COMM_STATE_ERROR;
    SPI_Comm_NotifyTaskFromISR();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI_NSS_Pin) {
        SPI_Comm_HandleNssEdge(
            HAL_GPIO_ReadPin(SPI_NSS_GPIO_Port, SPI_NSS_Pin) == GPIO_PIN_SET);
    }
}

#endif /* SPI_COMM_USE_HARDWARE */

/* ============================================ */
/* Public Function Implementations             */
/* ============================================ */

/**
 * @brief Initialize SPI communication module
 */
HAL_StatusTypeDef SPI_Comm_Init(SPI_Comm_State *comm)
{
    // Input validation
    if (comm == NULL) {
        return HAL_ERROR;
    }

    // Clear all buffers
    memset(&comm->rx_buffer_primary, 0, sizeof(SPI_Frame));
    memset(&comm->rx_buffer_secondary, 0, sizeof(SPI_Frame));
    memset(&comm->tx_buffer, 0, sizeof(SPI_Frame));
    memset(&comm->cached_response, 0, sizeof(SPI_Frame));
    memset(comm->response_cache, 0, sizeof(comm->response_cache));
    memset(comm->response_sequence, 0, sizeof(comm->response_sequence));
    memset(comm->response_valid, 0, sizeof(comm->response_valid));
    memset(&comm->telemetry, 0, sizeof(comm->telemetry));
    memset(comm->pending_queue, 0, sizeof(comm->pending_queue));

    // Initialize flags and counters
    comm->buffer_swap_flag = 0;
    comm->dma_complete = 0U;
    comm->nss_high = 1U;
    comm->short_frame = 0U;
    comm->spi_error = 0U;
    comm->run_state = SPI_COMM_STATE_STOPPED;
    comm->last_sequence = 0U;
    comm->last_sequence_valid = 0U;
    comm->response_cache_next = 0U;
    comm->frame_received_count = 0;
    comm->crc_error_count = 0;
    comm->frame_error_count = 0;
    comm->valid_command_count = 0;
    comm->duplicate_count = 0U;
    comm->short_frame_count = 0U;
    comm->dma_error_count = 0U;
    comm->dma_restart_error_count = 0U;
    comm->pending_head = 0U;
    comm->pending_tail = 0U;
    comm->pending_count = 0U;

    // Initialize control data cache
    comm->target_steering_deg = 0.0f;
    comm->target_throttle_percent = 0.0f;
    comm->target_mode = VEHICLE_MODE_IDLE;

    // Initialize watchdog timestamp
    comm->last_valid_command_time = HAL_GetTick();

    // Initialize vehicle pointer (set to NULL, user should set it later)
    if (comm->vehicle == NULL) {
        // Keep NULL - will return dummy data in GET_STATUS/GET_SENSORS
    }

    // Save global pointer for callback access
    g_spi_comm_ptr = comm;

    /* DMA is intentionally started only after the RTOS task exists. */
    SPI_Comm_BuildResponse(&comm->tx_buffer, SPI_RESP_ACK_OK,
                           SPI_SEQUENCE_BOOT, NULL, 0);
    memcpy(&comm->cached_response, &comm->tx_buffer, sizeof(SPI_Frame));
    SPI_Comm_SetReady(GPIO_PIN_RESET);

    return HAL_OK;
}

HAL_StatusTypeDef SPI_Comm_Start(SPI_Comm_State *comm)
{
#if SPI_COMM_USE_HARDWARE
    return SPI_Comm_ArmDMA(comm);
#else
    (void)comm;
    return HAL_OK;
#endif
}

static void SPI_Comm_SetReady(GPIO_PinState state)
{
    HAL_GPIO_WritePin(SPI_READY_GPIO_Port, SPI_READY_Pin, state);
}

static void SPI_Comm_NotifyTaskFromISR(void)
{
    if (SPICommTaskHandle == NULL) {
        return;
    }

    BaseType_t task_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)SPICommTaskHandle, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

static HAL_StatusTypeDef SPI_Comm_ArmDMA(SPI_Comm_State *comm)
{
    if (comm == NULL || comm->hspi == NULL) {
        return HAL_ERROR;
    }

    memset(&comm->rx_buffer_primary, 0, sizeof(SPI_Frame));
    comm->dma_complete = 0U;
    comm->short_frame = 0U;
    comm->spi_error = 0U;
    comm->nss_high = 1U;

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        comm->hspi,
        (uint8_t *)&comm->tx_buffer,
        (uint8_t *)&comm->rx_buffer_primary,
        sizeof(SPI_Frame));

    if (status != HAL_OK) {
        comm->dma_restart_error_count++;
        comm->run_state = SPI_COMM_STATE_ERROR;
        SPI_Comm_SetReady(GPIO_PIN_RESET);
        return status;
    }

    comm->run_state = SPI_COMM_STATE_ARMED;
    SPI_Comm_SetReady(GPIO_PIN_SET);
    return HAL_OK;
}

void SPI_Comm_HandleNssEdge(uint8_t nss_is_high)
{
    SPI_Comm_State *comm = g_spi_comm_ptr;
    if (comm == NULL) {
        return;
    }

    if (!nss_is_high) {
        comm->nss_high = 0U;
        if (comm->run_state == SPI_COMM_STATE_ARMED) {
            comm->run_state = SPI_COMM_STATE_TRANSFER;
        }
        SPI_Comm_SetReady(GPIO_PIN_RESET);
        return;
    }

    comm->nss_high = 1U;
    if (!comm->dma_complete && comm->run_state == SPI_COMM_STATE_TRANSFER &&
        comm->hspi != NULL && comm->hspi->hdmarx != NULL &&
        __HAL_DMA_GET_COUNTER(comm->hspi->hdmarx) != 0U) {
        comm->short_frame = 1U;
    }
    SPI_Comm_NotifyTaskFromISR();
}

/**
 * @brief Calculate CRC-16/MODBUS checksum
 */
uint16_t SPI_Comm_CalculateCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;  // Initial value

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // Polynomial 0x8005 reversed
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Validate SPI frame (Header/Footer/CRC check)
 */
uint8_t SPI_Comm_ValidateFrame(const SPI_Frame *frame)
{
    return SPI_Comm_ValidateFrameDetailed(frame) == SPI_ERROR_NONE;
}

static uint8_t SPI_Comm_ValidateFrameDetailed(const SPI_Frame *frame)
{
    if (frame == NULL) {
        return SPI_ERROR_INTERNAL;
    }
    if (frame->header != SPI_FRAME_HEADER) {
        return SPI_ERROR_HEADER;
    }
    if (frame->footer != SPI_FRAME_FOOTER) {
        return SPI_ERROR_FOOTER;
    }
    if ((frame->version_flags >> 4) != SPI_PROTOCOL_VERSION) {
        return SPI_ERROR_VERSION;
    }
    if (frame->length > SPI_PAYLOAD_MAX_SIZE) {
        return SPI_ERROR_LENGTH;
    }

    uint16_t calculated_crc = SPI_Comm_CalculateCRC16(
        (const uint8_t *)frame, 29U);
    uint16_t frame_crc = Uint16_FromBigEndian(frame->crc16);
    return calculated_crc == frame_crc ? SPI_ERROR_NONE : SPI_ERROR_CRC;
}

/**
 * @brief Build response frame
 */
HAL_StatusTypeDef SPI_Comm_BuildResponse(SPI_Frame *tx_frame,
                                         uint8_t response_code,
                                         uint16_t sequence,
                                         const void *payload,
                                         uint8_t payload_length)
{
    // Input validation
    if (tx_frame == NULL || payload_length > SPI_PAYLOAD_MAX_SIZE) {
        return HAL_ERROR;
    }

    // Clear entire frame buffer (important: zeroes the Padding area)
    memset(tx_frame, 0, sizeof(SPI_Frame));

    // Build frame structure
    tx_frame->header = SPI_FRAME_HEADER;
    tx_frame->version_flags = SPI_VERSION_FLAGS;
    Uint16_ToBigEndian(sequence, tx_frame->sequence);
    tx_frame->command = response_code;
    tx_frame->length = payload_length;

    // Copy payload if provided (remaining bytes stay zero - Padding)
    if (payload != NULL && payload_length > 0) {
        memcpy(tx_frame->payload, payload, payload_length);
    }

    // Calculate CRC over fixed 29 bytes (Header + Command + Length + Payload)
    // Note: Always 29 bytes regardless of payload_length
    // This ensures Padding area (zeroed) is also protected by CRC
    uint16_t crc = SPI_Comm_CalculateCRC16((const uint8_t*)tx_frame, 29);

    // Store CRC in Big-Endian format
    Uint16_ToBigEndian(crc, tx_frame->crc16);

    // Set footer
    tx_frame->footer = SPI_FRAME_FOOTER;

    return HAL_OK;
}

HAL_StatusTypeDef SPI_Comm_Service(SPI_Comm_State *comm)
{
    if (comm == NULL || comm->hspi == NULL) {
        return HAL_ERROR;
    }

    if (comm->spi_error) {
        return SPI_Comm_Recover(comm, SPI_ERROR_INTERNAL);
    }

    if (comm->short_frame) {
        comm->short_frame_count++;
        return SPI_Comm_Recover(comm, SPI_ERROR_SHORT_FRAME);
    }

    /* DMA completion may precede the NSS rising edge by a few microseconds. */
    if (!comm->dma_complete || !comm->nss_high) {
        return HAL_BUSY;
    }

    memcpy(&comm->rx_buffer_secondary,
           &comm->rx_buffer_primary,
           sizeof(SPI_Frame));
    comm->dma_complete = 0U;

    SPI_Frame *rx = &comm->rx_buffer_secondary;
    uint16_t sequence = Uint16_FromBigEndian(rx->sequence);
    uint8_t validation = SPI_Comm_ValidateFrameDetailed(rx);

    if (validation != SPI_ERROR_NONE) {
        Payload_AckError error_payload = { .error_code = validation };
        comm->frame_error_count++;
        if (validation == SPI_ERROR_CRC) {
            comm->crc_error_count++;
        }
        SPI_Comm_BuildResponse(&comm->tx_buffer, SPI_RESP_ACK_ERROR,
                               sequence, &error_payload,
                               sizeof(error_payload));
        return SPI_Comm_ArmDMA(comm);
    }

    comm->frame_received_count++;

    for (uint8_t i = 0U; i < SPI_RESPONSE_CACHE_SIZE; i++) {
        if (comm->response_valid[i] &&
            comm->response_sequence[i] == sequence) {
            memcpy(&comm->tx_buffer,
                   &comm->response_cache[i],
                   sizeof(SPI_Frame));
            comm->duplicate_count++;
            return SPI_Comm_ArmDMA(comm);
        }
    }

    (void)SPI_Comm_ProcessFrame(comm, rx, &comm->tx_buffer);
    comm->last_sequence = sequence;
    comm->last_sequence_valid = 1U;
    memcpy(&comm->cached_response, &comm->tx_buffer, sizeof(SPI_Frame));
    uint8_t cache_index = comm->response_cache_next;
    memcpy(&comm->response_cache[cache_index],
           &comm->tx_buffer,
           sizeof(SPI_Frame));
    comm->response_sequence[cache_index] = sequence;
    comm->response_valid[cache_index] = 1U;
    comm->response_cache_next = (uint8_t)((cache_index + 1U) %
                                          SPI_RESPONSE_CACHE_SIZE);

    return SPI_Comm_ArmDMA(comm);
}

static HAL_StatusTypeDef SPI_Comm_Recover(SPI_Comm_State *comm,
                                          uint8_t error_code)
{
    if (comm == NULL || comm->hspi == NULL) {
        return HAL_ERROR;
    }

    SPI_Comm_SetReady(GPIO_PIN_RESET);
    (void)HAL_SPI_Abort(comm->hspi);
    __HAL_SPI_CLEAR_OVRFLAG(comm->hspi);

    comm->dma_complete = 0U;
    comm->short_frame = 0U;
    comm->spi_error = 0U;
    comm->run_state = SPI_COMM_STATE_STOPPED;

    Payload_AckError error_payload = { .error_code = error_code };
    SPI_Comm_BuildResponse(&comm->tx_buffer, SPI_RESP_ACK_ERROR,
                           SPI_SEQUENCE_BOOT, &error_payload,
                           sizeof(error_payload));
    return SPI_Comm_ArmDMA(comm);
}

/**
 * @brief Process received SPI frame (parse and execute command)
 */
HAL_StatusTypeDef SPI_Comm_ProcessFrame(SPI_Comm_State *comm,
                                        const SPI_Frame *rx_frame,
                                        SPI_Frame *tx_frame)
{
    // Input validation
    if (comm == NULL || rx_frame == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;

    // Dispatch command to appropriate handler
    switch (rx_frame->command) {
        case SPI_CMD_GET_STATUS:
            status = HandleCmd_GetStatus(comm, rx_frame, tx_frame);
            break;

        case SPI_CMD_SET_CONTROL:
            status = HandleCmd_SetControl(comm, rx_frame, tx_frame);
            break;

        case SPI_CMD_GET_SENSORS:
            status = HandleCmd_GetSensors(comm, rx_frame, tx_frame);
            break;

        case SPI_CMD_SET_MODE:
            status = HandleCmd_SetMode(comm, rx_frame, tx_frame);
            break;

        case SPI_CMD_EMERGENCY_STOP:
            status = HandleCmd_EmergencyStop(comm, rx_frame, tx_frame);
            break;

        case SPI_CMD_HEARTBEAT:
            status = HandleCmd_Heartbeat(comm, rx_frame, tx_frame);
            break;

        default:
            // Unknown command - send error response
            if (tx_frame != NULL) {
                Payload_AckError error_payload;
                error_payload.error_code = SPI_ERROR_UNKNOWN_CMD;
                SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_ERROR,
                                      Uint16_FromBigEndian(rx_frame->sequence),
                                      &error_payload, sizeof(error_payload));
            }
            status = HAL_ERROR;
            break;
    }

    // Update statistics
    if (status == HAL_OK) {
        comm->valid_command_count++;
        comm->last_valid_command_time = HAL_GetTick();
    }

    return status;
}

/**
 * @brief Update vehicle control from SPI communication data
 */
HAL_StatusTypeDef SPI_Comm_UpdateVehicleControl(SPI_Comm_State *comm,
                                                 Vehicle_State *vehicle)
{
    if (comm == NULL || vehicle == NULL) {
        return HAL_ERROR;
    }

    for (;;) {
        SPI_PendingVehicleCommand pending;

        taskENTER_CRITICAL();
        if (comm->pending_count == 0U) {
            taskEXIT_CRITICAL();
            break;
        }
        pending = comm->pending_queue[comm->pending_tail];
        comm->pending_tail = (uint8_t)((comm->pending_tail + 1U) %
                                       SPI_PENDING_QUEUE_SIZE);
        comm->pending_count--;
        taskEXIT_CRITICAL();

        switch (pending.command) {
            case SPI_CMD_SET_CONTROL:
                (void)Vehicle_SetTargetSteering(vehicle,
                                                pending.steering_deg);
                (void)Vehicle_SetTargetThrottle(vehicle,
                                                pending.throttle_percent);
                break;
            case SPI_CMD_SET_MODE:
                (void)Vehicle_SetMode(vehicle, pending.mode);
                break;
            case SPI_CMD_EMERGENCY_STOP:
                (void)Vehicle_EmergencyStop(vehicle);
                break;
            default:
                break;
        }
    }

    return HAL_OK;
}

static uint8_t SPI_Comm_QueueVehicleCommand(
    SPI_Comm_State *comm,
    const SPI_PendingVehicleCommand *command)
{
    if (comm == NULL || command == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();

    /* Emergency stop must never be rejected because normal commands queued. */
    if (command->command == SPI_CMD_EMERGENCY_STOP) {
        comm->pending_head = 0U;
        comm->pending_tail = 0U;
        comm->pending_count = 0U;
    } else if (comm->pending_count >= SPI_PENDING_QUEUE_SIZE) {
        taskEXIT_CRITICAL();
        return 0U;
    }

    comm->pending_queue[comm->pending_head] = *command;
    comm->pending_head = (uint8_t)((comm->pending_head + 1U) %
                                   SPI_PENDING_QUEUE_SIZE);
    comm->pending_count++;
    taskEXIT_CRITICAL();
    return 1U;
}

void SPI_Comm_PublishTelemetry(SPI_Comm_State *comm,
                               const Vehicle_State *vehicle)
{
    if (comm == NULL || vehicle == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    comm->telemetry.mode = vehicle->mode;
    comm->telemetry.current_steering_deg = vehicle->current_steering_deg;
    comm->telemetry.current_throttle_percent = vehicle->current_throttle_percent;
    comm->telemetry.is_reversing = vehicle->is_reversing;
    comm->telemetry.safety_stop_triggered = vehicle->safety_stop_triggered;
    comm->telemetry.control_loop_counter = vehicle->control_loop_counter;
    comm->telemetry.sharp_left_distance_cm = vehicle->sharp_left_distance_cm;
    comm->telemetry.sharp_right_distance_cm = vehicle->sharp_right_distance_cm;
    comm->telemetry.sharp_left_valid = vehicle->sharp_left_valid;
    comm->telemetry.sharp_right_valid = vehicle->sharp_right_valid;
    comm->telemetry.roll_deg = vehicle->roll_deg;
    comm->telemetry.pitch_deg = vehicle->pitch_deg;
    comm->telemetry.yaw_deg = vehicle->yaw_deg;
    comm->telemetry.imu_valid = vehicle->imu_valid;
    taskEXIT_CRITICAL();
}

/**
 * @brief Check if SPI communication has timed out
 */
uint8_t SPI_Comm_IsTimeout(const SPI_Comm_State *comm)
{
    if (comm == NULL) {
        return 1;  // Treat NULL as timeout
    }

    uint32_t time_since_last_cmd = HAL_GetTick() - comm->last_valid_command_time;
    return (time_since_last_cmd > SPI_COMM_TIMEOUT_MS) ? 1 : 0;
}

/**
 * @brief Print SPI communication statistics to UART
 */
void SPI_Comm_PrintStats(const SPI_Comm_State *comm)
{
    if (comm == NULL || comm->huart_debug == NULL) {
        return;
    }

    char buffer[240];
    snprintf(buffer, sizeof(buffer),
             "[SPI] RX=%lu Valid=%lu Dup=%lu CRC=%lu Frame=%lu Short=%lu DMA=%lu Restart=%lu\r\n",
             comm->frame_received_count,
             comm->valid_command_count,
             comm->duplicate_count,
             comm->crc_error_count,
             comm->frame_error_count,
             comm->short_frame_count,
             comm->dma_error_count,
             comm->dma_restart_error_count);

    HAL_UART_Transmit(comm->huart_debug, (uint8_t*)buffer, strlen(buffer), 100);
}

/* ============================================ */
/* Command Handler Implementations             */
/* ============================================ */

/**
 * @brief Handle GET_STATUS command
 */
HAL_StatusTypeDef HandleCmd_GetStatus(SPI_Comm_State *comm,
                                      const SPI_Frame *rx_frame,
                                      SPI_Frame *tx_frame)
{
    if (comm == NULL || rx_frame == NULL) {
        return HAL_ERROR;
    }

    if (tx_frame == NULL) {
        return HAL_OK;  // No response needed
    }

    // Build response payload
    uint8_t payload[20];  // Enough for Payload_GetStatus
    uint8_t payload_len = 0;

    SPI_TelemetrySnapshot snapshot;
    taskENTER_CRITICAL();
    snapshot = comm->telemetry;
    taskEXIT_CRITICAL();

    payload[payload_len++] = (uint8_t)snapshot.mode;
    Float_ToBigEndian(snapshot.current_steering_deg, &payload[payload_len]);
    payload_len += 4;
    Float_ToBigEndian(snapshot.current_throttle_percent, &payload[payload_len]);
    payload_len += 4;
    payload[payload_len++] = snapshot.is_reversing ? 1U : 0U;
    payload[payload_len++] = snapshot.safety_stop_triggered ? 1U : 0U;
    Uint32_ToBigEndian(snapshot.control_loop_counter, &payload[payload_len]);
    payload_len += 4;

    // Build response frame
    SPI_Comm_BuildResponse(tx_frame, SPI_RESP_DATA,
                           Uint16_FromBigEndian(rx_frame->sequence),
                           payload, payload_len);

    return HAL_OK;
}

/**
 * @brief Handle SET_CONTROL command
 */
HAL_StatusTypeDef HandleCmd_SetControl(SPI_Comm_State *comm,
                                       const SPI_Frame *rx_frame,
                                       SPI_Frame *tx_frame)
{
    // Input validation
    if (comm == NULL || rx_frame == NULL) {
        return HAL_ERROR;
    }

    // Check payload length
    if (rx_frame->length != sizeof(Payload_SetControl)) {
        if (tx_frame != NULL) {
            Payload_AckError error_payload;
            error_payload.error_code = SPI_ERROR_LENGTH;
            SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_ERROR,
                                  Uint16_FromBigEndian(rx_frame->sequence),
                                  &error_payload, sizeof(error_payload));
        }
        return HAL_ERROR;
    }

    // Extract payload (Big-Endian conversion)
    float steering = Float_FromBigEndian(&rx_frame->payload[0]);
    float throttle = Float_FromBigEndian(&rx_frame->payload[4]);

    if (!isfinite(steering) || !isfinite(throttle) ||
        steering < -30.0f || steering > 30.0f ||
        throttle < -100.0f || throttle > 100.0f) {
        if (tx_frame != NULL) {
            Payload_AckError error_payload = {
                .error_code = SPI_ERROR_INVALID_VALUE
            };
            SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_ERROR,
                                   Uint16_FromBigEndian(rx_frame->sequence),
                                   &error_payload, sizeof(error_payload));
        }
        return HAL_ERROR;
    }

    SPI_PendingVehicleCommand pending = {
        .command = SPI_CMD_SET_CONTROL,
        .steering_deg = steering,
        .throttle_percent = throttle,
        .mode = VEHICLE_MODE_IDLE
    };
    if (!SPI_Comm_QueueVehicleCommand(comm, &pending)) {
        if (tx_frame != NULL) {
            Payload_AckError error_payload = { .error_code = SPI_ERROR_BUSY };
            SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_ERROR,
                                   Uint16_FromBigEndian(rx_frame->sequence),
                                   &error_payload, sizeof(error_payload));
        }
        return HAL_BUSY;
    }

    // Update control cache
    comm->target_steering_deg = steering;
    comm->target_throttle_percent = throttle;

    // Send ACK_OK response
    if (tx_frame != NULL) {
        SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_OK,
                               Uint16_FromBigEndian(rx_frame->sequence),
                               NULL, 0);
    }

    return HAL_OK;
}

/**
 * @brief Handle GET_SENSORS command
 */
HAL_StatusTypeDef HandleCmd_GetSensors(SPI_Comm_State *comm,
                                       const SPI_Frame *rx_frame,
                                       SPI_Frame *tx_frame)
{
    if (comm == NULL || rx_frame == NULL) {
        return HAL_ERROR;
    }

    if (tx_frame == NULL) {
        return HAL_OK;  // No response needed
    }

    // Build response payload (matches Payload_GetSensors structure)
    uint8_t payload[SPI_PAYLOAD_MAX_SIZE];
    uint8_t payload_len = 0;

    SPI_TelemetrySnapshot snapshot;
    taskENTER_CRITICAL();
    snapshot = comm->telemetry;
    taskEXIT_CRITICAL();

    Float_ToBigEndian(snapshot.sharp_left_distance_cm, &payload[payload_len]);
    payload_len += 4;
    Float_ToBigEndian(snapshot.sharp_right_distance_cm, &payload[payload_len]);
    payload_len += 4;
    payload[payload_len++] = snapshot.sharp_left_valid ? 1U : 0U;
    payload[payload_len++] = snapshot.sharp_right_valid ? 1U : 0U;
    Float_ToBigEndian(snapshot.roll_deg, &payload[payload_len]);
    payload_len += 4;
    Float_ToBigEndian(snapshot.pitch_deg, &payload[payload_len]);
    payload_len += 4;
    Float_ToBigEndian(snapshot.yaw_deg, &payload[payload_len]);
    payload_len += 4;
    payload[payload_len++] = snapshot.imu_valid ? 1U : 0U;

    // Build response frame
    SPI_Comm_BuildResponse(tx_frame, SPI_RESP_DATA,
                           Uint16_FromBigEndian(rx_frame->sequence),
                           payload, payload_len);

    return HAL_OK;
}

/**
 * @brief Handle SET_MODE command
 */
HAL_StatusTypeDef HandleCmd_SetMode(SPI_Comm_State *comm,
                                    const SPI_Frame *rx_frame,
                                    SPI_Frame *tx_frame)
{
    // Input validation
    if (comm == NULL || rx_frame == NULL) {
        return HAL_ERROR;
    }

    // Check payload length
    if (rx_frame->length != sizeof(Payload_SetMode)) {
        if (tx_frame != NULL) {
            Payload_AckError error_payload;
            error_payload.error_code = SPI_ERROR_LENGTH;
            SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_ERROR,
                                  Uint16_FromBigEndian(rx_frame->sequence),
                                  &error_payload, sizeof(error_payload));
        }
        return HAL_ERROR;
    }

    // Extract mode
    uint8_t mode = rx_frame->payload[0];

    // Validate mode range
    if (mode > VEHICLE_MODE_EMERGENCY) {
        if (tx_frame != NULL) {
            Payload_AckError error_payload;
            error_payload.error_code = SPI_ERROR_INVALID_VALUE;
            SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_ERROR,
                                  Uint16_FromBigEndian(rx_frame->sequence),
                                  &error_payload, sizeof(error_payload));
        }
        return HAL_ERROR;
    }

    // Update mode cache
    comm->target_mode = (Vehicle_Mode)mode;

    SPI_PendingVehicleCommand pending = {
        .command = SPI_CMD_SET_MODE,
        .steering_deg = 0.0f,
        .throttle_percent = 0.0f,
        .mode = (Vehicle_Mode)mode
    };
    if (!SPI_Comm_QueueVehicleCommand(comm, &pending)) {
        if (tx_frame != NULL) {
            Payload_AckError error_payload = { .error_code = SPI_ERROR_BUSY };
            SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_ERROR,
                                   Uint16_FromBigEndian(rx_frame->sequence),
                                   &error_payload, sizeof(error_payload));
        }
        return HAL_BUSY;
    }

    // Send ACK_OK response
    if (tx_frame != NULL) {
        SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_OK,
                               Uint16_FromBigEndian(rx_frame->sequence),
                               NULL, 0);
    }

    return HAL_OK;
}

/**
 * @brief Handle EMERGENCY_STOP command
 */
HAL_StatusTypeDef HandleCmd_EmergencyStop(SPI_Comm_State *comm,
                                          const SPI_Frame *rx_frame,
                                          SPI_Frame *tx_frame)
{
    // Input validation
    if (comm == NULL || rx_frame == NULL) {
        return HAL_ERROR;
    }

    // Set emergency stop state in cache
    comm->target_steering_deg = 0.0f;
    comm->target_throttle_percent = 0.0f;
    comm->target_mode = VEHICLE_MODE_EMERGENCY;

    SPI_PendingVehicleCommand pending = {
        .command = SPI_CMD_EMERGENCY_STOP,
        .steering_deg = 0.0f,
        .throttle_percent = 0.0f,
        .mode = VEHICLE_MODE_EMERGENCY
    };
    if (!SPI_Comm_QueueVehicleCommand(comm, &pending)) {
        return HAL_BUSY;
    }

    // Send ACK_OK response
    if (tx_frame != NULL) {
        SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_OK,
                               Uint16_FromBigEndian(rx_frame->sequence),
                               NULL, 0);
    }

    return HAL_OK;
}

/**
 * @brief Handle HEARTBEAT command
 */
HAL_StatusTypeDef HandleCmd_Heartbeat(SPI_Comm_State *comm,
                                      const SPI_Frame *rx_frame,
                                      SPI_Frame *tx_frame)
{
    // Input validation
    if (comm == NULL || rx_frame == NULL) {
        return HAL_ERROR;
    }

    // Update watchdog timestamp (already done in ProcessFrame, but explicit here)
    comm->last_valid_command_time = HAL_GetTick();

    // Send ACK_OK response
    if (tx_frame != NULL) {
        SPI_Comm_BuildResponse(tx_frame, SPI_RESP_ACK_OK,
                               Uint16_FromBigEndian(rx_frame->sequence),
                               NULL, 0);
    }

    return HAL_OK;
}

/* ============================================ */
/* UART Test Interface Implementation          */
/* ============================================ */

#if (SPI_COMM_TEST_ENABLE || SPI_VEHICLE_TEST_ENABLE)

/**
 * @brief Parse UART test command and generate SPI frame
 *
 * This function is shared by both SPI_COMM_TEST and SPI_VEHICLE_TEST modes
 */
HAL_StatusTypeDef UART_ParseTestCommand(const char *uart_buffer, SPI_Frame *frame)
{
    // Input validation
    if (uart_buffer == NULL || frame == NULL) {
        return HAL_ERROR;
    }

    // Clear frame
    memset(frame, 0, sizeof(SPI_Frame));

    // Skip leading whitespace and control characters
    while (*uart_buffer && (*uart_buffer <= ' ')) {
        uart_buffer++;
    }

    // Check if buffer is empty after trimming
    if (*uart_buffer == '\0') {
        return HAL_ERROR;
    }

    // Parse command
    char cmd[32];
    memset(cmd, 0, sizeof(cmd));
    if (sscanf(uart_buffer, "%31s", cmd) != 1) {
        return HAL_ERROR;
    }

    // SET_CONTROL command
    if (strcmp(cmd, "SET_CONTROL") == 0) {
        // Parse using strtof (newlib-nano doesn't support sscanf %f)
        const char *ptr = uart_buffer + 11;  // Skip "SET_CONTROL"

        // Skip whitespace
        while (*ptr && (*ptr == ' ' || *ptr == '\t')) ptr++;
        if (*ptr == '\0') return HAL_ERROR;

        // Parse steering
        char *endptr;
        float steering = strtof(ptr, &endptr);
        if (endptr == ptr) return HAL_ERROR;  // No conversion

        // Skip whitespace
        ptr = endptr;
        while (*ptr && (*ptr == ' ' || *ptr == '\t')) ptr++;
        if (*ptr == '\0') return HAL_ERROR;

        // Parse throttle
        float throttle = strtof(ptr, &endptr);
        if (endptr == ptr) return HAL_ERROR;  // No conversion

        // Build frame (clear entire frame first to zero Padding)
        memset(frame, 0, sizeof(SPI_Frame));
        frame->header = SPI_FRAME_HEADER;
        frame->version_flags = SPI_VERSION_FLAGS;
        Uint16_ToBigEndian(0U, frame->sequence);
        frame->command = SPI_CMD_SET_CONTROL;
        frame->length = sizeof(Payload_SetControl);

        // Encode payload (Big-Endian)
        Float_ToBigEndian(steering, &frame->payload[0]);
        Float_ToBigEndian(throttle, &frame->payload[4]);

        // Calculate CRC (fixed 29 bytes)
        uint16_t crc = SPI_Comm_CalculateCRC16((const uint8_t*)frame, 29);
        Uint16_ToBigEndian(crc, frame->crc16);

        frame->footer = SPI_FRAME_FOOTER;
        return HAL_OK;
    }
    // SET_MODE command
    else if (strcmp(cmd, "SET_MODE") == 0) {
        char mode_str[16];
        if (sscanf(uart_buffer, "SET_MODE %s", mode_str) == 1) {
            uint8_t mode = VEHICLE_MODE_IDLE;

            if (strcmp(mode_str, "IDLE") == 0) mode = VEHICLE_MODE_IDLE;
            else if (strcmp(mode_str, "REMOTE") == 0) mode = VEHICLE_MODE_REMOTE;
            else if (strcmp(mode_str, "MANUAL") == 0) mode = VEHICLE_MODE_MANUAL;
            else if (strcmp(mode_str, "EMERGENCY") == 0) mode = VEHICLE_MODE_EMERGENCY;
            else return HAL_ERROR;

            // Build frame (clear entire frame first to zero Padding)
            memset(frame, 0, sizeof(SPI_Frame));
            frame->header = SPI_FRAME_HEADER;
            frame->version_flags = SPI_VERSION_FLAGS;
            Uint16_ToBigEndian(0U, frame->sequence);
            frame->command = SPI_CMD_SET_MODE;
            frame->length = 1;
            frame->payload[0] = mode;

            // Calculate CRC (fixed 29 bytes)
            uint16_t crc = SPI_Comm_CalculateCRC16((const uint8_t*)frame, 29);
            Uint16_ToBigEndian(crc, frame->crc16);

            frame->footer = SPI_FRAME_FOOTER;
            return HAL_OK;
        }
    }
    // GET_STATUS command
    else if (strcmp(cmd, "GET_STATUS") == 0) {
        memset(frame, 0, sizeof(SPI_Frame));
        frame->header = SPI_FRAME_HEADER;
        frame->version_flags = SPI_VERSION_FLAGS;
        Uint16_ToBigEndian(0U, frame->sequence);
        frame->command = SPI_CMD_GET_STATUS;
        frame->length = 0;

        uint16_t crc = SPI_Comm_CalculateCRC16((const uint8_t*)frame, 29);
        Uint16_ToBigEndian(crc, frame->crc16);

        frame->footer = SPI_FRAME_FOOTER;
        return HAL_OK;
    }
    // GET_SENSORS command
    else if (strcmp(cmd, "GET_SENSORS") == 0) {
        memset(frame, 0, sizeof(SPI_Frame));
        frame->header = SPI_FRAME_HEADER;
        frame->version_flags = SPI_VERSION_FLAGS;
        Uint16_ToBigEndian(0U, frame->sequence);
        frame->command = SPI_CMD_GET_SENSORS;
        frame->length = 0;

        uint16_t crc = SPI_Comm_CalculateCRC16((const uint8_t*)frame, 29);
        Uint16_ToBigEndian(crc, frame->crc16);

        frame->footer = SPI_FRAME_FOOTER;
        return HAL_OK;
    }
    // EMERGENCY_STOP command
    else if (strcmp(cmd, "EMERGENCY_STOP") == 0) {
        memset(frame, 0, sizeof(SPI_Frame));
        frame->header = SPI_FRAME_HEADER;
        frame->version_flags = SPI_VERSION_FLAGS;
        Uint16_ToBigEndian(0U, frame->sequence);
        frame->command = SPI_CMD_EMERGENCY_STOP;
        frame->length = 0;

        uint16_t crc = SPI_Comm_CalculateCRC16((const uint8_t*)frame, 29);
        Uint16_ToBigEndian(crc, frame->crc16);

        frame->footer = SPI_FRAME_FOOTER;
        return HAL_OK;
    }
    // HEARTBEAT command
    else if (strcmp(cmd, "HEARTBEAT") == 0) {
        memset(frame, 0, sizeof(SPI_Frame));
        frame->header = SPI_FRAME_HEADER;
        frame->version_flags = SPI_VERSION_FLAGS;
        Uint16_ToBigEndian(0U, frame->sequence);
        frame->command = SPI_CMD_HEARTBEAT;
        frame->length = 0;

        uint16_t crc = SPI_Comm_CalculateCRC16((const uint8_t*)frame, 29);
        Uint16_ToBigEndian(crc, frame->crc16);

        frame->footer = SPI_FRAME_FOOTER;
        return HAL_OK;
    }

    return HAL_ERROR;  // Unknown command
}

#endif /* SPI_COMM_TEST_ENABLE || SPI_VEHICLE_TEST_ENABLE */

/* ============================================ */
/* SPI Communication Test (Protocol Only)      */
/* ============================================ */

#if SPI_COMM_TEST_ENABLE

/**
 * @brief SPI communication test program (UART simulation mode)
 */
void SPI_Comm_Test(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    char uart_tx_buffer[512];
    char uart_rx_buffer[128];
    SPI_Comm_State spi_comm;
    SPI_Frame test_frame;
    SPI_Frame response_frame;

    // Print header
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "  CoVAPSy Phase 2 SPI_Comm Test\r\n"
             "  UART Command Mode (20Hz simulation)\r\n"
             "========================================\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Initialize SPI communication
    memset(&spi_comm, 0, sizeof(spi_comm));  // Clear all fields
    spi_comm.hspi = hspi;
    spi_comm.huart_debug = huart;
    spi_comm.vehicle = NULL;  // Test mode: use dummy data for GET_STATUS/GET_SENSORS

    if (SPI_Comm_Init(&spi_comm) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[FATAL] SPI_Comm initialization failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        while (1) {
            HAL_Delay(1000);
        }
    }

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] SPI_Comm initialized!\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Print usage
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "Supported commands:\r\n"
             "  SET_CONTROL <steering> <throttle>\r\n"
             "  SET_MODE <IDLE|REMOTE|MANUAL|EMERGENCY>\r\n"
             "  GET_STATUS\r\n"
             "  GET_SENSORS\r\n"
             "  EMERGENCY_STOP\r\n"
             "  HEARTBEAT\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Main test loop
    while (1) {
        // Prompt
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "Enter command:\r\n> ");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Receive command (simplified - in real implementation use interrupt)
        memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
        uint8_t idx = 0;
        while (idx < sizeof(uart_rx_buffer) - 1) {
            uint8_t ch;
            if (HAL_UART_Receive(huart, &ch, 1, 5000) == HAL_OK) {
                if (ch == '\r' || ch == '\n') {
                    break;
                }
                uart_rx_buffer[idx++] = ch;
                HAL_UART_Transmit(huart, &ch, 1, 10);  // Echo
            }
        }
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "\r\n\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Debug: Print received command (show hidden characters)
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[DEBUG] Received: \"%s\" (len=%d)\r\n",
                 uart_rx_buffer, (int)strlen(uart_rx_buffer));
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

        // Parse command
        if (UART_ParseTestCommand(uart_rx_buffer, &test_frame) == HAL_OK) {
            // Print generated frame
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[SPI_Comm] Generated SPI Frame:\r\n"
                     "  Header:  0x%02X\r\n"
                     "  Command: 0x%02X\r\n"
                     "  Length:  %u\r\n"
                     "  CRC16:   0x%04X\r\n"
                     "  Footer:  0x%02X\r\n\r\n",
                     test_frame.header,
                     test_frame.command,
                     test_frame.length,
                     Uint16_FromBigEndian(test_frame.crc16),
                     test_frame.footer);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

            // Validate frame
            if (SPI_Comm_ValidateFrame(&test_frame)) {
                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                         "[SPI_Comm] Frame validation: PASSED\r\n");
                HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                // Process frame
                if (SPI_Comm_ProcessFrame(&spi_comm, &test_frame, &response_frame) == HAL_OK) {
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                             "[SPI_Comm] Command processed successfully\r\n");
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                    // Print response header
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                             "[SPI_Comm] Response: 0x%02X (len=%d)\r\n",
                             response_frame.command, response_frame.length);
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                    // Parse and display DATA_RESPONSE content
                    if (response_frame.command == SPI_RESP_DATA && response_frame.length > 0) {
                        uint8_t *p = response_frame.payload;

                        // Check if this is GET_STATUS response (15 bytes)
                        if (test_frame.command == SPI_CMD_GET_STATUS && response_frame.length >= 15) {
                            uint8_t mode = p[0];
                            float steer = Float_FromBigEndian(&p[1]);
                            float throttle = Float_FromBigEndian(&p[5]);
                            uint8_t reversing = p[9];
                            uint8_t safety = p[10];
                            uint32_t counter = Uint32_FromBigEndian(&p[11]);

                            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                     "  [STATUS] Mode=%d Steer=%.1f Throttle=%.1f\r\n"
                                     "           Rev=%d Safety=%d Counter=%lu\r\n",
                                     mode, steer, throttle, reversing, safety, counter);
                            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                        }
                        // Check if this is GET_SENSORS response (23 bytes)
                        else if (test_frame.command == SPI_CMD_GET_SENSORS && response_frame.length >= 23) {
                            float sharp_l = Float_FromBigEndian(&p[0]);
                            float sharp_r = Float_FromBigEndian(&p[4]);
                            uint8_t sharp_l_v = p[8];
                            uint8_t sharp_r_v = p[9];
                            float roll = Float_FromBigEndian(&p[10]);
                            float pitch = Float_FromBigEndian(&p[14]);
                            float yaw = Float_FromBigEndian(&p[18]);
                            uint8_t imu_v = p[22];

                            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                     "  [SENSORS] SHARP: L=%.1fcm(%d) R=%.1fcm(%d)\r\n"
                                     "            IMU: R=%.1f P=%.1f Y=%.1f (%d)\r\n",
                                     sharp_l, sharp_l_v, sharp_r, sharp_r_v,
                                     roll, pitch, yaw, imu_v);
                            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                        }
                    }
                } else {
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                             "[SPI_Comm] Command processing FAILED\r\n");
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                }
            } else {
                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                         "[SPI_Comm] Frame validation: FAILED\r\n");
                HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
            }

            // Print statistics
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "\r\n");
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
            SPI_Comm_PrintStats(&spi_comm);

        } else {
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[ERROR] Invalid command format\r\n");
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        }

        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "========================================\r\n\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
    }
}

#endif /* SPI_COMM_TEST_ENABLE */

/* ============================================ */
/* SPI + Vehicle Integration Test               */
/* ============================================ */

#if SPI_VEHICLE_TEST_ENABLE

/**
 * @brief SPI + Vehicle integration test implementation
 */
void SPI_Vehicle_IntegrationTest(TIM_HandleTypeDef *htim,
                                  I2C_HandleTypeDef *hi2c,
                                  ADC_HandleTypeDef *hadc,
                                  UART_HandleTypeDef *huart)
{
    char uart_tx_buffer[650];  // Increased size to fit help message (592 bytes needed)
    char uart_rx_buffer[100];
    uint8_t rx_index = 0;

    // DMA buffer for SHARP sensors (2 channels)
    static uint16_t sharp_adc_dma_buffer[2];

    // Vehicle state
    Vehicle_State vehicle = {0};

    // SPI communication state
    SPI_Comm_State spi_comm = {0};
    SPI_Frame test_frame = {0};
    SPI_Frame response_frame = {0};

    // === Step 1: Print welcome banner ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n"
             "========================================\r\n"
             "  CoVAPSy SPI + Vehicle Integration Test\r\n"
             "  UART -> SPI_Comm -> Vehicle_Control\r\n"
             "========================================\r\n"
             "\r\n"
             "WARNING: This test controls REAL actuators!\r\n"
             "         Ensure vehicle is safely positioned.\r\n"
             "\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // === Step 2: Initialize Vehicle ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INIT] Initializing vehicle systems...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // Set hardware handles
    vehicle.htim_servo_esc = htim;
    vehicle.hi2c_imu = hi2c;
    vehicle.hadc_sharp = hadc;
    vehicle.huart_debug = huart;
    vehicle.sharp_dma_buffer = sharp_adc_dma_buffer;
    vehicle.sharp_dma_buffer_size = 2;

    // Initialize vehicle
    if (Vehicle_Init(&vehicle) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] Vehicle initialization failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        return;
    }

    // === Step 3: Initialize SPI Communication ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INIT] Initializing SPI communication...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    spi_comm.huart_debug = huart;
    spi_comm.vehicle = &vehicle;  // Link to vehicle state!
    spi_comm.target_mode = VEHICLE_MODE_IDLE;
    spi_comm.last_valid_command_time = HAL_GetTick();

    // Set global pointer for Vehicle_ControlLoop integration
    g_spi_comm_ptr = &spi_comm;

    if (SPI_Comm_Init(&spi_comm) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] SPI communication initialization failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        return;
    }

    // === Step 4: Arm ESC ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INIT] Arming ESC (2 seconds neutral pulse)...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    if (ESC_Arm(htim, &vehicle.esc_data, huart) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[ERROR] ESC arming failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        return;
    }

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[OK] ESC armed successfully!\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // === Step 5: Set to REMOTE mode for SPI control ===
    Vehicle_SetMode(&vehicle, VEHICLE_MODE_REMOTE);
    spi_comm.target_mode = VEHICLE_MODE_REMOTE;
    vehicle.last_command_timestamp = HAL_GetTick();  // Reset watchdog

    // === Step 6: Print help ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "========================================\r\n"
             "  Available Commands:\r\n"
             "========================================\r\n"
             "  SET_CONTROL <steer> <throttle>\r\n"
             "    Example: SET_CONTROL 15.0 30.0\r\n"
             "    Steering: -30 to +30 deg\r\n"
             "    Throttle: -100 to +100 %%\r\n"
             "\r\n"
             "  SET_MODE <mode>\r\n"
             "    Modes: IDLE, REMOTE, MANUAL, EMERGENCY\r\n"
             "\r\n"
             "  GET_STATUS    - Get vehicle status\r\n"
             "  GET_SENSORS   - Get sensor data\r\n"
             "  EMERGENCY_STOP - Emergency stop\r\n"
             "  HEARTBEAT     - Reset watchdog\r\n"
             "  STOP          - Quick stop (throttle=0)\r\n"
             "  HELP          - Show this help\r\n"
             "========================================\r\n\r\n"
             "[READY] Enter command:\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // === Step 7: Main loop - UART command + 50Hz control ===
    uint32_t last_control_time = HAL_GetTick();
    uint32_t last_status_time = HAL_GetTick();
    const uint32_t CONTROL_PERIOD_MS = 20;   // 50Hz
    const uint32_t STATUS_PERIOD_MS = 1000;  // 1Hz status output

    while (1) {
        uint32_t now = HAL_GetTick();

        // --- 50Hz Control Loop ---
        if (now - last_control_time >= CONTROL_PERIOD_MS) {
            last_control_time = now;

            // Update sensors
            Vehicle_UpdateSensors(&vehicle);

            // Run control loop (includes safety checks)
            if (Vehicle_ControlLoop(&vehicle) != HAL_OK) {
                // Safety triggered - already handled in Vehicle_ControlLoop
            }
        }

        // --- 1Hz Status Output (optional, can be commented out) ---
        if (now - last_status_time >= STATUS_PERIOD_MS) {
            last_status_time = now;
            // Uncomment below to enable periodic status output:
            // Vehicle_PrintStatus(&vehicle);
        }

        // --- Check for UART input ---
        uint8_t rx_byte;
        if (HAL_UART_Receive(huart, &rx_byte, 1, 1) == HAL_OK) {
            // Echo character
            HAL_UART_Transmit(huart, &rx_byte, 1, 10);

            if (rx_byte == '\r' || rx_byte == '\n') {
                if (rx_index > 0) {
                    uart_rx_buffer[rx_index] = '\0';

                    // Print newline
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "\r\n");
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                    // Reset watchdog timestamp on any command
                    vehicle.last_command_timestamp = HAL_GetTick();
                    spi_comm.last_valid_command_time = HAL_GetTick();

                    // --- Handle special commands ---
                    if (strcmp(uart_rx_buffer, "HELP") == 0) {
                        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                 "Commands: SET_CONTROL, SET_MODE, GET_STATUS,\r\n"
                                 "          GET_SENSORS, EMERGENCY_STOP, HEARTBEAT, STOP\r\n");
                        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                    }
                    else if (strcmp(uart_rx_buffer, "STOP") == 0) {
                        // Quick stop - set throttle to 0
                        spi_comm.target_throttle_percent = 0.0f;
                        Vehicle_SetTargetThrottle(&vehicle, 0.0f);
                        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                 "[OK] Throttle set to 0%%\r\n");
                        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                    }
                    // --- Parse as SPI command ---
                    else if (UART_ParseTestCommand(uart_rx_buffer, &test_frame) == HAL_OK) {
                        // Print generated frame info
                        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                 "[SPI] Cmd=0x%02X Len=%d\r\n",
                                 test_frame.command, test_frame.length);
                        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                        // Validate and process
                        if (SPI_Comm_ValidateFrame(&test_frame)) {
                            if (SPI_Comm_ProcessFrame(&spi_comm, &test_frame, &response_frame) == HAL_OK) {
                                // Apply control values to vehicle immediately
                                if (test_frame.command == SPI_CMD_SET_CONTROL) {
                                    Vehicle_SetTargetSteering(&vehicle, spi_comm.target_steering_deg);
                                    Vehicle_SetTargetThrottle(&vehicle, spi_comm.target_throttle_percent);

                                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                             "[OK] Steering=%.1f deg, Throttle=%.1f%%\r\n",
                                             spi_comm.target_steering_deg,
                                             spi_comm.target_throttle_percent);
                                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                                }
                                else if (test_frame.command == SPI_CMD_GET_STATUS) {
                                    // Print status data
                                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                             "[STATUS] Mode=%d Steer=%.1f Throttle=%.1f Rev=%d\r\n",
                                             vehicle.mode,
                                             vehicle.current_steering_deg,
                                             vehicle.current_throttle_percent,
                                             vehicle.is_reversing);
                                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                                }
                                else if (test_frame.command == SPI_CMD_GET_SENSORS) {
                                    // Print sensor data
                                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                             "[SENSORS] SHARP L=%.1fcm R=%.1fcm\r\n"
                                             "          IMU R=%.1f P=%.1f Y=%.1f\r\n",
                                             vehicle.sharp_left_distance_cm,
                                             vehicle.sharp_right_distance_cm,
                                             vehicle.roll_deg,
                                             vehicle.pitch_deg,
                                             vehicle.yaw_deg);
                                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                                }
                                else if (test_frame.command == SPI_CMD_SET_MODE) {
                                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                             "[OK] Mode changed to %d\r\n", vehicle.mode);
                                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                                }
                                else if (test_frame.command == SPI_CMD_EMERGENCY_STOP) {
                                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                             "[OK] Emergency stop activated!\r\n");
                                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                                }
                                else if (test_frame.command == SPI_CMD_HEARTBEAT) {
                                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                             "[OK] Heartbeat received\r\n");
                                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                                }
                                else {
                                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                             "[OK] Response=0x%02X\r\n", response_frame.command);
                                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                                }
                            } else {
                                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                         "[ERROR] Command processing failed\r\n");
                                HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                            }
                        } else {
                            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                     "[ERROR] Frame validation failed\r\n");
                            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                        }
                    }
                    else {
                        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                                 "[ERROR] Unknown command: %s\r\n", uart_rx_buffer);
                        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                    }

                    // Print prompt
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "> ");
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                    rx_index = 0;
                }
            }
            else if (rx_byte == 0x7F || rx_byte == 0x08) {  // Backspace
                if (rx_index > 0) {
                    rx_index--;
                    // Erase character on terminal
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "\b \b");
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                }
            }
            else if (rx_index < sizeof(uart_rx_buffer) - 1) {
                uart_rx_buffer[rx_index++] = rx_byte;
            }
        }
    }
}

#endif /* SPI_VEHICLE_TEST_ENABLE */

/* ============================================ */
/* SPI Hardware Debug Mode Implementation       */
/* ============================================ */

#if SPI_HARDWARE_DEBUG_ENABLE

/**
 * @brief Get command name string for debug output
 */
static const char* GetCommandName(uint8_t cmd)
{
    switch (cmd) {
        case SPI_CMD_GET_STATUS:    return "GET_STATUS";
        case SPI_CMD_SET_CONTROL:   return "SET_CONTROL";
        case SPI_CMD_GET_SENSORS:   return "GET_SENSORS";
        case SPI_CMD_SET_MODE:      return "SET_MODE";
        case SPI_CMD_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case SPI_CMD_HEARTBEAT:     return "HEARTBEAT";
        default:                    return "UNKNOWN";
    }
}

/**
 * @brief SPI Hardware Debug Mode
 */
void SPI_Hardware_Debug(SPI_HandleTypeDef *hspi,
                        TIM_HandleTypeDef *htim,
                        I2C_HandleTypeDef *hi2c,
                        ADC_HandleTypeDef *hadc,
                        UART_HandleTypeDef *huart)
{
    char uart_tx_buffer[256];

    // === Step 1: Print header ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "\r\n========================================\r\n"
             "  CoVAPSy SPI Hardware Debug Mode\r\n"
             "  Real SPI + Vehicle + UART Debug\r\n"
             "========================================\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // === Step 2: Initialize Vehicle ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "[INIT] Initializing vehicle...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    Vehicle_State vehicle = {0};
    vehicle.htim_servo_esc = htim;
    vehicle.hi2c_imu = hi2c;
    vehicle.hadc_sharp = hadc;
    vehicle.huart_debug = huart;

    // Allocate DMA buffer for SHARP sensors
    static uint16_t sharp_dma_buffer[2] = {0};
    vehicle.sharp_dma_buffer = sharp_dma_buffer;
    vehicle.sharp_dma_buffer_size = 2;

    // Initialize vehicle
    Vehicle_Init(&vehicle);

    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "[INIT] Arming ESC...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    ESC_Arm(htim, &vehicle.esc_data, huart);

    // === Step 3: Initialize SPI Communication ===
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "[INIT] Initializing SPI3...\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    static SPI_Comm_State spi_comm = {0};
    spi_comm.hspi = hspi;
    spi_comm.huart_debug = huart;
    spi_comm.vehicle = &vehicle;

    if (SPI_Comm_Init(&spi_comm) != HAL_OK) {
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                 "[FATAL] SPI_Comm initialization failed!\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        while (1) { HAL_Delay(1000); }
    }

    // === Step 4: Set REMOTE mode ===
    Vehicle_SetMode(&vehicle, VEHICLE_MODE_REMOTE);
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
             "[INIT] Vehicle mode: REMOTE\r\n"
             "[INIT] Waiting for SPI commands from Raspberry Pi...\r\n\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

    // === Step 5: Main loop ===
    uint32_t last_control_time = HAL_GetTick();
    uint32_t last_status_time = HAL_GetTick();
    uint32_t last_frame_count = 0;
    const uint32_t CONTROL_PERIOD_MS = 20;   // 50Hz
    const uint32_t STATUS_PERIOD_MS = 1000;  // 1Hz status output

    while (1) {
        uint32_t now = HAL_GetTick();

        // --- 50Hz Control Loop ---
        if (now - last_control_time >= CONTROL_PERIOD_MS) {
            last_control_time = now;

            // Update sensors
            Vehicle_UpdateSensors(&vehicle);

            // Check for new SPI command and print debug info
            if (spi_comm.buffer_swap_flag) {
                // Copy from primary to secondary buffer
                __disable_irq();
                SPI_Frame rx_copy = spi_comm.rx_buffer_primary;
                spi_comm.buffer_swap_flag = 0;
                __enable_irq();

                // Print received command
                snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                         "[SPI RX] Cmd=0x%02X (%s) Len=%d\r\n",
                         rx_copy.command,
                         GetCommandName(rx_copy.command),
                         rx_copy.length);
                HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

                // Print payload for SET_CONTROL
                if (rx_copy.command == SPI_CMD_SET_CONTROL && rx_copy.length >= 8) {
                    float steering = Float_FromBigEndian(&rx_copy.payload[0]);
                    float throttle = Float_FromBigEndian(&rx_copy.payload[4]);
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                             "        Steering=%.1f deg, Throttle=%.1f%%\r\n",
                             steering, throttle);
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                }
                // Print payload for SET_MODE
                else if (rx_copy.command == SPI_CMD_SET_MODE && rx_copy.length >= 1) {
                    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                             "        Mode=%d\r\n", rx_copy.payload[0]);
                    HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
                }

                // Apply control values
                Vehicle_SetTargetSteering(&vehicle, spi_comm.target_steering_deg);
                Vehicle_SetTargetThrottle(&vehicle, spi_comm.target_throttle_percent);
            }

            // Run control loop
            Vehicle_ControlLoop(&vehicle);
        }

        // --- 1Hz Status Output ---
        if (now - last_status_time >= STATUS_PERIOD_MS) {
            last_status_time = now;

            uint32_t new_frames = spi_comm.frame_received_count - last_frame_count;
            last_frame_count = spi_comm.frame_received_count;

            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                     "[STATUS] RX=%lu (+%lu/s) Err=%lu | Steer=%.1f Thr=%.1f | "
                     "SHARP L=%.1f R=%.1f\r\n",
                     spi_comm.frame_received_count, new_frames,
                     spi_comm.frame_error_count + spi_comm.crc_error_count,
                     vehicle.current_steering_deg, vehicle.current_throttle_percent,
                     vehicle.sharp_left_distance_cm, vehicle.sharp_right_distance_cm);
            HAL_UART_Transmit(huart, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
        }
    }
}

#endif /* SPI_HARDWARE_DEBUG_ENABLE */

/* ============================================ */
/* SPI Production Mode Implementation           */
/* ============================================ */

#if SPI_PRODUCTION_ENABLE

/**
 * @brief SPI Production Mode (Real SPI + Vehicle, NO debug output)
 */
void SPI_Production(SPI_HandleTypeDef *hspi,
                    TIM_HandleTypeDef *htim,
                    I2C_HandleTypeDef *hi2c,
                    ADC_HandleTypeDef *hadc)
{
    // === Step 1: Initialize Vehicle ===
    Vehicle_State vehicle = {0};
    vehicle.htim_servo_esc = htim;
    vehicle.hi2c_imu = hi2c;
    vehicle.hadc_sharp = hadc;
    vehicle.huart_debug = NULL;  // No debug output in production mode

    // Allocate DMA buffer for SHARP sensors
    static uint16_t sharp_dma_buffer[2] = {0};
    vehicle.sharp_dma_buffer = sharp_dma_buffer;
    vehicle.sharp_dma_buffer_size = 2;

    // Initialize vehicle
    Vehicle_Init(&vehicle);

    // Arm ESC (2-second neutral pulse)
    ESC_Arm(htim, &vehicle.esc_data, NULL);

    // === Step 2: Initialize SPI Communication ===
    static SPI_Comm_State spi_comm = {0};
    spi_comm.hspi = hspi;
    spi_comm.huart_debug = NULL;  // No debug output
    spi_comm.vehicle = &vehicle;

    if (SPI_Comm_Init(&spi_comm) != HAL_OK) {
        // Fatal error - blink LED or halt
        while (1) { HAL_Delay(100); }
    }

    // === Step 3: Set REMOTE mode ===
    Vehicle_SetMode(&vehicle, VEHICLE_MODE_REMOTE);

    // === Step 4: Main loop (50Hz control, no debug output) ===
    uint32_t last_control_time = HAL_GetTick();
    const uint32_t CONTROL_PERIOD_MS = 20;  // 50Hz

    while (1) {
        uint32_t now = HAL_GetTick();

        // --- 50Hz Control Loop ---
        if (now - last_control_time >= CONTROL_PERIOD_MS) {
            last_control_time = now;

            // Update sensors
            Vehicle_UpdateSensors(&vehicle);

            // Check for new SPI command
            if (spi_comm.buffer_swap_flag) {
                // Copy from primary to secondary buffer (atomic)
                __disable_irq();
                spi_comm.buffer_swap_flag = 0;
                __enable_irq();

                // Apply control values
                Vehicle_SetTargetSteering(&vehicle, spi_comm.target_steering_deg);
                Vehicle_SetTargetThrottle(&vehicle, spi_comm.target_throttle_percent);
            }

            // Run control loop
            Vehicle_ControlLoop(&vehicle);
        }
    }
}

#endif /* SPI_PRODUCTION_ENABLE */

#endif /* SPI_COMM_ENABLE */
