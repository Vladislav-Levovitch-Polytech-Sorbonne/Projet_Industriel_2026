#!/usr/bin/env python3
"""
CoVAPSy SPI Communication Library
=================================

SPI communication library for Raspberry Pi 4B to control CoVAPSy autonomous vehicle.

Protocol:
- Frame size: 32 bytes fixed
- CRC: CRC-16/MODBUS
- Byte order: Big-endian
- SPI Mode: 0 (CPOL=0, CPHA=0)
- Speed: 1 MHz

Author: CoVAPSy Team
Date: 2026-01-16
"""

import struct
import time
from typing import Optional, Dict, Any, Tuple

# Try to import spidev (only available on Raspberry Pi)
try:
    import spidev
    SPI_AVAILABLE = True
except ImportError:
    SPI_AVAILABLE = False
    print("[Warning] spidev not available, running in simulation mode")


# ============================================
# Protocol Constants
# ============================================

# Frame structure
FRAME_SIZE = 32
HEADER_BYTE = 0xAA
FOOTER_BYTE = 0x55
MAX_PAYLOAD_SIZE = 26

# Command codes (Pi -> STM32)
CMD_GET_STATUS = 0x01
CMD_SET_CONTROL = 0x02
CMD_GET_SENSORS = 0x03
CMD_SET_MODE = 0x04
CMD_EMERGENCY_STOP = 0x05
CMD_HEARTBEAT = 0x10

# Response codes (STM32 -> Pi)
RESP_ACK_OK = 0x80
RESP_ACK_ERROR = 0x81
RESP_DATA = 0x82

# Error codes
ERROR_NONE = 0x00
ERROR_CRC = 0x01
ERROR_HEADER = 0x02
ERROR_FOOTER = 0x03
ERROR_LENGTH = 0x04
ERROR_UNKNOWN_CMD = 0x05
ERROR_TIMEOUT = 0x06

# Vehicle modes
MODE_IDLE = 0
MODE_REMOTE = 1
MODE_MANUAL = 2
MODE_EMERGENCY = 3

# Mode name mapping
MODE_NAMES = {
    MODE_IDLE: "IDLE",
    MODE_REMOTE: "REMOTE",
    MODE_MANUAL: "MANUAL",
    MODE_EMERGENCY: "EMERGENCY"
}

# Error name mapping
ERROR_NAMES = {
    ERROR_NONE: "No error",
    ERROR_CRC: "CRC mismatch",
    ERROR_HEADER: "Invalid header",
    ERROR_FOOTER: "Invalid footer",
    ERROR_LENGTH: "Invalid length",
    ERROR_UNKNOWN_CMD: "Unknown command",
    ERROR_TIMEOUT: "Timeout"
}


# ============================================
# CRC-16/MODBUS Implementation
# ============================================

def calculate_crc16(data: bytes) -> int:
    """
    Calculate CRC-16/MODBUS checksum.

    Polynomial: 0x8005 (reflected: 0xA001)
    Initial value: 0xFFFF

    Args:
        data: Input bytes to calculate CRC for

    Returns:
        16-bit CRC value

    Test vector:
        Input: [0x01, 0x03, 0x00, 0x00, 0x00, 0x0A]
        Output: 0xC5CD
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ============================================
# CoVAPSy SPI Class
# ============================================

class CoVAPSySPI:
    """
    SPI communication class for CoVAPSy vehicle control.

    Features:
    - Full duplex SPI communication
    - CRC-16/MODBUS validation
    - All vehicle commands supported
    - Response parsing and validation

    Usage:
        spi = CoVAPSySPI()
        spi.open()
        result = spi.set_control(15.0, 30.0)
        spi.close()
    """

    def __init__(self, bus: int = 0, device: int = 0, speed: int = 1000000):
        """
        Initialize SPI communication.

        Args:
            bus: SPI bus number (default 0)
            device: SPI device/chip select (default 0 = CE0)
            speed: SPI clock speed in Hz (default 1 MHz)
        """
        self.bus = bus
        self.device = device
        self.speed = speed
        self.spi = None
        self.is_open = False

        # Statistics
        self.tx_count = 0
        self.rx_count = 0
        self.crc_errors = 0
        self.frame_errors = 0

    def open(self) -> bool:
        """
        Open SPI connection.

        Returns:
            True if successful, False otherwise
        """
        if not SPI_AVAILABLE:
            print("[SPI] Running in simulation mode (no spidev)")
            self.is_open = True
            return True

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.device)
            self.spi.max_speed_hz = self.speed
            self.spi.mode = 0  # CPOL=0, CPHA=0
            self.spi.bits_per_word = 8
            self.is_open = True
            print(f"[SPI] Opened bus={self.bus}, device={self.device}, speed={self.speed}Hz")
            return True
        except Exception as e:
            print(f"[SPI] Failed to open: {e}")
            return False

    def close(self):
        """Close SPI connection."""
        if self.spi:
            self.spi.close()
            self.spi = None
        self.is_open = False
        print("[SPI] Connection closed")

    def build_frame(self, command: int, payload: bytes = b'') -> bytes:
        """
        Build a 32-byte SPI frame.

        Frame format:
            [0]     Header (0xAA)
            [1]     Command
            [2]     Length (payload size)
            [3-28]  Payload (26 bytes, zero-padded)
            [29-30] CRC-16 (big-endian)
            [31]    Footer (0x55)

        Args:
            command: Command code (0x01-0x10)
            payload: Command payload data

        Returns:
            32-byte frame
        """
        if len(payload) > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {len(payload)} > {MAX_PAYLOAD_SIZE}")

        frame = bytearray(FRAME_SIZE)
        frame[0] = HEADER_BYTE
        frame[1] = command
        frame[2] = len(payload)
        frame[3:3+len(payload)] = payload

        # Calculate CRC over first 29 bytes
        crc = calculate_crc16(bytes(frame[:29]))
        frame[29] = (crc >> 8) & 0xFF  # CRC MSB
        frame[30] = crc & 0xFF          # CRC LSB
        frame[31] = FOOTER_BYTE

        return bytes(frame)

    def parse_response(self, data: bytes) -> Dict[str, Any]:
        """
        Parse and validate a response frame.

        Args:
            data: 32-byte response frame

        Returns:
            Dictionary with parsed data:
            - valid: True if frame is valid
            - command: Response command code
            - length: Payload length
            - payload: Raw payload bytes
            - error: Error message if invalid
            - data: Parsed data (for DATA responses)
        """
        result = {
            'valid': False,
            'command': 0,
            'length': 0,
            'payload': b'',
            'error': None,
            'data': None
        }

        if len(data) != FRAME_SIZE:
            result['error'] = f'Invalid frame size: {len(data)}'
            self.frame_errors += 1
            return result

        # Check header and footer
        if data[0] != HEADER_BYTE:
            result['error'] = f'Invalid header: 0x{data[0]:02X}'
            self.frame_errors += 1
            return result

        if data[31] != FOOTER_BYTE:
            result['error'] = f'Invalid footer: 0x{data[31]:02X}'
            self.frame_errors += 1
            return result

        # Verify CRC
        crc_calc = calculate_crc16(data[:29])
        crc_recv = (data[29] << 8) | data[30]
        if crc_calc != crc_recv:
            result['error'] = f'CRC mismatch: calc=0x{crc_calc:04X}, recv=0x{crc_recv:04X}'
            self.crc_errors += 1
            return result

        # Parse fields
        result['valid'] = True
        result['command'] = data[1]
        result['length'] = data[2]
        result['payload'] = bytes(data[3:3+data[2]])

        # Parse response data based on command type
        if data[1] == RESP_ACK_OK:
            result['data'] = {'ack': 'OK'}
        elif data[1] == RESP_ACK_ERROR:
            error_code = data[3] if data[2] > 0 else 0
            result['data'] = {
                'ack': 'ERROR',
                'error_code': error_code,
                'error_name': ERROR_NAMES.get(error_code, 'Unknown')
            }
        elif data[1] == RESP_DATA:
            result['data'] = self._parse_data_payload(result['payload'])

        self.rx_count += 1
        return result

    def _parse_data_payload(self, payload: bytes) -> Dict[str, Any]:
        """
        Parse DATA response payload based on length.

        Args:
            payload: Response payload bytes

        Returns:
            Parsed data dictionary
        """
        data = {}

        if len(payload) == 15:
            # GET_STATUS response
            data['type'] = 'status'
            data['mode'] = payload[0]
            data['mode_name'] = MODE_NAMES.get(payload[0], 'UNKNOWN')
            data['steering_deg'] = struct.unpack('>f', payload[1:5])[0]
            data['throttle_percent'] = struct.unpack('>f', payload[5:9])[0]
            data['is_reversing'] = payload[9]
            data['safety_stop'] = payload[10]
            data['loop_counter'] = struct.unpack('>I', payload[11:15])[0]

        elif len(payload) == 23:
            # GET_SENSORS response
            data['type'] = 'sensors'
            data['sharp_left_cm'] = struct.unpack('>f', payload[0:4])[0]
            data['sharp_right_cm'] = struct.unpack('>f', payload[4:8])[0]
            data['sharp_left_valid'] = payload[8]
            data['sharp_right_valid'] = payload[9]
            data['roll_deg'] = struct.unpack('>f', payload[10:14])[0]
            data['pitch_deg'] = struct.unpack('>f', payload[14:18])[0]
            data['yaw_deg'] = struct.unpack('>f', payload[18:22])[0]
            data['imu_valid'] = payload[22]

        else:
            data['type'] = 'unknown'
            data['raw'] = payload.hex()

        return data

    def _transfer(self, tx_frame: bytes) -> bytes:
        """
        Perform SPI transfer (send and receive).

        Args:
            tx_frame: Frame to send

        Returns:
            Received frame
        """
        self.tx_count += 1

        if not SPI_AVAILABLE or not self.spi:
            # Simulation mode - return empty response
            return bytes([HEADER_BYTE, RESP_ACK_OK, 0] + [0]*26 +
                        list(calculate_crc16(bytes([HEADER_BYTE, RESP_ACK_OK, 0] + [0]*26)).to_bytes(2, 'big')) +
                        [FOOTER_BYTE])

        # Full duplex transfer
        rx_data = self.spi.xfer2(list(tx_frame))
        return bytes(rx_data)

    # ============================================
    # Command Methods
    # ============================================

    def set_control(self, steering: float, throttle: float) -> Dict[str, Any]:
        """
        Set vehicle steering and throttle.

        Args:
            steering: Steering angle in degrees (-30 to +30)
            throttle: Throttle percentage (-100 to +100)

        Returns:
            Parsed response dictionary
        """
        # Clamp values
        steering = max(-30.0, min(30.0, steering))
        throttle = max(-100.0, min(100.0, throttle))

        # Pack payload (big-endian floats)
        payload = struct.pack('>ff', steering, throttle)

        # Build and send frame
        tx_frame = self.build_frame(CMD_SET_CONTROL, payload)
        rx_frame = self._transfer(tx_frame)

        return self.parse_response(rx_frame)

    def get_status(self) -> Dict[str, Any]:
        """
        Get vehicle status.

        Returns:
            Parsed response with status data:
            - mode: Current vehicle mode
            - steering_deg: Current steering angle
            - throttle_percent: Current throttle
            - is_reversing: Reverse flag
            - safety_stop: Safety stop flag
            - loop_counter: Control loop counter
        """
        tx_frame = self.build_frame(CMD_GET_STATUS)
        rx_frame = self._transfer(tx_frame)
        return self.parse_response(rx_frame)

    def get_sensors(self) -> Dict[str, Any]:
        """
        Get sensor data.

        Returns:
            Parsed response with sensor data:
            - sharp_left_cm: Left SHARP distance (cm)
            - sharp_right_cm: Right SHARP distance (cm)
            - sharp_left_valid: Left sensor validity
            - sharp_right_valid: Right sensor validity
            - roll_deg: IMU roll angle
            - pitch_deg: IMU pitch angle
            - yaw_deg: IMU yaw angle
            - imu_valid: IMU validity flag
        """
        tx_frame = self.build_frame(CMD_GET_SENSORS)
        rx_frame = self._transfer(tx_frame)
        return self.parse_response(rx_frame)

    def set_mode(self, mode: int) -> Dict[str, Any]:
        """
        Set vehicle operation mode.

        Args:
            mode: Mode value (0=IDLE, 1=REMOTE, 2=MANUAL, 3=EMERGENCY)

        Returns:
            Parsed response dictionary
        """
        if mode not in MODE_NAMES:
            raise ValueError(f"Invalid mode: {mode}")

        payload = bytes([mode])
        tx_frame = self.build_frame(CMD_SET_MODE, payload)
        rx_frame = self._transfer(tx_frame)
        return self.parse_response(rx_frame)

    def emergency_stop(self) -> Dict[str, Any]:
        """
        Trigger emergency stop.

        Immediately stops the vehicle:
        - Steering set to 0
        - Throttle set to 0
        - Mode changed to EMERGENCY

        Returns:
            Parsed response dictionary
        """
        tx_frame = self.build_frame(CMD_EMERGENCY_STOP)
        rx_frame = self._transfer(tx_frame)
        return self.parse_response(rx_frame)

    def heartbeat(self) -> Dict[str, Any]:
        """
        Send heartbeat to reset watchdog.

        Must be called at least every 500ms to prevent
        watchdog timeout in REMOTE mode.

        Returns:
            Parsed response dictionary
        """
        tx_frame = self.build_frame(CMD_HEARTBEAT)
        rx_frame = self._transfer(tx_frame)
        return self.parse_response(rx_frame)

    def get_statistics(self) -> Dict[str, int]:
        """
        Get communication statistics.

        Returns:
            Dictionary with:
            - tx_count: Frames transmitted
            - rx_count: Valid frames received
            - crc_errors: CRC validation failures
            - frame_errors: Frame format errors
        """
        return {
            'tx_count': self.tx_count,
            'rx_count': self.rx_count,
            'crc_errors': self.crc_errors,
            'frame_errors': self.frame_errors
        }

    def reset_statistics(self):
        """Reset communication statistics."""
        self.tx_count = 0
        self.rx_count = 0
        self.crc_errors = 0
        self.frame_errors = 0


# ============================================
# Utility Functions
# ============================================

def print_frame(frame: bytes, title: str = "Frame"):
    """
    Print frame contents in hex format.

    Args:
        frame: Frame bytes to print
        title: Title string
    """
    print(f"\n{title} ({len(frame)} bytes):")
    print("  " + " ".join(f"{b:02X}" for b in frame[:16]))
    print("  " + " ".join(f"{b:02X}" for b in frame[16:]))

    # Parse structure
    print(f"  Header:  0x{frame[0]:02X}")
    print(f"  Command: 0x{frame[1]:02X}")
    print(f"  Length:  {frame[2]}")
    print(f"  CRC:     0x{frame[29]:02X}{frame[30]:02X}")
    print(f"  Footer:  0x{frame[31]:02X}")


def print_response(response: Dict[str, Any]):
    """
    Print parsed response in readable format.

    Args:
        response: Parsed response dictionary
    """
    if not response['valid']:
        print(f"[ERROR] {response['error']}")
        return

    cmd = response['command']
    if cmd == RESP_ACK_OK:
        print("[OK] Command acknowledged")
    elif cmd == RESP_ACK_ERROR:
        data = response['data']
        print(f"[ERROR] {data['error_name']} (code: 0x{data['error_code']:02X})")
    elif cmd == RESP_DATA:
        data = response['data']
        if data['type'] == 'status':
            print(f"[STATUS] Mode: {data['mode_name']}")
            print(f"         Steering: {data['steering_deg']:.1f} deg")
            print(f"         Throttle: {data['throttle_percent']:.1f} %")
            print(f"         Reversing: {data['is_reversing']}")
            print(f"         Safety Stop: {data['safety_stop']}")
            print(f"         Loop Counter: {data['loop_counter']}")
        elif data['type'] == 'sensors':
            print(f"[SENSORS] SHARP Left:  {data['sharp_left_cm']:.1f} cm (valid: {data['sharp_left_valid']})")
            print(f"          SHARP Right: {data['sharp_right_cm']:.1f} cm (valid: {data['sharp_right_valid']})")
            print(f"          IMU Roll:  {data['roll_deg']:.1f} deg")
            print(f"          IMU Pitch: {data['pitch_deg']:.1f} deg")
            print(f"          IMU Yaw:   {data['yaw_deg']:.1f} deg")
            print(f"          IMU Valid: {data['imu_valid']}")
        else:
            print(f"[DATA] Raw: {data}")


# ============================================
# Module Test
# ============================================

if __name__ == "__main__":
    print("=" * 50)
    print("CoVAPSy SPI Library Test")
    print("=" * 50)

    # Test CRC calculation
    print("\n1. CRC-16/MODBUS Test:")
    test_data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])
    crc = calculate_crc16(test_data)
    expected = 0xC5CD
    print(f"   Input:    {test_data.hex()}")
    print(f"   CRC:      0x{crc:04X}")
    print(f"   Expected: 0x{expected:04X}")
    print(f"   Result:   {'PASS' if crc == expected else 'FAIL'}")

    # Test frame building
    print("\n2. Frame Building Test:")
    spi = CoVAPSySPI()

    # SET_CONTROL frame
    frame = spi.build_frame(CMD_SET_CONTROL, struct.pack('>ff', 15.0, 30.0))
    print_frame(frame, "SET_CONTROL (15.0 deg, 30.0%)")

    # GET_STATUS frame
    frame = spi.build_frame(CMD_GET_STATUS)
    print_frame(frame, "GET_STATUS")

    # Test connection (simulation mode if not on Pi)
    print("\n3. Connection Test:")
    if spi.open():
        print("   SPI connection opened successfully")

        # Test commands in simulation mode
        print("\n4. Command Tests (simulation mode):")

        result = spi.get_status()
        print("\n   GET_STATUS response:")
        print_response(result)

        result = spi.set_control(15.0, 30.0)
        print("\n   SET_CONTROL response:")
        print_response(result)

        # Statistics
        print("\n5. Statistics:")
        stats = spi.get_statistics()
        for key, value in stats.items():
            print(f"   {key}: {value}")

        spi.close()
    else:
        print("   Failed to open SPI connection")

    print("\n" + "=" * 50)
    print("Test completed")
    print("=" * 50)
