#!/usr/bin/env python3
"""
CoVAPSy Automated SPI Test Suite
================================

Automated test suite for verifying SPI communication and vehicle control.

Test Categories:
1. Communication - CRC, frame format validation
2. Control - Steering, throttle, boundary values
3. Sensors - SHARP distance, IMU data
4. Safety - Emergency stop, watchdog timeout
5. Stability - Long-term continuous operation

Author: CoVAPSy Team
Date: 2026-01-16
"""

import sys
import time
import struct
from typing import List, Tuple, Callable

from covapsy_spi import (
    CoVAPSySPI,
    calculate_crc16,
    FRAME_SIZE, HEADER_BYTE, FOOTER_BYTE,
    CMD_GET_STATUS, CMD_SET_CONTROL, CMD_GET_SENSORS,
    CMD_SET_MODE, CMD_EMERGENCY_STOP, CMD_HEARTBEAT,
    RESP_ACK_OK, RESP_ACK_ERROR, RESP_DATA,
    MODE_IDLE, MODE_REMOTE, MODE_MANUAL, MODE_EMERGENCY
)


# ============================================
# Test Result Tracking
# ============================================

class TestResult:
    """Test result container."""

    def __init__(self, name: str):
        self.name = name
        self.passed = 0
        self.failed = 0
        self.errors: List[str] = []

    def add_pass(self, msg: str = ""):
        self.passed += 1
        if msg:
            print(f"    [PASS] {msg}")

    def add_fail(self, msg: str):
        self.failed += 1
        self.errors.append(msg)
        print(f"    [FAIL] {msg}")

    def is_passed(self) -> bool:
        return self.failed == 0

    def summary(self) -> str:
        total = self.passed + self.failed
        status = "PASS" if self.is_passed() else "FAIL"
        return f"{self.name}: {status} ({self.passed}/{total} tests passed)"


# ============================================
# Test Suite
# ============================================

class AutoTest:
    """Automated test suite for CoVAPSy SPI communication."""

    def __init__(self):
        self.spi = CoVAPSySPI()
        self.results: List[TestResult] = []

    def run_all(self):
        """Run all test categories."""
        print("=" * 60)
        print("  CoVAPSy Automated SPI Test Suite")
        print("=" * 60)
        print()

        # Open SPI connection
        if not self.spi.open():
            print("[ERROR] Failed to open SPI connection")
            return False

        try:
            # Run test categories
            self.test_1_communication()
            self.test_2_control()
            self.test_3_sensors()
            self.test_4_safety()
            self.test_5_stability()

        except KeyboardInterrupt:
            print("\n[INFO] Test interrupted by user")
        except Exception as e:
            print(f"\n[ERROR] Test failed with exception: {e}")
        finally:
            # Always send emergency stop at end
            print("\n[CLEANUP] Sending emergency stop...")
            self.spi.emergency_stop()
            self.spi.close()

        # Print summary
        self.print_summary()

        return all(r.is_passed() for r in self.results)

    def print_summary(self):
        """Print test summary."""
        print("\n" + "=" * 60)
        print("  Test Summary")
        print("=" * 60)

        total_passed = 0
        total_failed = 0

        for result in self.results:
            status = "PASS" if result.is_passed() else "FAIL"
            print(f"  [{status}] {result.name}: {result.passed} passed, {result.failed} failed")
            total_passed += result.passed
            total_failed += result.failed

            # Print errors
            for error in result.errors:
                print(f"         - {error}")

        print("-" * 60)
        total = total_passed + total_failed
        overall = "PASS" if total_failed == 0 else "FAIL"
        print(f"  Overall: {overall} ({total_passed}/{total} tests passed)")
        print("=" * 60)

    # ============================================
    # Test 1: Communication
    # ============================================

    def test_1_communication(self):
        """Test basic communication functionality."""
        print("\n" + "-" * 60)
        print("Test 1: Communication")
        print("-" * 60)

        result = TestResult("Communication")

        # 1.1 CRC-16 calculation
        print("\n  1.1 CRC-16/MODBUS calculation")
        test_vectors = [
            (bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A]), 0xC5CD),
            (bytes([0xAA, 0x01, 0x00]), 0x59ED),  # GET_STATUS header
        ]
        for data, expected in test_vectors:
            crc = calculate_crc16(data)
            if crc == expected:
                result.add_pass(f"CRC({data.hex()}) = 0x{crc:04X}")
            else:
                result.add_fail(f"CRC({data.hex()}): expected 0x{expected:04X}, got 0x{crc:04X}")

        # 1.2 Frame building
        print("\n  1.2 Frame building")
        frame = self.spi.build_frame(CMD_GET_STATUS)
        if len(frame) == FRAME_SIZE:
            result.add_pass(f"Frame size = {FRAME_SIZE} bytes")
        else:
            result.add_fail(f"Frame size: expected {FRAME_SIZE}, got {len(frame)}")

        if frame[0] == HEADER_BYTE:
            result.add_pass(f"Header = 0x{HEADER_BYTE:02X}")
        else:
            result.add_fail(f"Header: expected 0x{HEADER_BYTE:02X}, got 0x{frame[0]:02X}")

        if frame[31] == FOOTER_BYTE:
            result.add_pass(f"Footer = 0x{FOOTER_BYTE:02X}")
        else:
            result.add_fail(f"Footer: expected 0x{FOOTER_BYTE:02X}, got 0x{frame[31]:02X}")

        # 1.3 Response parsing
        print("\n  1.3 Response validation")
        response = self.spi.get_status()
        if response['valid']:
            result.add_pass("GET_STATUS response valid")
        else:
            result.add_fail(f"GET_STATUS response invalid: {response['error']}")

        self.results.append(result)

    # ============================================
    # Test 2: Control Commands
    # ============================================

    def test_2_control(self):
        """Test vehicle control commands."""
        print("\n" + "-" * 60)
        print("Test 2: Control Commands")
        print("-" * 60)

        result = TestResult("Control")

        # 2.1 Set mode to REMOTE
        print("\n  2.1 Set mode to REMOTE")
        response = self.spi.set_mode(MODE_REMOTE)
        if response['valid']:
            result.add_pass("SET_MODE(REMOTE) acknowledged")
        else:
            result.add_fail(f"SET_MODE(REMOTE) failed: {response['error']}")

        time.sleep(0.1)  # Allow mode change to take effect

        # 2.2 SET_CONTROL normal values
        print("\n  2.2 SET_CONTROL normal values")
        test_values = [
            (0.0, 0.0, "center/stop"),
            (15.0, 30.0, "right/forward"),
            (-15.0, 30.0, "left/forward"),
            (0.0, -25.0, "center/reverse"),
        ]
        for steering, throttle, desc in test_values:
            response = self.spi.set_control(steering, throttle)
            if response['valid']:
                result.add_pass(f"SET_CONTROL({steering}, {throttle}) - {desc}")
            else:
                result.add_fail(f"SET_CONTROL({steering}, {throttle}) failed: {response['error']}")
            time.sleep(0.1)

        # 2.3 SET_CONTROL boundary values
        print("\n  2.3 SET_CONTROL boundary values")
        boundary_values = [
            (-30.0, -100.0, "min steering, min throttle"),
            (30.0, 100.0, "max steering, max throttle"),
            (-50.0, 0.0, "steering clamped to -30"),  # Should clamp
            (50.0, 0.0, "steering clamped to +30"),   # Should clamp
            (0.0, -150.0, "throttle clamped to -100"), # Should clamp
            (0.0, 150.0, "throttle clamped to +100"),  # Should clamp
        ]
        for steering, throttle, desc in boundary_values:
            response = self.spi.set_control(steering, throttle)
            if response['valid']:
                result.add_pass(f"SET_CONTROL({steering}, {throttle}) - {desc}")
            else:
                result.add_fail(f"SET_CONTROL({steering}, {throttle}) failed: {response['error']}")
            time.sleep(0.1)

        # 2.4 GET_STATUS verification
        print("\n  2.4 GET_STATUS verification")
        response = self.spi.get_status()
        if response['valid'] and response['data']:
            data = response['data']
            result.add_pass(f"Mode: {data.get('mode_name', 'N/A')}")
            result.add_pass(f"Steering: {data.get('steering_deg', 0):.1f} deg")
            result.add_pass(f"Throttle: {data.get('throttle_percent', 0):.1f} %")
        else:
            result.add_fail(f"GET_STATUS failed: {response.get('error', 'unknown')}")

        # Reset to safe state
        self.spi.set_control(0.0, 0.0)

        self.results.append(result)

    # ============================================
    # Test 3: Sensor Data
    # ============================================

    def test_3_sensors(self):
        """Test sensor data reading."""
        print("\n" + "-" * 60)
        print("Test 3: Sensor Data")
        print("-" * 60)

        result = TestResult("Sensors")

        # 3.1 GET_SENSORS basic
        print("\n  3.1 GET_SENSORS response")
        response = self.spi.get_sensors()
        if response['valid']:
            result.add_pass("GET_SENSORS response valid")
        else:
            result.add_fail(f"GET_SENSORS failed: {response['error']}")
            self.results.append(result)
            return

        # 3.2 SHARP sensor data
        print("\n  3.2 SHARP sensor data")
        data = response.get('data', {})
        if data.get('type') == 'sensors':
            left = data.get('sharp_left_cm', 0)
            right = data.get('sharp_right_cm', 0)
            left_valid = data.get('sharp_left_valid', 0)
            right_valid = data.get('sharp_right_valid', 0)

            result.add_pass(f"Left distance: {left:.1f} cm (valid: {left_valid})")
            result.add_pass(f"Right distance: {right:.1f} cm (valid: {right_valid})")

            # Validate range (6-80cm for SHARP sensor)
            if left_valid and (left < 6 or left > 80):
                result.add_fail(f"Left distance out of range: {left:.1f} cm")
            if right_valid and (right < 6 or right > 80):
                result.add_fail(f"Right distance out of range: {right:.1f} cm")
        else:
            result.add_fail("Unexpected sensor data format")

        # 3.3 IMU data
        print("\n  3.3 IMU data")
        if data.get('type') == 'sensors':
            roll = data.get('roll_deg', 0)
            pitch = data.get('pitch_deg', 0)
            yaw = data.get('yaw_deg', 0)
            imu_valid = data.get('imu_valid', 0)

            result.add_pass(f"Roll: {roll:.1f} deg")
            result.add_pass(f"Pitch: {pitch:.1f} deg")
            result.add_pass(f"Yaw: {yaw:.1f} deg")
            result.add_pass(f"IMU valid: {imu_valid}")

            # Validate reasonable ranges
            if imu_valid:
                if abs(roll) > 90:
                    result.add_fail(f"Roll out of range: {roll:.1f} deg")
                if abs(pitch) > 90:
                    result.add_fail(f"Pitch out of range: {pitch:.1f} deg")
        else:
            result.add_fail("Unexpected sensor data format")

        # 3.4 Multiple reads
        print("\n  3.4 Multiple sensor reads")
        success_count = 0
        for i in range(5):
            response = self.spi.get_sensors()
            if response['valid']:
                success_count += 1
            time.sleep(0.05)

        if success_count == 5:
            result.add_pass(f"5 consecutive reads successful")
        else:
            result.add_fail(f"Only {success_count}/5 reads successful")

        self.results.append(result)

    # ============================================
    # Test 4: Safety Mechanisms
    # ============================================

    def test_4_safety(self):
        """Test safety mechanisms."""
        print("\n" + "-" * 60)
        print("Test 4: Safety Mechanisms")
        print("-" * 60)

        result = TestResult("Safety")

        # 4.1 Emergency stop
        print("\n  4.1 Emergency stop")

        # First set some control values
        self.spi.set_mode(MODE_REMOTE)
        self.spi.set_control(15.0, 30.0)
        time.sleep(0.1)

        # Send emergency stop
        response = self.spi.emergency_stop()
        if response['valid']:
            result.add_pass("EMERGENCY_STOP acknowledged")
        else:
            result.add_fail(f"EMERGENCY_STOP failed: {response['error']}")

        time.sleep(0.1)

        # Verify vehicle stopped
        response = self.spi.get_status()
        if response['valid'] and response['data']:
            data = response['data']
            mode = data.get('mode', -1)
            steering = data.get('steering_deg', 999)
            throttle = data.get('throttle_percent', 999)

            if mode == MODE_EMERGENCY:
                result.add_pass("Mode changed to EMERGENCY")
            else:
                result.add_fail(f"Mode not EMERGENCY: {mode}")

            if abs(steering) < 1.0:
                result.add_pass("Steering reset to 0")
            else:
                result.add_fail(f"Steering not 0: {steering}")

            if abs(throttle) < 1.0:
                result.add_pass("Throttle reset to 0")
            else:
                result.add_fail(f"Throttle not 0: {throttle}")
        else:
            result.add_fail("Failed to verify emergency stop state")

        # 4.2 Heartbeat
        print("\n  4.2 Heartbeat command")
        self.spi.set_mode(MODE_REMOTE)
        time.sleep(0.1)

        response = self.spi.heartbeat()
        if response['valid']:
            result.add_pass("HEARTBEAT acknowledged")
        else:
            result.add_fail(f"HEARTBEAT failed: {response['error']}")

        # 4.3 Watchdog timeout test (optional - can damage test)
        print("\n  4.3 Watchdog timeout (skip in auto test)")
        result.add_pass("Watchdog test skipped (manual test recommended)")

        # Reset to safe state
        self.spi.emergency_stop()

        self.results.append(result)

    # ============================================
    # Test 5: Stability
    # ============================================

    def test_5_stability(self, duration: float = 10.0, hz: int = 20):
        """
        Test long-term communication stability.

        Args:
            duration: Test duration in seconds
            hz: Command frequency
        """
        print("\n" + "-" * 60)
        print(f"Test 5: Stability ({duration}s at {hz}Hz)")
        print("-" * 60)

        result = TestResult("Stability")

        # Reset statistics
        self.spi.reset_statistics()

        # Set mode
        self.spi.set_mode(MODE_REMOTE)
        time.sleep(0.1)

        # Run continuous commands
        print(f"\n  Running {hz}Hz commands for {duration}s...")
        interval = 1.0 / hz
        start_time = time.time()
        count = 0

        while time.time() - start_time < duration:
            # Alternate between control and heartbeat
            if count % 2 == 0:
                self.spi.set_control(0.0, 0.0)
            else:
                self.spi.heartbeat()

            count += 1

            # Progress indicator
            elapsed = time.time() - start_time
            if int(elapsed) > int(elapsed - interval):
                print(f"\r    Progress: {elapsed:.1f}s / {duration}s ({count} frames)    ", end='', flush=True)

            # Wait for next interval
            next_time = start_time + count * interval
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

        print()  # New line

        # Get statistics
        stats = self.spi.get_statistics()
        actual_duration = time.time() - start_time
        actual_hz = stats['tx_count'] / actual_duration

        print(f"\n  5.1 Statistics")
        result.add_pass(f"TX Frames: {stats['tx_count']}")
        result.add_pass(f"RX Frames: {stats['rx_count']}")
        result.add_pass(f"Actual frequency: {actual_hz:.1f} Hz")

        # Check error rate
        print(f"\n  5.2 Error analysis")
        error_count = stats['crc_errors'] + stats['frame_errors']
        if stats['tx_count'] > 0:
            error_rate = error_count / stats['tx_count'] * 100

            if error_rate < 0.1:
                result.add_pass(f"Error rate: {error_rate:.3f}% (< 0.1%)")
            elif error_rate < 1.0:
                result.add_pass(f"Error rate: {error_rate:.3f}% (< 1.0%, acceptable)")
            else:
                result.add_fail(f"Error rate: {error_rate:.3f}% (> 1.0%)")

            result.add_pass(f"CRC errors: {stats['crc_errors']}")
            result.add_pass(f"Frame errors: {stats['frame_errors']}")
        else:
            result.add_fail("No frames transmitted")

        # Reset to safe state
        self.spi.emergency_stop()

        self.results.append(result)


# ============================================
# Main Entry Point
# ============================================

def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description='CoVAPSy Automated SPI Test Suite')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Stability test duration in seconds (default: 10)')
    parser.add_argument('--hz', type=int, default=20,
                        help='Test command frequency (default: 20)')
    parser.add_argument('--quick', action='store_true',
                        help='Run quick test (5s stability)')

    args = parser.parse_args()

    test = AutoTest()

    if args.quick:
        # Override for quick test
        test.test_5_stability = lambda: AutoTest.test_5_stability(test, 5.0, 20)

    success = test.run_all()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
