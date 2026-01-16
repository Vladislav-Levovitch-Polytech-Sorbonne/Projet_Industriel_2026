#!/usr/bin/env python3
"""
CoVAPSy Interactive SPI Test Tool
=================================

Interactive command-line tool for testing SPI communication
with the CoVAPSy autonomous vehicle.

Commands:
    control <steering> <throttle>  - Set steering and throttle
    status                         - Get vehicle status
    sensors                        - Get sensor data
    mode <mode>                    - Set vehicle mode (idle/remote/manual)
    stop                           - Emergency stop
    heartbeat                      - Send heartbeat
    loop <hz>                      - Start continuous mode at specified Hz
    stats                          - Show communication statistics
    help                           - Show help message
    quit                           - Exit program

Author: CoVAPSy Team
Date: 2026-01-16
"""

import sys
import time
import threading
from typing import Optional

from covapsy_spi import (
    CoVAPSySPI,
    print_response,
    print_frame,
    MODE_IDLE, MODE_REMOTE, MODE_MANUAL, MODE_EMERGENCY,
    MODE_NAMES
)


class InteractiveTest:
    """Interactive SPI test controller."""

    def __init__(self):
        self.spi = CoVAPSySPI()
        self.running = True
        self.loop_thread: Optional[threading.Thread] = None
        self.loop_running = False
        self.loop_hz = 20
        self.loop_steering = 0.0
        self.loop_throttle = 0.0

    def start(self):
        """Start interactive test session."""
        print("=" * 60)
        print("  CoVAPSy Interactive SPI Test Tool")
        print("=" * 60)
        print()

        # Open SPI connection
        if not self.spi.open():
            print("[ERROR] Failed to open SPI connection")
            return

        print("Type 'help' for available commands")
        print()

        # Main loop
        while self.running:
            try:
                # Get user input
                cmd_input = input("> ").strip()
                if not cmd_input:
                    continue

                # Parse and execute command
                self.execute_command(cmd_input)

            except KeyboardInterrupt:
                print("\n[INFO] Interrupted by user")
                self.stop_loop()
                break
            except EOFError:
                break

        # Cleanup
        self.stop_loop()
        self.spi.close()
        print("\n[INFO] Session ended")

    def execute_command(self, cmd_input: str):
        """
        Parse and execute a command.

        Args:
            cmd_input: Command string from user
        """
        parts = cmd_input.lower().split()
        cmd = parts[0]
        args = parts[1:]

        try:
            if cmd == 'help' or cmd == '?':
                self.cmd_help()
            elif cmd == 'quit' or cmd == 'exit' or cmd == 'q':
                self.running = False
            elif cmd == 'control' or cmd == 'c':
                self.cmd_control(args)
            elif cmd == 'status' or cmd == 's':
                self.cmd_status()
            elif cmd == 'sensors':
                self.cmd_sensors()
            elif cmd == 'mode' or cmd == 'm':
                self.cmd_mode(args)
            elif cmd == 'stop':
                self.cmd_stop()
            elif cmd == 'heartbeat' or cmd == 'hb':
                self.cmd_heartbeat()
            elif cmd == 'loop' or cmd == 'l':
                self.cmd_loop(args)
            elif cmd == 'stoploop' or cmd == 'sl':
                self.stop_loop()
            elif cmd == 'stats':
                self.cmd_stats()
            elif cmd == 'frame':
                self.cmd_frame(args)
            else:
                print(f"[ERROR] Unknown command: {cmd}")
                print("        Type 'help' for available commands")

        except Exception as e:
            print(f"[ERROR] Command failed: {e}")

    def cmd_help(self):
        """Show help message."""
        print("""
Available Commands:
===================

Vehicle Control:
  control <steering> <throttle>  - Set steering (-30~+30 deg) and throttle (-100~+100 %)
                                   Alias: c
                                   Example: control 15.0 30.0

  stop                           - Emergency stop (steering=0, throttle=0)

  mode <mode>                    - Set vehicle mode
                                   Alias: m
                                   Modes: idle (0), remote (1), manual (2)
                                   Example: mode remote

Data Queries:
  status                         - Get vehicle status (mode, steering, throttle, etc.)
                                   Alias: s

  sensors                        - Get sensor data (SHARP distances, IMU angles)

Communication:
  heartbeat                      - Send heartbeat to reset watchdog timer
                                   Alias: hb

  loop <hz>                      - Start continuous command loop at specified frequency
                                   Alias: l
                                   Example: loop 20 (sends heartbeat at 20Hz)

  stoploop                       - Stop continuous loop
                                   Alias: sl

  stats                          - Show communication statistics

Debug:
  frame <cmd> [payload_hex]      - Build and show raw frame
                                   Example: frame 02 41700000 42340000

General:
  help                           - Show this help message
  quit                           - Exit program (Alias: q, exit)

Notes:
  - In REMOTE mode, send commands at least every 500ms to prevent watchdog timeout
  - Use 'loop 20' to maintain 20Hz command rate automatically
  - Emergency stop can be triggered anytime with 'stop' command
""")

    def cmd_control(self, args: list):
        """Set steering and throttle."""
        if len(args) < 2:
            print("Usage: control <steering> <throttle>")
            print("       steering: -30 to +30 degrees")
            print("       throttle: -100 to +100 percent")
            return

        try:
            steering = float(args[0])
            throttle = float(args[1])
        except ValueError:
            print("[ERROR] Invalid number format")
            return

        print(f"[CMD] SET_CONTROL: steering={steering:.1f} deg, throttle={throttle:.1f} %")
        result = self.spi.set_control(steering, throttle)
        print_response(result)

        # Update loop values if loop is running
        if self.loop_running:
            self.loop_steering = steering
            self.loop_throttle = throttle
            print(f"[LOOP] Updated control values")

    def cmd_status(self):
        """Get vehicle status."""
        print("[CMD] GET_STATUS")
        result = self.spi.get_status()
        print_response(result)

    def cmd_sensors(self):
        """Get sensor data."""
        print("[CMD] GET_SENSORS")
        result = self.spi.get_sensors()
        print_response(result)

    def cmd_mode(self, args: list):
        """Set vehicle mode."""
        if len(args) < 1:
            print("Usage: mode <mode>")
            print("       Modes: idle (0), remote (1), manual (2)")
            return

        mode_str = args[0].lower()
        mode_map = {
            'idle': MODE_IDLE, '0': MODE_IDLE,
            'remote': MODE_REMOTE, '1': MODE_REMOTE,
            'manual': MODE_MANUAL, '2': MODE_MANUAL,
            'emergency': MODE_EMERGENCY, '3': MODE_EMERGENCY
        }

        if mode_str not in mode_map:
            print(f"[ERROR] Unknown mode: {mode_str}")
            print("        Valid modes: idle, remote, manual")
            return

        mode = mode_map[mode_str]
        print(f"[CMD] SET_MODE: {MODE_NAMES[mode]}")
        result = self.spi.set_mode(mode)
        print_response(result)

    def cmd_stop(self):
        """Emergency stop."""
        print("[CMD] EMERGENCY_STOP")
        self.stop_loop()  # Stop any running loop
        result = self.spi.emergency_stop()
        print_response(result)
        print("[WARNING] Vehicle emergency stopped!")

    def cmd_heartbeat(self):
        """Send heartbeat."""
        print("[CMD] HEARTBEAT")
        result = self.spi.heartbeat()
        print_response(result)

    def cmd_loop(self, args: list):
        """Start continuous command loop."""
        if self.loop_running:
            print("[INFO] Loop already running. Use 'stoploop' to stop first.")
            return

        hz = 20  # Default frequency
        if args:
            try:
                hz = int(args[0])
                if hz < 1 or hz > 100:
                    print("[ERROR] Frequency must be between 1 and 100 Hz")
                    return
            except ValueError:
                print("[ERROR] Invalid frequency")
                return

        self.loop_hz = hz
        self.loop_running = True
        self.loop_thread = threading.Thread(target=self._loop_worker, daemon=True)
        self.loop_thread.start()
        print(f"[LOOP] Started at {hz} Hz")
        print(f"[LOOP] Current control: steering={self.loop_steering:.1f}, throttle={self.loop_throttle:.1f}")
        print("[LOOP] Use 'control' to update values, 'stoploop' to stop")

    def stop_loop(self):
        """Stop continuous loop."""
        if self.loop_running:
            self.loop_running = False
            if self.loop_thread:
                self.loop_thread.join(timeout=1.0)
            print("[LOOP] Stopped")

    def _loop_worker(self):
        """Background loop worker thread.

        Uses set_control_fast() for higher throughput - accepts that
        responses are delayed by one frame (SPI full-duplex characteristic).
        """
        interval = 1.0 / self.loop_hz
        count = 0
        valid_count = 0
        start_time = time.time()

        while self.loop_running:
            try:
                # Send control command with current values (fast mode - no fetch)
                # Response is for PREVIOUS command, which is fine for continuous mode
                result = self.spi.set_control_fast(self.loop_steering, self.loop_throttle)
                count += 1

                # Count valid responses
                if result.get('valid', False):
                    valid_count += 1

                # Print status every second
                elapsed = time.time() - start_time
                if elapsed >= 1.0:
                    actual_hz = count / elapsed
                    stats = self.spi.get_statistics()
                    print(f"\r[LOOP] {actual_hz:.1f} Hz | TX:{stats['tx_count']} RX:{valid_count} ERR:{stats['frame_errors']}    ", end='', flush=True)
                    count = 0
                    valid_count = 0
                    start_time = time.time()

                # Wait for next interval
                time.sleep(interval)

            except Exception as e:
                print(f"\n[LOOP ERROR] {e}")
                break

        print()  # New line after loop status

    def cmd_stats(self):
        """Show communication statistics."""
        stats = self.spi.get_statistics()
        print("\nCommunication Statistics:")
        print(f"  TX Frames:    {stats['tx_count']}")
        print(f"  RX Frames:    {stats['rx_count']}")
        print(f"  CRC Errors:   {stats['crc_errors']}")
        print(f"  Frame Errors: {stats['frame_errors']}")

        if stats['tx_count'] > 0:
            error_rate = (stats['crc_errors'] + stats['frame_errors']) / stats['tx_count'] * 100
            print(f"  Error Rate:   {error_rate:.2f}%")

    def cmd_frame(self, args: list):
        """Build and display a raw frame."""
        if len(args) < 1:
            print("Usage: frame <cmd_hex> [payload_hex]")
            print("       Example: frame 02 41700000 42340000")
            return

        try:
            cmd = int(args[0], 16)
            payload = bytes()
            if len(args) > 1:
                payload = bytes.fromhex(''.join(args[1:]))

            frame = self.spi.build_frame(cmd, payload)
            print_frame(frame, f"Frame (cmd=0x{cmd:02X})")

        except ValueError as e:
            print(f"[ERROR] Invalid hex value: {e}")


def main():
    """Main entry point."""
    test = InteractiveTest()
    test.start()


if __name__ == "__main__":
    main()
