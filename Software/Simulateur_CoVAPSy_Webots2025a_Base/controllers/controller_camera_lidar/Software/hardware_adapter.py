# Hardware Adapter for Raspberry Pi
# Usage:
# 1. Implement the 'TODO' sections with real hardware drivers (e.g., rplidar, gpiozero).
# 2. In 'controller_camera_lidar.py', change imports:
#    FROM: from vehicle import Driver
#          from controller import Lidar, Camera
#    TO:   from hardware_adapter import Driver, Lidar, Camera

import time
import math

class Driver:
    def __init__(self):
        print("[HW] Initializing Driver (Motors/Servo)...")
        self.start_time = time.time()
        # TODO: Initialize GPIO, PWM for Motor and Servo
    
    def getBasicTimeStep(self):
        return 32 # ms
    
    def step(self):
        # Emulate Webots step
        # Return -1 to stop, 0 to continue
        time.sleep(0.032) 
        return 0
        
    def setSteeringAngle(self, angle_rad):
        # Input: Radians. -0.5 (Right) to +0.5 (Left)
        # Webots: Positive = Left, Negative = Right
        deg = math.degrees(angle_rad)
        # print(f"[HW] Steering: {deg:.1f} deg")
        # TODO: Map to Servo PWM
        pass
        
    def setCruisingSpeed(self, speed_kmh):
        # Input: km/h
        # print(f"[HW] Speed: {speed_kmh:.1f} km/h")
        # TODO: Map to Motor PWM
        pass
        
    def getTime(self):
        return time.time() - self.start_time

class Lidar:
    def __init__(self, name="RpLidarA2"):
        print(f"[HW] Initializing Lidar: {name}")
        # TODO: Initialize RPLidar (e.g., using adafruit_rplidar)
        
    def enable(self, timestep):
        pass
        
    def enablePointCloud(self):
        pass
        
    def getRangeImage(self):
        # MUST RETURN: List of 360 floats (meters)
        # Index 0 = Front
        # Index 90 = Left
        # Index 270 (-90) = Right
        # TODO: Read 360 scan from Lidar, fill array
        # Ensure correct orientation!
        return [2.0] * 360 # Dummy data (Open space)

class Camera:
    def __init__(self, name="camera"):
        print(f"[HW] Initializing Camera: {name}")
        # TODO: Initialize Camera (cv2.VideoCapture)
        
    def enable(self, timestep):
        pass
        
    def getWidth(self):
        return 640 # Adjust resolution
        
    def getHeight(self):
        return 480
        
    def getImage(self):
        # Webots returns raw bytes (BGRA). 
        # OpenCV usually reads BGR.
        # Controller expects bytes.
        # TODO: return frame.tobytes() or similar
        return None
