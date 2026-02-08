# Autonomous Car Controller - Hardware Porting Guide

This folder contains the software package for deployment on the Raspberry Pi.

## Files
- **`controller_camera_lidar.py`**: The main autonomous driving logic.
- **`hardware_adapter.py`**: A template file to interface with real sensors (Lidar, Camera, Motors).
- **`requirements.txt`**: Python dependencies.

## Instructions for Hardware Team

1.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

2.  **Implement Drivers**:
    - Open `hardware_adapter.py`.
    - Fill in the `TODO` sections in the `Driver`, `Lidar`, and `Camera` classes.
    - **Lidar**: Must return an array of 360 distances in mm (Index 0 = Front, 90 = Left).
    - **Camera**: Must return a raw image frame (OpenCV format).
    - **Driver**: Map steering (-0.5 to +0.5 rad) and speed (km/h) to your motor controller PWM.

3.  **Update Main Controller**:
    - Open `controller_camera_lidar.py`.
    - **Modify Imports** (Lines 5-6):
      ```python
      # Replace these lines:
      # from vehicle import Driver
      # from controller import Lidar, Camera

      # With this:
      from hardware_adapter import Driver, Lidar, Camera
      ```
    - **Control Loop**:
      - Replace `while driver.step() != -1:` with a standard loop:
        ```python
        while True:
            # logic...
            driver.step() # Handles sleep/timing
        ```

4.  **Run**:
    ```bash
    python controller_camera_lidar.py
    ```
