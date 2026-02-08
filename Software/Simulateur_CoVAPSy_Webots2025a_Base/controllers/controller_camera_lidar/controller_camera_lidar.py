# Copyright 2024 Antigravity for CoVAPSy
# Controller combining Lidar (Safety/Gap) and Camera (OpenCV Optimization)
# Optimized based on controller_jaune, controller_violet, and PDF recommendations

from vehicle import Driver
from controller import Lidar, Camera
import numpy as np
import cv2

# --- Constants ---
# --- Constants ---
MAX_SPEED_KMH = 28
MAX_ANGLE_DEGRE = 16
CRASH_DISTANCE_MM = 350  # Increased for earlier detection
# Physical Car Dimensions relative to Lidar
LIDAR_FRONT_OFFSET = 250
LIDAR_SIDE_OFFSET = 200
TURN_ANTICIPATION_MM = 900  # Moderate increase for slightly earlier turning
WIDE_CORNER_THRESHOLD = 0.38  # Balanced sensitivity
WRONG_WAY_THRESHOLD = 0.10  # Threshold for wrong direction detection (lowered)
MIN_COLOR_PIXELS = 800  # Minimum pixels to consider color detected

# --- Initialization ---
driver = Driver()
basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Sensors
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

camera = Camera("camera")
camera.enable(sensorTimeStep)

# --- Helper Functions ---

def get_lidar_array_mm(lidar_device):
    """
    Convert Lidar data to mm array with proper indexing.
    Based on controller_jaune.py indexing: tableau_lidar_mm[i-180]
    Index 0 = Front, positive = left, negative = right
    """
    donnees_lidar_brutes = lidar_device.getRangeImage()
    if not donnees_lidar_brutes:
        return [0] * 360
    
    tableau_lidar_mm = [0] * 360
    
    # Convert to mm and remap indices (from controller_jaune.py)
    for i in range(360):
        if (donnees_lidar_brutes[-i] > 0) and (donnees_lidar_brutes[-i] < 20):
            tableau_lidar_mm[i-180] = 1000 * donnees_lidar_brutes[-i]
        else:
            tableau_lidar_mm[i-180] = 0
    
    return tableau_lidar_mm

def process_camera_opencv(camera_device):
    """
    Uses OpenCV to detect wall colors and find drivable path.
    Returns: 
    - vision_error: steering correction (-1.0 to 1.0)
    - wall_color: -1 (Red/Left), 1 (Green/Right), 0 (None)
    - red_left_ratio: ratio of red pixels on left side
    - green_right_ratio: ratio of green pixels on right side
    """
    img_data = camera_device.getImage()
    if img_data is None:
        return 0.0, 0, 0.0, 0.0
    
    width = camera_device.getWidth()
    height = camera_device.getHeight()
    
    # Webots BGRA -> OpenCV BGR
    img_arr = np.frombuffer(img_data, dtype=np.uint8).reshape((height, width, 4))
    img_bgr = img_arr[:, :, :3]
    
    # Resize for performance
    target_w = 320
    target_h = int(height * (target_w / width))
    img_small = cv2.resize(img_bgr, (target_w, target_h))
    
    # ROI: Look ahead at road
    roi = img_small[int(target_h*0.4):int(target_h*0.9), :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Detect Green (Right Wall) - RAL 6037
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([90, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    # Detect Red (Left Wall) - RAL 3020
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    
    # Split into left and right halves for direction checking
    roi_h, roi_w = roi.shape[:2]
    mid_x = roi_w // 2
    
    mask_red_left = mask_red[:, :mid_x]
    mask_red_right = mask_red[:, mid_x:]
    mask_green_left = mask_green[:, :mid_x]
    mask_green_right = mask_green[:, mid_x:]
    
    # Calculate ratios for direction validation
    total_pixels_half = (roi_h * mid_x)
    red_left_ratio = cv2.countNonZero(mask_red_left) / total_pixels_half
    green_right_ratio = cv2.countNonZero(mask_green_right) / total_pixels_half
    
    # Determine dominant wall color (overall)
    red_pixels = cv2.countNonZero(mask_red)
    green_pixels = cv2.countNonZero(mask_green)
    wall_side = 0
    if red_pixels > 500 and red_pixels > green_pixels:
        wall_side = -1  # Red/Left
    elif green_pixels > 500 and green_pixels > red_pixels:
        wall_side = 1  # Green/Right
    
    # Find drivable path (not wall)
    mask_walls = mask_red | mask_green
    mask_road = cv2.bitwise_not(mask_walls)
    
    # Calculate road center
    M = cv2.moments(mask_road)
    if M["m00"] > 0:
        cX = int(M["m10"] / M["m00"])
        center_x = target_w // 2
        # Positive error = road is right, need to steer right
        vision_error = (cX - center_x) / center_x
    else:
        vision_error = 0.0
    
    return vision_error, wall_side, red_left_ratio, green_right_ratio

def set_vitesse_m_s(vitesse_m_s):
    """Convert m/s to km/h and set speed (from PDF and controller_jaune)"""
    speed = vitesse_m_s * 3.6
    if speed > MAX_SPEED_KMH:
        speed = MAX_SPEED_KMH
    if speed < 0:
        speed = 0
    driver.setCruisingSpeed(speed)

def set_direction_degre(angle_degre):
    """Set steering angle in degrees (from PDF and controller_jaune)"""
    if angle_degre > MAX_ANGLE_DEGRE:
        angle_degre = MAX_ANGLE_DEGRE
    elif angle_degre < -MAX_ANGLE_DEGRE:
        angle_degre = -MAX_ANGLE_DEGRE
    # Convert to radians (negative because of Webots convention)
    angle = -angle_degre * 3.14 / 180
    driver.setSteeringAngle(angle)

def recule():
    """Reverse at increased speed (3 km/h) for efficiency"""
    driver.setCruisingSpeed(-3)

# --- Main Loop ---
print("Antigravity Controller: Optimized Lidar + OpenCV")
print("Based on controller_jaune, controller_violet, and PDF recommendations")
driver.setSteeringAngle(0)
driver.setCruisingSpeed(0)

# State machine
STATE_FORWARD = 0
STATE_REVERSE = 1
STATE_UTURN = 2  # New state for wrong direction
current_state = STATE_FORWARD
reverse_timer = 0.0
reverse_angle = 0.0
uturn_stage = 0  # 0: initial reverse, 1: turn, 2: forward
special_corner_2_mode = None # None, 'left', 'right' - Sticky state for wide corners
STATE_AVOIDANCE_RECOVERY = 3 # New- [x] Refine Obstacle Avoidance (Maneuver: Straight Reverse -> Forward Turn Away). <!-- id: 12 -->
# - [x] Refine Collision Recovery (Min 650mm reverse, Opposite Forward Steering). <!-- id: 13 -->   
is_obstacle_avoidance = False
recovery_steering_angle = 0.0
recovery_timer = 0.0

while driver.step() != -1:
    
    # 1. Acquire sensor data
    tableau_lidar_mm = get_lidar_array_mm(lidar)
    vis_error, wall_color, red_left_ratio, green_right_ratio = process_camera_opencv(camera)
    
    # 2. Check for wrong direction (Red should be LEFT, Green should be RIGHT)
    # TEMPORARILY DISABLED - needs more testing
    is_wrong_direction = False
    
    # TODO: Fix direction detection logic
    # Current issue: ratios seem inverted
    # For now, skip this check entirely
    
    if False:  # Disabled
        # First, check if we can see both colors clearly
        roi_h, roi_w = 100, 320  # Approximate ROI size
        total_pixels = roi_h * roi_w
        
        # Calculate absolute pixel counts (approximate from ratios)
        red_left_pixels = red_left_ratio * (total_pixels / 2)
        green_right_pixels = green_right_ratio * (total_pixels / 2)
        red_right_pixels = (1.0 - red_left_ratio) * (total_pixels / 2)
        green_left_pixels = (1.0 - green_right_ratio) * (total_pixels / 2)
        
        # Only check direction if BOTH colors are visible
        total_red = red_left_pixels + red_right_pixels
        total_green = green_left_pixels + green_right_pixels
        
        if total_red > MIN_COLOR_PIXELS and total_green > MIN_COLOR_PIXELS:
            # Both colors visible, now check if they're in correct positions
            # CORRECT: Red mainly on LEFT, Green mainly on RIGHT
            # WRONG: Red mainly on RIGHT, Green mainly on LEFT
            
            if red_left_ratio > WRONG_WAY_THRESHOLD and green_right_ratio > WRONG_WAY_THRESHOLD:
                # Red on left AND green on right - CORRECT direction
                is_wrong_direction = False
            elif red_right_pixels > red_left_pixels and green_left_pixels > green_right_pixels:
                # Red primarily on right AND green primarily on left - WRONG direction
                if red_right_pixels > MIN_COLOR_PIXELS/2 and green_left_pixels > MIN_COLOR_PIXELS/2:
                    is_wrong_direction = True
                    if current_state == STATE_FORWARD:
                        print(f"⚠️ WRONG DIRECTION! RedL:{red_left_ratio:.2f} RedR:{1-red_left_ratio:.2f} GreenL:{1-green_right_ratio:.2f} GreenR:{green_right_ratio:.2f}")
                        current_state = STATE_UTURN
                        uturn_stage = 0
                        reverse_timer = driver.getTime() + 1.5
        # else: Only one color or no colors visible - don't check direction
    
    # 3. Obstacle detection (from PDF strategy)
    front_dist = tableau_lidar_mm[0]  # Front (0 degrees)
    left_60_dist = tableau_lidar_mm[60]  # Left 60 degrees
    right_60_dist = tableau_lidar_mm[-60]  # Right 60 degrees
    left_30_dist = tableau_lidar_mm[30]  # Left 30 degrees
    right_30_dist = tableau_lidar_mm[-30]  # Right 30 degrees
    left_90_dist = tableau_lidar_mm[90]  # Left 90 degrees (side)
    right_90_dist = tableau_lidar_mm[-90]  # Right 90 degrees (side)
    
    # 4. State transitions
    if current_state == STATE_UTURN:
        # Execute U-turn maneuver
        if driver.getTime() < reverse_timer:
            if uturn_stage == 0:
                # Stage 0: Reverse
                set_direction_degre(0)
                recule()
            elif uturn_stage == 1:
                # Stage 1: Turn sharply
                set_direction_degre(16)
                set_vitesse_m_s(0.5)
        else:
            if uturn_stage == 0:
                uturn_stage = 1
                reverse_timer = driver.getTime() + 2.0
            elif uturn_stage == 1:
                current_state = STATE_FORWARD
                print("✅ U-turn complete. Resuming forward.")
        continue
    
    elif current_state == STATE_FORWARD:
        # Check for obstacles (Combined check for immediate reaction)
        # Any of Front, Left 30, or Right 30 < CRASH_DISTANCE_MM triggers reverse
        
        crash_detected = False
        collision_source = "NONE"
        
        # Priority: Check if ANY sensor is too close
        if (front_dist > 0 and front_dist < CRASH_DISTANCE_MM):
            crash_detected = True
            collision_source = "FRONT"
        elif (left_30_dist > 0 and left_30_dist < CRASH_DISTANCE_MM):
            crash_detected = True
            collision_source = "LEFT_30"
        elif (right_30_dist > 0 and right_30_dist < CRASH_DISTANCE_MM):
            crash_detected = True
            collision_source = "RIGHT_30"
            
        if crash_detected:
            print(f"💥 COLLISION IMMINENT ({collision_source}) - SWITCHING TO REVERSE")
            if collision_source == "FRONT": print(f"Dist: {front_dist:.0f}mm - Color: {wall_color}")
            if collision_source == "LEFT_30": print(f"Dist: {left_30_dist:.0f}mm")
            if collision_source == "RIGHT_30": print(f"Dist: {right_30_dist:.0f}mm")
            
            current_state = STATE_REVERSE
            reverse_timer = driver.getTime() + 1.2  # Sufficient reverse time
            
            camera_priority = False
            
            # Common Reverse Settings
            # Distance 650mm. Speed 3 km/h (0.83 m/s). Time = 0.65/0.83 = 0.78s.
            # Use 1.0s for safely covering >650mm
            reverse_duration = 1.0
            reverse_timer = driver.getTime() + reverse_duration
            
            # --- DETERMINE REVERSE & RECOVERY ANGLES ---
            if wall_color == 1: # Green (Right Wall)
                 # Wall on Right -> Reverse RIGHT (Full Lock -16) to clear rear
                 # Forward Recovery -> Steer LEFT (Opposite, +16) to move front away
                 reverse_angle = -MAX_ANGLE_DEGRE
                 recovery_steering_angle = MAX_ANGLE_DEGRE 
                 print(" -> Color GREEN (Right Wall): Rev RIGHT (Max) -> Fwd LEFT (Max)")
                 
            elif wall_color == -1: # Red (Left Wall)
                 # Wall on Left -> Reverse LEFT (Full Lock +16)
                 # Forward Recovery -> Steer RIGHT (Opposite, -16)
                 reverse_angle = MAX_ANGLE_DEGRE
                 recovery_steering_angle = -MAX_ANGLE_DEGRE
                 print(" -> Color RED (Left Wall): Rev LEFT (Max) -> Fwd RIGHT (Max)")
                 
            else:
                 # OBSTACLE (No Color)
                 # Reverse STRAIGHT
                 reverse_angle = 0
                 
                 # Recovery: Steer AWAY from closer wall
                 print(" -> No Wall Color (Obstacle). Rev STRAIGHT.")
                 
                 safe_l90_chk = left_90_dist if left_90_dist > 0 else 5000
                 safe_r90_chk = right_90_dist if right_90_dist > 0 else 5000
                 
                 if safe_l90_chk < safe_r90_chk:
                     print(f" -> Left Wall Closer ({safe_l90_chk:.0f} < {safe_r90_chk:.0f}) -> Recov: Steer RIGHT (Max)")
                     recovery_steering_angle = -MAX_ANGLE_DEGRE
                 else:
                     print(f" -> Right Wall Closer ({safe_r90_chk:.0f} < {safe_l90_chk:.0f}) -> Recov: Steer LEFT (Max)")
                     recovery_steering_angle = MAX_ANGLE_DEGRE
            
            current_state = STATE_REVERSE
            set_direction_degre(reverse_angle)
            recule()
            continue
    
    elif current_state == STATE_REVERSE:
        if driver.getTime() > reverse_timer:
            # ALWAYS transition to AVOIDANCE_RECOVERY for Forward Correction
            current_state = STATE_AVOIDANCE_RECOVERY
            recovery_timer = driver.getTime() + 0.8 # ~400mm at 0.5 m/s
            print("🔄 REVERSE DONE. Starting RECOVERY MANEUVER (Forward + Steer).")
        else:
            # Continue reversing with steering
            continue
    
    elif current_state == STATE_AVOIDANCE_RECOVERY:
        # Move Forward and Steer Away
        if driver.getTime() > recovery_timer:
            current_state = STATE_FORWARD
            print("✅ AVOIDANCE MANEUVER COMPLETE. Resuming Normal Navigation.")
        else:
            set_vitesse_m_s(0.5) # Slow forward speed (1.8 km/h)
            set_direction_degre(recovery_steering_angle)
            continue
    
    # 5. Forward driving logic
    if current_state == STATE_FORWARD:
        # Detect wide corners (asymmetric corridor)
        left_avg = (left_60_dist + left_30_dist) / 2 if left_60_dist > 0 and left_30_dist > 0 else max(left_60_dist, left_30_dist)
        right_avg = (right_60_dist + right_30_dist) / 2 if right_60_dist > 0 and right_30_dist > 0 else max(right_60_dist, right_30_dist)
        
        # Check for wide corner condition
        is_wide_corner = False
        corner_direction = 0  # -1: left, +1: right
        
        if left_avg > 0 and right_avg > 0:
            asymmetry_ratio = abs(left_avg - right_avg) / max(left_avg, right_avg)
            if asymmetry_ratio > WIDE_CORNER_THRESHOLD:
                is_wide_corner = True
                corner_direction = 1 if left_avg > right_avg else -1
        
        # Special case: One side very far (straight wall), other side close (curved wall)
        # EXTREMELY STRICT thresholds - only trigger on very obvious special corners
        special_corner = False
        if (left_avg > 3000 and right_avg < 400 and right_avg > 100) or \
           (right_avg > 3000 and left_avg < 400 and left_avg > 100):
            special_corner = True
            is_wide_corner = True  # Treat as wide corner
            print(f"⚠️ SPECIAL CORNER: L={left_avg:.0f} R={right_avg:.0f}")
        
        # Primary steering: Enhanced centering to maintain equal distance from both walls
        # Basic formula from PDF: angle = 0.02 * (left - right)
        # Enhanced with stronger centering gain
        
        # Calculate centering error (positive = too close to right, need to go left)
        centering_error = left_60_dist - right_60_dist
        
        # Base steering from centering - balanced response
        angle_degre = 0.035 * centering_error  # Balanced between 0.03 and 0.04
        
        # Detect if we're in a corner by checking inner wall curvature
        # Also detect sharp corner setup (one straight, one curved)
        in_corner = False
        inner_wall_side = None  # 'left' or 'right'
        sharp_corner_ahead = False
        sharp_corner_side = None  # Which side is curving
        
        # Calculate wall curvatures
        left_wall_curve = 0
        right_wall_curve = 0
        left_is_straight = False
        right_is_straight = False
        
        if left_30_dist > 0 and left_60_dist > 0:
            left_wall_curve = abs(left_30_dist - left_60_dist)
            left_is_straight = (left_wall_curve < 150)  # Straight if curve < 150mm
        
        if right_30_dist > 0 and right_60_dist > 0:
            right_wall_curve = abs(right_30_dist - right_60_dist)
            right_is_straight = (right_wall_curve < 150)  # Straight if curve < 150mm
        
        if left_avg > 0 and right_avg > 0:
            # Determine which is inner wall (closer one)
            if left_avg < right_avg:
                inner_wall_side = 'left'
                inner_wall_dist = left_avg
                # Check if left wall is curving (corner not finished)
                # STRICTER: Wall must be curving AND we must be close to it AND asymmetric
                if left_wall_curve > 200 and left_avg < 600 and asymmetry_ratio > 0.25:
                    in_corner = True
            else:
                inner_wall_side = 'right'
                inner_wall_dist = right_avg
                # Check if right wall is curving
                # STRICTER: Wall must be curving AND we must be close to it AND asymmetric
                if right_wall_curve > 200 and right_avg < 600 and asymmetry_ratio > 0.25:
                    in_corner = True
            
            # Detect sharp corner setup: one wall straight, other curved
            # This indicates upcoming 90-degree turn
            # IMPORTANT: Large curve values (>1000mm) are actually valid for wide corridors
            if left_is_straight and not right_is_straight and right_wall_curve > 200:
                # Left wall straight, right wall curving -> sharp RIGHT turn ahead
                # MUST follow the RIGHT wall (curved wall), NOT the left wall!
                sharp_corner_ahead = True
                sharp_corner_side = 'right'
                print(f"⚠️ SHARP RIGHT TURN AHEAD - L straight, R curving ({right_wall_curve:.0f}mm)")
            elif right_is_straight and not left_is_straight and left_wall_curve > 200:
                # Right wall straight, left wall curving -> sharp LEFT turn ahead
                # MUST follow the LEFT wall (curved wall), NOT the right wall!
                sharp_corner_ahead = True
                sharp_corner_side = 'left'
                print(f"⚠️ SHARP LEFT TURN AHEAD - R straight, L curving ({left_wall_curve:.0f}mm)")
        
        # --- NEW NAVIGATION LOGIC ---
        
        # 1. Determine driving context (Straight vs Corner)
        # Using 30-degree sensors for lookahead
        # Add safety checks for 0 values
        safe_l30 = left_30_dist if left_30_dist > 0 else 5000
        safe_r30 = right_30_dist if right_30_dist > 0 else 5000
        safe_l90 = left_90_dist if left_90_dist > 0 else 5000
        safe_r90 = right_90_dist if right_90_dist > 0 else 5000
        
        turn_direction = 0  # 0=Straight, -1=Left, 1=Right
        corner_type = "STRAIGHT"
        
        # Thresholds for turn detection
        # If one side is significantly more open than the other
        # --- DYNAMIC TURN DETECTION ---
        # Calculate candidates independently
        dir_30 = 0
        type_30 = ""
        # Logic 1: Lookahead (30 deg) - Sensitive for early detection
        # Dynamic Threshold based on Front Distance
        # Far (>2m): Conservative (1.25) to prevent wobble on straights
        # Near (<2m): Ultra-Sensitive (1.05) to catch S-bends where side walls block view
        lookahead_threshold = 1.25
        if front_dist < 2000:
            lookahead_threshold = 1.05
            
        if safe_l30 > safe_r30 * lookahead_threshold: 
            dir_30 = -1
            type_30 = f"LEFT TURN (Lookahead {lookahead_threshold})"
        elif safe_r30 > safe_l30 * lookahead_threshold: 
            dir_30 = 1
            type_30 = f"RIGHT TURN (Lookahead {lookahead_threshold})"
            
        dir_60 = 0
        type_60 = ""
        # Logic 2: Diagonal (60 deg) - Robust (1.5) for geometry
        if left_60_dist > right_60_dist * 1.5:
             dir_60 = -1
             type_60 = "LEFT TURN (Diag 60)"
        elif right_60_dist > left_60_dist * 1.5:
             dir_60 = 1
             type_60 = "RIGHT TURN (Diag 60)"
             
        # Decision Logic based on context (Distance to front wall)
        if front_dist > 1200:
            # FAR FIELD: Trust Lookahead (30) first
            # We are approaching a turn, so look further ahead
            if dir_30 != 0:
                turn_direction = dir_30
                corner_type = type_30
            elif dir_60 != 0:
                turn_direction = dir_60
                corner_type = type_60
        else:
            # NEAR FIELD: Trust Diagonal (60) first... BUT verify with Lookahead
            # This handles the S-bend "skew" issue
            if dir_60 != 0:
                turn_direction = dir_60
                corner_type = type_60
                
                # CONFLICT RESOLUTION:
                # If Diag (60) says Right, but Lookahead (30) says Left (and sees a gap),
                # it means Diag is blocked by the wall we are passing, while Lookahead sees the new turn.
                if dir_60 != dir_30 and dir_30 != 0:
                     # Check if Lookahead is seeing a "Real Gap" (points further than front wall)
                     # Lowered threshold to 1.01: If Lookahead sees *any* gap past the front wall, trust it.
                     # This prevents "Hugging Inner Wall" (failed Diag logic) from overriding the correct path.
                     if (dir_30 == -1 and safe_l30 > front_dist * 1.01) or \
                        (dir_30 == 1 and safe_r30 > front_dist * 1.01):
                           turn_direction = dir_30
                           corner_type = type_30 + " (OverrideGap)"
            elif dir_30 != 0:
                turn_direction = dir_30
                corner_type = type_30

                
        # Fallback: Side 90 (Last resort)
        if turn_direction == 0:
             if safe_l90 > safe_r90 * 2.0 and safe_l30 > 800:
                 turn_direction = -1
                 corner_type = "LEFT TURN (Side 90)"
             elif safe_r90 > safe_l90 * 2.0 and safe_r30 > 800:
                 turn_direction = 1
                 corner_type = "RIGHT TURN (Side 90)"
                 
        # --- SPECIAL WIDE CORNER 2: STATE MACHINE ---
        # Logic: Sticky state to handle large corners where L60 sees gap
        
        # 1. State Maintenance / Exit Conditions
        # 1. State Maintenance / Exit Conditions
        # Only exit if:
        # 1. Road clears up ahead (Front > 2500)
        # 2. OR Side wall is lost "gradually" (1200 < Dist < 3000)
        # If Dist jumps to > 3000, it's likely a sensor gap/infinity -> STAY LOCKED
        # The user reported R90 jumping to 3759mm inside the turn -> Should NOT exit
        
        if special_corner_2_mode == 'left':
            if front_dist > 2500 or (left_90_dist > 1200 and left_90_dist < 3000):
                special_corner_2_mode = None
                print("➡️ Exit Special Wide 2 (Left)")
        elif special_corner_2_mode == 'right':
            if front_dist > 2500 or (right_90_dist > 1200 and right_90_dist < 3000):
                special_corner_2_mode = None
                print("➡️ Exit Special Wide 2 (Right)")
        
        # 2. Entry Conditions (If not active)
        if special_corner_2_mode is None and front_dist < 2000:
             # Check Left Entry
             # Gap > 1500mm (Low threshold) AND Pivot < 800mm
             if left_60_dist > 1500 and left_90_dist > 0 and left_90_dist < 800:
                 special_corner_2_mode = 'left'
                 print("⬅️ Enter Special Wide 2 (Left) - Locking State")
                 
             # Check Right Entry
             elif right_60_dist > 1500 and right_90_dist > 0 and right_90_dist < 800:
                 special_corner_2_mode = 'right'
                 print("➡️ Enter Special Wide 2 (Right) - Locking State")
        
        # 3. Override Turn Direction
        if special_corner_2_mode == 'left':
            turn_direction = -1
            corner_type = "SPECIAL WIDE 2 (L90 Locked)"
        elif special_corner_2_mode == 'right':
            turn_direction = 1
            corner_type = "SPECIAL WIDE 2 (R90 Locked)"
            
        # 2. Calculate Steering Angle based on Context
        angle_degre = 0
        
        if turn_direction == 0:
            # STRAIGHT: Maintain center
            # Use average of 30 and 60 for stability
            l_dist = (safe_l30 + (left_60_dist if left_60_dist > 0 else safe_l30)) / 2
            r_dist = (safe_r30 + (right_60_dist if right_60_dist > 0 else safe_r30)) / 2
            
            # Simple centering controller
            # Error is difference between left and right clearance
            error = l_dist - r_dist
            
            # Proportional gain for centering
            k_p_straight = 0.04 
            
            angle_degre = error * k_p_straight
            
            # Limit correction on straights to be smooth
            angle_degre = max(min(angle_degre, 15), -15)
            
            print(f"STRAIGHT - Centering Error: {error:.0f}")
            
        elif turn_direction == -1:
            # LEFT TURN
            # Strategy: Hug the INNER wall (Left Wall)
            # We want to be close to the Left wall
            
            target_dist = 400  # Target distance from inner wall (200mm offset + 200mm buffer)
            
            # Use L60 mainly as it's orthogonal to the car when turning? 
            # Actually L30 is lookahead. L60 is side.
            # In a turn, L30 might be huge (gap). L60 sees the corner apex.
            current_dist = left_60_dist if left_60_dist > 0 else left_30_dist
            
            # Safety: cap distance measurement to avoid massive error spikes if wall is lost
            
            # --- SPECIAL WIDE CORNER 2 LOGIC (Override) ---
            if special_corner_2_mode == 'left':
                 # FULL LOCK LEFT
                 # Bypass PID, force maximum angle (will be clamped to MAX_ANGLE_DEGRE)
                 current_dist = left_90_dist
                 angle_degre = 60 # Force Max
                 print(f"   [Sticky PIVOT] LEFT FULL LOCK (Mode: {special_corner_2_mode})")
                 
            if current_dist > 1500: current_dist = 1500
            
            error = current_dist - target_dist
            
            # Gain for wall following in corner
            k_p_corner = 0.12
            
            # Positive angle = Steer Left (towards inner wall)
            # If (dist > target), error > 0 -> angle > 0 -> Turn Left (Closer to wall)
            # If (dist < target), error < 0 -> angle < 0 -> Turn Right (Away from wall)
            # If (dist < target), error < 0 -> angle < 0 -> Turn Right (Away from wall)
            if special_corner_2_mode != 'left':
                angle_degre = error * k_p_corner
                
                # Add base turn bias
                angle_degre += 15
            
            print(f"LEFT TURN - Hugging Left Wall: Dist={current_dist:.0f}")
            
        elif turn_direction == 1:
            # RIGHT TURN
            # Strategy: Hug the INNER wall (Right Wall)
            
            target_dist = 400
            
            current_dist = right_60_dist if right_60_dist > 0 else right_30_dist
            
            # Safety cap
            
            # --- SPECIAL WIDE CORNER 2 LOGIC (Override) ---
            if special_corner_2_mode == 'right':
                 # FULL LOCK RIGHT
                 current_dist = right_90_dist
                 angle_degre = -60 # Force Max Negative
                 print(f"   [Sticky PIVOT] RIGHT FULL LOCK (Mode: {special_corner_2_mode})")
                
            if current_dist > 1500: current_dist = 1500
            
            error = current_dist - target_dist
            
            # Gain
            k_p_corner = 0.12
            
            # Negative angle = Steer Right (towards inner wall)
            # If (dist > target), error > 0 -> We want Right Turn -> Angle negative
            # Negative angle = Steer Right (towards inner wall)
            # If (dist > target), error > 0 -> We want Right Turn -> Angle negative
            if special_corner_2_mode != 'right':
                angle_degre = -(error * k_p_corner)
                
                # Add base turn bias
                angle_degre -= 15
            
            print(f"RIGHT TURN - Hugging Right Wall: Dist={current_dist:.0f}")

        # 3. Final Safety Clamp (Critical fix for >180 degree angles)
        angle_degre = max(min(angle_degre, 44), -44)
        
        # Add basic speed control
        if abs(angle_degre) > 20:
            set_vitesse_m_s(0.8)
        else:
            set_vitesse_m_s(1.5)

        
        # Secondary: Camera-based road following (reduced weight for better centering)
        camera_correction = -vis_error * 6.0  # Reduced from 8.0
        
        # Combine Lidar (primary, 85%) and Camera (secondary, 15%)
        final_angle = angle_degre + (camera_correction * 0.15)  # Reduced camera influence
        print(f"Final angle: {final_angle:.2f}° [{corner_type}]")
        if final_angle < 0:
             print(f"Distances - [L90: {left_90_dist:.0f}] L60: {left_60_dist:.0f}; L30: {left_30_dist:.0f}; Front: {front_dist:.0f}; R30: {right_30_dist:.0f}; R60: {right_60_dist:.0f} [R90: {right_90_dist:.0f}]")
        else:
             print(f"Distances - [L90: {left_90_dist:.0f}] L60: {left_60_dist:.0f}; L30: {left_30_dist:.0f}; Front: {front_dist:.0f}; R30: {right_30_dist:.0f}; R60: {right_60_dist:.0f} [R90: {right_90_dist:.0f}]")
        print(" ")
        
        # Set steering
        set_direction_degre(final_angle)
        
        # Speed control based on front clearance and corner type
        if in_corner:
            vitesse_m_s = 0.65  # Slow down when still in corner (wall curving)
        elif is_wide_corner or special_corner:
            vitesse_m_s = 0.7  # Slower for wide/special corners
        elif front_dist > 1000 or front_dist == 0:
            vitesse_m_s = 1.5  # Fast on straights
        elif front_dist > 500:
            vitesse_m_s = 1.0  # Medium
        else:
            vitesse_m_s = 0.6  # Slow when approaching obstacles
        
        set_vitesse_m_s(vitesse_m_s)

