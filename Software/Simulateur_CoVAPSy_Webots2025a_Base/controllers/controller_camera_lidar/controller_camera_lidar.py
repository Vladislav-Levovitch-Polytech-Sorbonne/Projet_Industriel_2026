# Copyright 2024 Antigravity for CoVAPSy
# Subsumption Architecture - Base Control Layer (Layer 1)
# 纯 Lidar 主控（PD + FGM）+ 视觉语义检查器

from vehicle import Driver
from controller import Lidar, Camera
import numpy as np
import cv2

# --- 常量定义 ---
MAX_SPEED_KMH = 4 # 28
MAX_ANGLE_DEGRE = 16
CRASH_DISTANCE_MM = 350  # 碰撞检测距离 350
MIN_COLOR_PIXELS = 800  # 最小颜色像素数

# --- 初始化 ---
driver = Driver()
basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = basicTimeStep  # 20ms

# 传感器
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

camera = Camera("camera")
camera.enable(sensorTimeStep)

# --- 辅助函数 ---

def get_lidar_array_mm(lidar_device):
    """
    将 Lidar 数据转换为 mm 数组（原始数据，不池化）
    索引映射：Index 0 = 正前方, 正数 = 左侧, 负数 = 右侧
    """
    donnees_lidar_brutes = lidar_device.getRangeImage()
    if not donnees_lidar_brutes:
        return [0] * 360

    tableau_lidar_mm = [0] * 360

    # 转换为 mm 并重映射索引（参考 controller_jaune.py）
    for i in range(360):
        if (donnees_lidar_brutes[-i] > 0) and (donnees_lidar_brutes[-i] < 20):
            tableau_lidar_mm[i-180] = 1000 * donnees_lidar_brutes[-i]
        else:
            tableau_lidar_mm[i-180] = 0

    return tableau_lidar_mm


def check_wrong_way_hsv(image_bgr):
    """
    视觉语义检查器：检测车辆是否逆行

    判断逻辑：
    - 正确方向：红墙在左（red_left > red_right），绿墙在右（green_right > green_left）
    - 逆行：红墙在右，绿墙在左

    参数：
    - image_bgr: OpenCV BGR格式图像

    返回：
    - is_wrong_way: 布尔值，True表示逆行
    """
    if image_bgr is None:
        return False

    h, w = image_bgr.shape[:2]

    # 1. 缩放图像以提升性能
    target_w = 320
    target_h = int(h * (target_w / w))
    img_small = cv2.resize(image_bgr, (target_w, target_h))

    # 2. ROI提取：聚焦前方道路区域
    roi = img_small[int(target_h*0.4):int(target_h*0.9), :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 3. HSV颜色检测
    # 绿色（右墙）- RAL 6037
    mask_green = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([90, 255, 255]))

    # 红色（左墙）- RAL 3020（两个色相范围）
    mask_red = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255])) | \
               cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))

    # 4. 左右半区统计
    roi_h, roi_w = roi.shape[:2]
    mid_x = roi_w // 2

    red_left = cv2.countNonZero(mask_red[:, :mid_x])
    red_right = cv2.countNonZero(mask_red[:, mid_x:])
    green_left = cv2.countNonZero(mask_green[:, :mid_x])
    green_right = cv2.countNonZero(mask_green[:, mid_x:])

    # 5. 逆行判断逻辑
    total_red = red_left + red_right
    total_green = green_left + green_right

    is_wrong_way = False

    # 只有当两种颜色都可见时才检查方向
    if total_red > MIN_COLOR_PIXELS and total_green > MIN_COLOR_PIXELS:
        # 严格判据：红色主要在右 AND 绿色主要在左
        red_mainly_right = (red_right / total_red) > 0.7
        green_mainly_left = (green_left / total_green) > 0.7

        if red_mainly_right and green_mainly_left:
            is_wrong_way = True

    return is_wrong_way


def set_vitesse_m_s(vitesse_m_s):
    """将 m/s 转换为 km/h 并设置速度"""
    speed = vitesse_m_s * 3.6
    if speed > MAX_SPEED_KMH:
        speed = MAX_SPEED_KMH
    if speed < 0:
        speed = 0
    driver.setCruisingSpeed(speed)


def set_direction_degre(angle_degre):
    """设置转向角（度）"""
    if angle_degre > MAX_ANGLE_DEGRE:
        angle_degre = MAX_ANGLE_DEGRE
    elif angle_degre < -MAX_ANGLE_DEGRE:
        angle_degre = -MAX_ANGLE_DEGRE
    # 转换为弧度（负号是 Webots 约定）
    angle = -angle_degre * 3.14 / 180
    driver.setSteeringAngle(angle)


def recule():
    """倒车（3 km/h）"""
    driver.setCruisingSpeed(-2)


# --- 纯 Lidar 控制器类 ---

class PureLidarController:
    """
    纯 Lidar 控制器
    - 直线段：墙壁跟随（PD 控制）
    - 弯道：FGM 间隙跟随
    """

    def __init__(self):
        # PD 参数
        self.k_p_straight = 0.04  # 直线段比例增益
        self.k_d_straight = 0.005  # 直线段微分增益（降低以减少震荡）
        self.k_p_corner = 0.3  # 弯道比例增益（FGM 增益，降低以防止过冲）

        # 状态变量
        self.prev_error = 0.0
        self.prev_mode = 'straight'  # 用于迟滞判断
        self.sensors_valid = False  # 传感器是否已经初始化

    def detect_mode(self, lidar_mm):
        """
        检测驾驶模式：直线段 vs 弯道（带迟滞）

        判断依据：左右距离比值（限制在合理范围内，避免传感器穿透误判）
        - 比值 > 2.5 → 弯道（进入阈值更高）
        - 比值 < 1.8 → 直线（退出阈值更低，迟滞区间 1.8~2.5）
        """
        raw_left = lidar_mm[60]
        raw_right = lidar_mm[-60]

        # 限制最大距离为 3000mm，防止传感器穿透空旷区域导致误判
        left_dist = min(raw_left, 3000) if raw_left > 0 else 3000
        right_dist = min(raw_right, 3000) if raw_right > 0 else 3000

        ratio = max(left_dist, right_dist) / (min(left_dist, right_dist) + 1)

        # 迟滞：避免直线/弯道频繁切换
        if self.prev_mode == 'straight':
            new_mode = 'corner' if ratio > 2.5 else 'straight'
        else:  # prev_mode == 'corner'
            new_mode = 'straight' if ratio < 1.8 else 'corner'

        # 模式切换时：将 prev_error 设为当前 error，使第一步微分项为 0
        if new_mode != self.prev_mode:
            current_error = left_dist - right_dist
            self.prev_error = current_error

        self.prev_mode = new_mode
        return new_mode

    def wall_following_pd(self, lidar_mm, dt):
        """
        墙壁跟随（直线段 PD 控制）

        PD 控制律：steer = k_p * error + k_d * (error - prev_error) / dt

        使用原始 Lidar 数据的精确角度
        """
        # 比例项：左右距离差（使用60°角度，最稳定）
        raw_left = lidar_mm[60]
        raw_right = lidar_mm[-60]
        now_valid = (raw_left > 0) and (raw_right > 0)

        left_60 = raw_left if raw_left > 0 else 2000
        right_60 = raw_right if raw_right > 0 else 2000
        error = left_60 - right_60

        # 传感器第一次有效时，预置 prev_error = 当前 error，避免微分项突变
        if now_valid and not self.sensors_valid:
            self.prev_error = error

        self.sensors_valid = now_valid

        # 微分项：仅在传感器有效时计算，限制最大变化率防止传感器突变爆炸
        if dt > 0 and now_valid:
            d_error = (error - self.prev_error) / dt
            d_error = max(min(d_error, 2000), -2000)  # 限制在 ±2000 mm/s
        else:
            d_error = 0

        # PD 控制
        steer = self.k_p_straight * error + self.k_d_straight * d_error

        # 更新历史
        self.prev_error = error

        return steer

    def fgm_gap_following(self, lidar_mm):
        """
        FGM 间隙跟随（弯道）

        算法：距离加权质心
        - threshold=1500mm：过滤掉近处障碍，区分弯道出口和直行区域
        - 加权质心：距离越远权重越高，自然指向最开阔方向
        - 弯道场景：左侧5000mm > 右侧800mm → 质心偏向左侧
        """
        threshold = 1500  # 提高阈值，过滤近处区域

        # 距离加权质心
        weighted_sum = 0.0
        total_weight = 0.0

        for angle in range(-90, 91):
            dist = lidar_mm[angle]
            if dist > threshold:
                weighted_sum += angle * dist
                total_weight += dist

        if total_weight > 0:
            center_angle = weighted_sum / total_weight
        else:
            # 无可行驶区域：指向最远点
            max_dist = 0
            center_angle = 0
            for angle in range(-90, 91):
                if lidar_mm[angle] > max_dist:
                    max_dist = lidar_mm[angle]
                    center_angle = angle

        steer = center_angle * self.k_p_corner
        return steer

    def compute_steering(self, lidar_mm, dt):
        """
        计算转向角

        返回：(steer, mode)
        """
        mode = self.detect_mode(lidar_mm)

        if mode == 'straight':
            steer = self.wall_following_pd(lidar_mm, dt)
        else:  # corner
            steer = self.fgm_gap_following(lidar_mm)

        return steer, mode


# --- Subsumption 仲裁器类 ---

class SubsumptionController:
    """
    Subsumption 架构仲裁器

    三层级（从高到低）：
    1. 视觉语义层（优先级 3）- 逆行检测 → U-Turn
    2. 碰撞恢复层（优先级 2）- 碰撞检测 → 倒车恢复
    3. Lidar 连续控制层（优先级 1，默认）- PD/FGM 控制
    """

    # 状态机
    STATE_FORWARD = 0
    STATE_REVERSE = 1
    STATE_UTURN = 2
    STATE_RECOVERY = 3

    def __init__(self, driver_obj):
        self.driver = driver_obj
        self.lidar_controller = PureLidarController()

        # 状态
        self.state = self.STATE_FORWARD
        self.state_timer = 0.0
        self.reverse_angle = 0.0
        self.recovery_angle = 0.0
        self.uturn_stage = 0  # 0: 倒车, 1: 转弯
        self.stuck_timer = 0.0  # 卡住计时器
        self.immune_until = 0.0  # 恢复后的免疫期结束时间

    def subsume(self, lidar_mm, is_wrong_way, dt):
        """
        层级仲裁主函数

        返回：(steer, speed, layer_name)
        """
        current_time = self.driver.getTime()

        # 层级 3：视觉语义（最高优先级）
        if is_wrong_way and self.state == self.STATE_FORWARD:
            print("⚠️ 检测到逆行！启动 U-Turn")
            self.state = self.STATE_UTURN
            self.uturn_stage = 0
            self.state_timer = current_time + 1.5  # 倒车 1.5 秒
            return self.execute_uturn(current_time)

        # 层级 2：碰撞检测（双重条件，免疫期内跳过）
        if self.state == self.STATE_FORWARD and current_time > self.immune_until:
            front_dist = lidar_mm[0]
            actual_speed_kmh = abs(self.driver.getCurrentSpeed())

            # 条件A：前方太近（轮子打滑顶墙）
            too_close = (front_dist > 0 and front_dist < 300)

            # 条件B：速度持续为零（真正卡住）
            if actual_speed_kmh < 1.0:
                self.stuck_timer += dt
            else:
                self.stuck_timer = 0.0
            truly_stuck = (self.stuck_timer > 0.8)

            if too_close or truly_stuck:
                self.stuck_timer = 0.0
                reason = f"前方 {front_dist:.0f}mm" if too_close else f"速度 {actual_speed_kmh:.1f}km/h"
                print(f"💥 触发倒车！原因: {reason}")
                self.state = self.STATE_REVERSE
                self.state_timer = current_time + 1.0

                # 恢复方向：选全方向中最开阔的一侧（用 90° 和 60° 综合判断）
                left_score = (lidar_mm[90] if lidar_mm[90] > 0 else 0) + (lidar_mm[60] if lidar_mm[60] > 0 else 0)
                right_score = (lidar_mm[-90] if lidar_mm[-90] > 0 else 0) + (lidar_mm[-60] if lidar_mm[-60] > 0 else 0)
                if left_score > right_score:
                    self.recovery_angle = MAX_ANGLE_DEGRE
                    # 倒车时也朝开阔方向（倒车+左转 = 后方右移，前方左移）
                    self.reverse_angle = -MAX_ANGLE_DEGRE
                    print(f"  恢复方向: 左转（左分{left_score:.0f} > 右分{right_score:.0f}）")
                else:
                    self.recovery_angle = -MAX_ANGLE_DEGRE
                    self.reverse_angle = MAX_ANGLE_DEGRE
                    print(f"  恢复方向: 右转（右分{right_score:.0f} > 左分{left_score:.0f}）")

                return 0, -0.83, 'reverse'

        # 执行 U-Turn
        if self.state == self.STATE_UTURN:
            return self.execute_uturn(current_time)

        # 执行碰撞恢复
        if self.state in [self.STATE_REVERSE, self.STATE_RECOVERY]:
            return self.execute_collision_recovery(current_time)

        # 层级 1：纯 Lidar 控制（默认）
        steer, mode = self.lidar_controller.compute_steering(lidar_mm, dt)
        # 弯道时降速，给更多时间转向
        if mode == 'corner':
            speed = 0.8
        else:
            speed = 0.8 if abs(steer) > 20 else 1.5

        # 前方距离比例降速（F < 600mm 时线性减速，最低 0.3 m/s）
        front_dist = lidar_mm[0]
        if front_dist > 0 and front_dist < 600:
            speed_factor = front_dist / 600.0
            speed = max(0.3, speed * speed_factor)

        return steer, speed, f'lidar_{mode}'

    def execute_uturn(self, current_time):
        """执行 U-Turn 动作"""
        if current_time < self.state_timer:
            if self.uturn_stage == 0:
                # 阶段 0：倒车
                set_direction_degre(0)
                recule()
                return 0, -0.83, 'uturn_reverse'
            elif self.uturn_stage == 1:
                # 阶段 1：转弯
                set_direction_degre(16)
                set_vitesse_m_s(0.5)
                return 16, 0.5, 'uturn_turn'
        else:
            if self.uturn_stage == 0:
                self.uturn_stage = 1
                self.state_timer = current_time + 2.0  # 转弯 2.0 秒
                return 0, -0.83, 'uturn_reverse'
            elif self.uturn_stage == 1:
                self.state = self.STATE_FORWARD
                self.uturn_stage = 0
                print("✅ U-Turn 完成，恢复前进")
                return 0, 1.5, 'uturn_complete'

        return 0, 0, 'uturn'

    def execute_collision_recovery(self, current_time):
        """执行碰撞恢复动作"""
        if self.state == self.STATE_REVERSE:
            if current_time > self.state_timer:
                # 倒车完成，切换到恢复阶段
                self.state = self.STATE_RECOVERY
                self.state_timer = current_time + 0.8  # 恢复转向 0.8 秒
                print("🔄 倒车完成，开始恢复转向")
                return self.recovery_angle, 0.5, 'recovery_start'
            else:
                # 继续倒车
                set_direction_degre(self.reverse_angle)
                recule()
                return self.reverse_angle, -0.83, 'reversing'

        elif self.state == self.STATE_RECOVERY:
            if current_time > self.state_timer:
                # 恢复完成，切换回前进，设置 2 秒免疫期
                self.state = self.STATE_FORWARD
                self.immune_until = current_time + 2.0
                print("✅ 碰撞恢复完成，恢复正常导航（免疫 2s）")
                return 0, 1.5, 'recovery_complete'
            else:
                # 执行恢复转向
                set_direction_degre(self.recovery_angle)
                set_vitesse_m_s(0.5)
                return self.recovery_angle, 0.5, 'recovering'

        return 0, 0, 'recovery'


# --- 主循环 ---

print("=" * 60)
print("✅ 基础控制层启动 - Subsumption 架构")
print("   - 纯 Lidar PD 控制 + FGM 间隙跟随（使用原始数据）")
print("   - 视觉降级为语义检查器（逆行检测）")
print("   - 三层级仲裁：语义 > 碰撞 > 连续控制")
print("=" * 60)

driver.setSteeringAngle(0)
driver.setCruisingSpeed(0)

# 初始化控制器
subsumption = SubsumptionController(driver)
camera_check_counter = 0
prev_time = 0.0

while driver.step() != -1:
    current_time = driver.getTime()
    dt = current_time - prev_time
    prev_time = current_time

    # 1. Lidar 数据获取（50Hz，原始数据）
    lidar_mm = get_lidar_array_mm(lidar)

    # 2. 视觉语义检查（10Hz 降频）
    is_wrong_way = False
    camera_check_counter += 1
    if camera_check_counter >= 5:
        camera_check_counter = 0
        img_data = camera.getImage()
        if img_data is not None:
            h, w = camera.getHeight(), camera.getWidth()
            img_arr = np.frombuffer(img_data, dtype=np.uint8).reshape((h, w, 4))
            img_bgr = img_arr[:, :, :3]
            is_wrong_way = check_wrong_way_hsv(img_bgr)

    # 3. Subsumption 仲裁（传入原始 Lidar 数据）
    steer, speed, layer = subsumption.subsume(lidar_mm, is_wrong_way, dt)

    # 4. 执行控制命令
    set_direction_degre(steer)
    if speed < 0:
        recule()
    else:
        set_vitesse_m_s(speed)

    # 5. 调试输出（每0.1秒打印一次）
    if current_time % 0.1 < 0.02:
        front = lidar_mm[0]
        left_30 = lidar_mm[30]
        right_30 = lidar_mm[-30]
        left_60 = lidar_mm[60]
        right_60 = lidar_mm[-60]
        left_90 = lidar_mm[90]
        right_90 = lidar_mm[-90]
        error = left_60 - right_60
        print(f"t={current_time:.2f} [{layer}] steer={steer:.1f}° spd={speed:.2f} | L90={left_90:.0f} L60={left_60:.0f} L30={left_30:.0f} F={front:.0f} R30={right_30:.0f} R60={right_60:.0f} R90={right_90:.0f} | err={error:.0f}")
