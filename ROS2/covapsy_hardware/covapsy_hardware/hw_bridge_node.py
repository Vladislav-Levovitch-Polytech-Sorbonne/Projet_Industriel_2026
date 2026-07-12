import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from .covapsy_spi import CoVAPSySPI, MODE_REMOTE

class HwBridgeNode(Node):
    def __init__(self):
        super().__init__('hw_bridge_node')

        # Initialize SPI
        self.spi = CoVAPSySPI()
        self.spi.open()
        self.spi.set_mode(MODE_REMOTE)

        # Publishers: STM32 sensor data → ROS2
        self.pub_sharp_left  = self.create_publisher(Float32, '/sharp/left',  10)
        self.pub_sharp_right = self.create_publisher(Float32, '/sharp/right', 10)
        self.pub_roll        = self.create_publisher(Float32, '/imu/roll',    10)
        self.pub_pitch       = self.create_publisher(Float32, '/imu/pitch',   10)

        # Subscriber: ROS2 control command → STM32
        self.sub_cmd = self.create_subscription(
            Float32MultiArray,
            '/cmd_control',
            self.cmd_callback,
            10
        )

        # Timer: read sensors from STM32 at 20Hz
        self.create_timer(0.05, self.sensor_timer)

        self.get_logger().info('HwBridgeNode Start')

    def sensor_timer(self):
        result = self.spi.get_sensors()
        if not result['valid']:
            return

        data = result['data']

        # Check if we got actual sensor data (simulation returns ACK instead)
        if data is None or data.get('type') != 'sensors':
            return

        # Publish SHARP distances
        msg_left = Float32()
        msg_left.data = float(data['sharp_left_cm'])
        self.pub_sharp_left.publish(msg_left)

        msg_right = Float32()
        msg_right.data = float(data['sharp_right_cm'])
        self.pub_sharp_right.publish(msg_right)

        # Publish IMU data
        msg_roll = Float32()
        msg_roll.data = float(data['roll_deg'])
        self.pub_roll.publish(msg_roll)

        msg_pitch = Float32()
        msg_pitch.data = float(data['pitch_deg'])
        self.pub_pitch.publish(msg_pitch)

    def cmd_callback(self, msg):
        # msg.data = [steering_deg, throttle_percent]
        if len(msg.data) < 2:
            return
        steering = msg.data[0]
        throttle = msg.data[1]
        self.spi.set_control_fast(steering, throttle)

    def destroy_node(self):
        self.spi.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HwBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()