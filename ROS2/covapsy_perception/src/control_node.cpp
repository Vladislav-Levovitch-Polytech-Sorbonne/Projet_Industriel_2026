#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/qos.hpp"

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {   

        // Use BEST_EFFORT QoS for low-latency sensor data
        auto qos_sensor = rclcpp::QoS(10).best_effort();


        // Subscribe to lane offset from camera
        sub_offset_ = this->create_subscription<std_msgs::msg::Float32>(
            "/camera/lane_offset", qos_sensor,
            std::bind(&ControlNode::offset_callback, this, std::placeholders::_1)
        );

        // Subscribe to sector distances from lidar
        sub_lidar_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/lidar/sectors", qos_sensor,
            std::bind(&ControlNode::lidar_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ControlNode Start");
    }

private:
    void offset_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        lane_offset_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Lane offset: %.3f",lane_offset_);
        compute_control();
    }

    void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        sector_distances_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Front distance: %.2fm",sector_distances_[0]);
        compute_control();
    }

    // Received data
    float lane_offset_ = 0.0f;
    std::vector<float> sector_distances_ = std::vector<float>(12, 10.0f);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_offset_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_lidar_;


    float prev_offset_ = 0.0f;  // for PD derivative term
    //command
    void compute_control()
{
    float steering = 0.0f;
    float speed    = 0.5f;  // default speed (m/s)

    float front  = sector_distances_[0];   // sector 0 = front
    float front_left  = sector_distances_[11]; // sector 11 = front-left
    float front_right = sector_distances_[1];  // sector 1  = front-right

    // Layer 3: TTC safety guardian — emergency stop
    if (front < 0.4f)
    {
        speed    = 0.0f;
        steering = 0.0f;
        RCLCPP_WARN(this->get_logger(), "[TTC] Emergency stop! front=%.2fm", front);
        return;
    }

    // Layer 2: obstacle avoidance — steer away from nearest side
    if (front < 1.5f)
    {
        steering = (front_left < front_right) ? 0.3f : -0.3f;
        speed    = 0.2f;
        RCLCPP_WARN(this->get_logger(), "[AVOID] obstacle front=%.2fm steer=%.2f", front, steering);
        return;
    }

    // Layer 1: PD centering control
    float kp = 0.8f;
    float kd = 0.2f;
    float d_offset = lane_offset_ - prev_offset_;
    steering = -(kp * lane_offset_ + kd * d_offset);
    prev_offset_ = lane_offset_;

    RCLCPP_INFO(this->get_logger(),
        "[PD] offset=%.3f steering=%.3f speed=%.2f",
        lane_offset_, steering, speed);
}
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}