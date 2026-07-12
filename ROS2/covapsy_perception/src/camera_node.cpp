#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/qos.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10,
            std::bind(&CameraNode::image_callback, this, std::placeholders::_1)
        );
        publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/camera/lane_offset", 
            rclcpp::QoS(10).best_effort()
        );
        RCLCPP_INFO(this->get_logger(), "CameraNode Start");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Step 1: Convert ROS2 image message to OpenCV format
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Step 2: Crop ROI (40%~90% of height) to ignore sky and ground
        int h = frame.rows;
        int w = frame.cols;
        cv::Mat roi = frame(cv::Range(h * 0.4, h * 0.9), cv::Range(0, w));

        // Step 3: Convert BGR to HSV color space
        cv::Mat hsv;
        cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

        // Step 4: Generate red mask (left wall)
        // Red wraps around both ends of the hue circle, so two ranges are needed
        cv::Mat mask_red1, mask_red2, mask_red;
        cv::inRange(hsv, cv::Scalar(0,   70, 50), cv::Scalar(10,  255, 255), mask_red1);
        cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask_red2);
        cv::bitwise_or(mask_red1, mask_red2, mask_red);

        // Step 5: Generate green mask (right wall)
        cv::Mat mask_green;
        cv::inRange(hsv, cv::Scalar(40, 50, 50), cv::Scalar(90, 255, 255), mask_green);

        // Step 6: Counten pixels in righthalf
        cv::Mat left_half_red    = mask_red(cv::Range::all(), cv::Range(0, w/2));
        cv::Mat right_half_green = mask_green(cv::Range::all(), cv::Range(w/2, w));

        int red_pixels   = cv::countNonZero(left_half_red);
        int green_pixels = cv::countNonZero(right_half_green);

        // Step 7: Compu drifting right,negative = drifting left)
        float offset = (float)(green_pixels - red_pixels) / (w * h * 0.5 * 0.5 + 1);

        RCLCPP_INFO(this->get_logger(),
            "red_pixels=%d  green_pixels=%d  offset=%.3f",
            red_pixels, green_pixels, offset);

        // Publish lane offset for control node
        std_msgs::msg::Float32 offset_msg;
        offset_msg.data = offset;
        publisher_->publish(offset_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
