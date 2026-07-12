#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <limits>
#include "rclcpp/qos.hpp"

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("lidar_node")
    {   
        
        //subscribe scan
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&LidarNode::scan_callback, this, std::placeholders::_1)
        );

        //publish sector data
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/lidar/sectors",
            rclcpp::QoS(10).best_effort()
        );

        RCLCPP_INFO(this->get_logger(),"LidarNode Start");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const int NUM_SECTORS = 12;
        int total_points = msg->ranges.size();
        int points_per_sector = total_points / NUM_SECTORS;

        std_msgs::msg::Float32MultiArray sector_msg;
        sector_msg.data.resize(NUM_SECTORS);

        for (int s = 0; s < NUM_SECTORS; s++)
        {
            float min_dist = std::numeric_limits<float>::max();

            int start = s * points_per_sector;
            int end   = start + points_per_sector;

            for (int i = start; i < end; i++)
            {
                float r = msg->ranges[i];
                if (std::isfinite(r) && r > msg->range_min && r < msg->range_max)
                {
                    if (r < min_dist) min_dist = r;
                }
            }

            if (min_dist == std::numeric_limits<float>::max())
                min_dist = msg->range_max;

            sector_msg.data[s] = min_dist;
        }

        publisher_->publish(sector_msg);

        RCLCPP_INFO(this->get_logger(),
            "sector0(front)=%.2fm  sector3(right)=%.2fm  sector6(rear)=%.2fm  sector9(left)=%.2fm",
            sector_msg.data[0], sector_msg.data[3],
            sector_msg.data[6], sector_msg.data[9]
        );
    }

    

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}