#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class VelodyneRepublisherNode : public rclcpp::Node
{
public:
    VelodyneRepublisherNode() : Node("velodyne_republisher_node")
    {
        // Subscriber to the /velodyne_points topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points",
            10,
            std::bind(&VelodyneRepublisherNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher to republish modified messages to the /velodyne_points topic
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points_modified", 10);

        RCLCPP_INFO(this->get_logger(), "Subscribed to and republishing on /velodyne_points topic");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Create a copy of the received message to modify
        auto modified_msg = *msg;

        // Check if width is 0, and only then set row_step to 0

        modified_msg.row_step = 0;
        RCLCPP_INFO(this->get_logger(), "Width is 0; setting row_step to 0 and republishing");

        // Republish the modified message
        publisher_->publish(modified_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelodyneRepublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
