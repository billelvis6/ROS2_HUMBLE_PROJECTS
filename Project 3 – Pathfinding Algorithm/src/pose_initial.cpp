#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher()
    : Node("initial_pose_publisher")
    {
        this->declare_parameter("x", 0.0);
        this->declare_parameter("y", 0.0);
        this->declare_parameter("yaw", 0.0);
        this->declare_parameter("frame_id", "map");

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        timer_ = this->create_wall_timer(
            1s, std::bind(&InitialPosePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        double x = this->get_parameter("x").as_double();
        double y = this->get_parameter("y").as_double();
        double yaw = this->get_parameter("yaw").as_double();
        std::string frame_id = this->get_parameter("frame_id").as_string();

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = frame_id;
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);  // Only yaw
        pose.pose.pose.orientation.z = q.z();
        pose.pose.pose.orientation.w = q.w();

        // Covariance
        pose.pose.covariance[0] = 0.25;
        pose.pose.covariance[7] = 0.25;
        pose.pose.covariance[35] = 0.06853891945200942;

        publisher_->publish(pose);

        RCLCPP_INFO(this->get_logger(), "Published initial pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

        // Cancel the timer to publish only once
        timer_->cancel();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
