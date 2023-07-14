#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

float linear_velocity;
float angular_velocity;

void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    linear_velocity = msg->linear_velocity;
    angular_velocity = msg->angular_velocity;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("turtle_to_p2dx");

    auto sub = node->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, &poseCallback);

    auto pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/p2dx/cmd_vel", 10);

    while (rclcpp::ok()) {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = linear_velocity;
        vel_msg.angular.z = angular_velocity;
        pub->publish(vel_msg);
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
