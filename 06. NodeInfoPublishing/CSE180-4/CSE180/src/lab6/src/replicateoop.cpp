#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TurtleSimToP2DX : public rclcpp::Node {
public:

    TurtleSimToP2DX() : Node("turtle_to_p2dx") {
        sub_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleSimToP2DX::poseCallback, this, std::placeholders::_1));

        pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/p2dx/cmd_vel", 10);
    }

    void run() {
        while (rclcpp::ok()) {

            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = linear_velocity_;
            vel_msg.angular.z = angular_velocity_;
            pub_->publish(vel_msg);
            rclcpp::spin_some(shared_from_this());

        }
    }

private:

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {

        linear_velocity_ = msg->linear_velocity;
        angular_velocity_ = msg->angular_velocity;

    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    float linear_velocity_;
    float angular_velocity_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto turtle_sim_to_p2dx = std::make_shared<TurtleSimToP2DX>();
    turtle_sim_to_p2dx->run(); 
    rclcpp::spin(turtle_sim_to_p2dx);
    rclcpp::shutdown();
    return 0;
}
