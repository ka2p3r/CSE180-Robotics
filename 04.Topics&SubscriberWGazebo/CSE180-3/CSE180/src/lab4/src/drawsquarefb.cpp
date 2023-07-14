#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float y, x, theta;  //initialize  the variable for robot's position and oriention 
bool init = false;

tf2::Quaternion q;
bool valid = false;

//callback function for the roboot odometry 
void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double roll, pitch, yaw;
    tf2::convert(msg->pose.pose.orientation, q); //extract position & orientation from odometry 
    tf2::Matrix3x3 m(q);                        
    m.getRPY(roll, pitch, yaw); // get the pitch, roll and yaw variable and update them                 
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    theta = yaw;
}

int main(int argc, char **argv) { // create sub and pub node declaration 
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("drawsquarefb");
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/p2dx/cmd_vel", 1000);
    auto sub = node->create_subscription<nav_msgs::msg::Odometry>("/p2dx/odom", 1000, &odomCallBack);

    geometry_msgs::msg::Twist msg;
    init = false;

    bool rotate = false;
    float start_x = x;
    float start_y = y;
    double DIRS[] = {0, M_PI/2, -M_PI, -M_PI/2}; 
    int direction = 0; 
    

    while (rclcpp::ok()) {
        rclcpp::spin_some(node); // get sensor reading, if available
        if (rotate) { // rotating?
            // reached desired heading?
            if (((direction == 0) && (theta < DIRS[0])) ||
                ((direction == 1) && (theta < DIRS[1])) ||
                ((direction == 2) && (theta > 0)) ||
                ((direction == 3) && (theta < DIRS[3]))) {
                msg.linear.x = 0;
                msg.angular.z = M_PI/8; // no; keep rotating
            } else { // yes
                msg.linear.x = 0;
                msg.angular.z = 0; // stop the robot
                rotate = false; // switch to translating
                start_x = x;
                start_y = y; // record current location
            }
        } else { // translating?
            if (hypotf((x-start_x), (y-start_y)) < 2) { // moved less than 2 units?
                msg.linear.x = 0.5;
                msg.angular.z = 0; // no, keep moving forward
            } else { // moved 2 units
                rotate = true; // switch to rotate
                msg.linear.x = 0;
                msg.angular.z = 0; // stop the robot
                direction = (direction + 1) % 4; // track next direction
            }
        }
        pub->publish(msg); // send motion command
    }
    rclcpp::shutdown();

    return 0;
}
