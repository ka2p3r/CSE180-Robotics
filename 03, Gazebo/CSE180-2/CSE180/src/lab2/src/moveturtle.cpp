#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main (int argc, char **argv) {
    
    rclcpp::init(argc,argv); 
    rclcpp::Node::SharedPtr nodeh; 
    rclcpp::Rate rate(1);
    
    nodeh = rclcpp::Node::make_shared("moveturtle");
    auto pub = nodeh->create_publisher<geometry_msgs::msg::Twist>
      ("turtle1/cmd_vel",100);

    geometry_msgs::msg::Twist msg; 

    
    while (rclcpp::ok()){
            msg.linear.x = 1.0;
            msg.angular.z = 0.0;
            pub->publish(msg);
            rate.sleep();
            msg.angular.z = 0.5; 
            msg.linear.x = 0.0;
            pub->publish(msg);
            rate.sleep();
        }
        
    
    

}

