#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/int32.hpp>
#include<std_msgs/msg/string.hpp>

rclcpp::Node::SharedPtr nodeh; 

int prv_val = 0; 
int sum = 0; 

void intCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
    sum += msg->data; 
    prv_val++; 
    if (prv_val > 0){
        RCLCPP_INFO(nodeh->get_logger(),"Sum pervious iterations: %d",sum);
        prv_val = 0; 
        sum = msg->data;
    }

}

int main(int argc,char **argv) {
    rclcpp::init(argc,argv);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subs; 
    nodeh = rclcpp::Node::make_shared("intsubscriber");

    subs = nodeh->create_subscription<std_msgs::msg::Int32>
        ("inttopic",10,&intCallBack);
    
    rclcpp::spin(nodeh);

    rclcpp::shutdown();

    return 0; 
}
