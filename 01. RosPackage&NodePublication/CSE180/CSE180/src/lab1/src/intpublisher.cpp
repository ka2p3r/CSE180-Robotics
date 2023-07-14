
#include <rclcpp/rclcpp.hpp> 
#include <std_msgs/msg/int32.hpp> 
#include <std_msgs/msg/string.hpp> 

int main(int argc,char **argv) {

	rclcpp::init(argc,argv); 

	rclcpp::Node::SharedPtr nodeh;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pubi;
	rclcpp::Rate rate(2);

	nodeh = rclcpp::Node::make_shared("intpublisher"); 
	// create publisher to topic "intm" of integers
	pubi = nodeh->create_publisher<std_msgs::msg::Int32>("inttopic",1);

	int value=0;
	std_msgs::msg::Int32 intToSend; 

	while (rclcpp::ok()) {
		intToSend.data = value++; 
		pubi->publish(intToSend); 
		rclcpp::spin_some(nodeh); 
		RCLCPP_INFO(nodeh->get_logger(),"Published iteration %d",value);
		rate.sleep(); // wait
	}
	rclcpp::shutdown(); // unreachable in the current form
	return 0;

}