//
// Created by mhamdi on 13/08/2025.
//

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DemoSub : public rclcpp::Node {
public:
    DemoSub() : Node("demo_sub") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "demo_topic", 10, std::bind(&DemoSub::topic_callback, this, _1));
    }
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoSub>());
    rclcpp::shutdown();
    return 0;