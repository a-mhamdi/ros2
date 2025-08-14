#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DemoPub : public rclcpp::Node {
public:
    DemoPub() : Node("demo_pub") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("demo_topic", 10);
        timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&DemoPub::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello World! " + std::to_string(count_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        ++count_;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 1;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoPub>());
    rclcpp::shutdown();
    return 0;
}