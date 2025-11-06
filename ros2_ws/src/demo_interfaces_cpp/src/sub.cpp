//
// Created by mhamdi on 13/08/2025.
//

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/sensor_data.hpp"

class DemoSub : public rclcpp::Node {
    public:
        DemoSub() : Node("demo_sub") {
            subscription_ = this->create_subscription<my_interfaces::msg::SensorData>(
                    "sensor_topic", 10, std::bind(&DemoSub::topic_callback, this, std::placeholders::_1));
        }
    private:
        void topic_callback(const my_interfaces::msg::SensorData::SharedPtr sensor_data) const {
            RCLCPP_INFO(this->get_logger(), "Measurements of %s are: \nTemperature\t %f\nHumidity\t %f\nPressure\t %f\nIs Valid\t %s", sensor_data->sensor_id.c_str(), sensor_data->temperature, sensor_data->humidity, sensor_data->pressure, sensor_data->is_valid ? "true": "false");
        }
        rclcpp::Subscription<my_interfaces::msg::SensorData>::SharedPtr subscription_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoSub>());
    rclcpp::shutdown();
    return 0;
}
