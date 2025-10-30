import rclpy
from rclpy.node import Node
from my_interfaces.msg import SensorData
from my_interfaces.srv import GetData


class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor_node")
        self.publisher_ = self.create_publisher(SensorData, "sensor_data", 10)
        self.timer_ = self.create_timer(1.0, self.publish_sensor_data)
        self.get_logger().info("Sensor node started")

    def publish_sensor_data(self):
        msg = SensorData()
        msg.temperature = 25.5
        msg.humidity = 60.2
        msg.pressure = 1013.25
        msg.is_valid = True
        msg.sensor_id = "sensor_101"
        self.publisher_.publish(msg)
        self.get_logger().info("Published sensor data")


def pub(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()
