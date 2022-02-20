#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class SubsriberTemperatureNode(Node):
    def __init__(self):
        super().__init__("subscriber_temperature_humidity")
        self.subscriber_ = self.create_subscription(
            String, "environmental_sensor", self.callback_robot_news, 10)
        self.get_logger().info("Subcriber Temperature and Humidity has been started.")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SubsriberTemperatureNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

