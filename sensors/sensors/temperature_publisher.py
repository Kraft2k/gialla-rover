#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

import RPi.GPIO as GPIO
import dht11
import time
import datetime

# initialize GPIO
GPIO.setwarnings(True)
GPIO.setmode(GPIO.BCM)

# read data using pin 6
instance = dht11.DHT11(pin=6)



class PublisherTemperatureNode(Node):
    def __init__(self):
        super().__init__("sensor_temperature_humidity")

        self.robot_name_ = "Jialla_series"
        self.publisher_ = self.create_publisher(String, "environmental_sensor", 10)
        self.timer_ = self.create_timer(5, self.publish_news)
        self.get_logger().info("Sensor Info Temperature and Humidity has been started")

    def publish_news(self):
        result = instance.read()
        msg = String()
        msg.data = "Last valid input:" + str(datetime.datetime.now()) + " Temperature: " + str(result.temperature) + " Humidity: " + str(result.humidity)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublisherTemperatureNode()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()

