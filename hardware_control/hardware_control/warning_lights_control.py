#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
import time


PIN_RIGTH_LIGHT = 19
PIN_LEFT_LIGHT = 20


class WarningLightsNode(Node):

    def __init__(self):
        super().__init__("warning_light")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_RIGTH_LIGHT,GPIO.OUT)
        GPIO.setup(PIN_LEFT_LIGHT,GPIO.OUT)

        self.counter_ = 0
        self.get_logger().info("Warning Lights has been started.")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        GPIO.output(PIN_RIGTH_LIGHT,GPIO.LOW)
        time.sleep(0.03)
        GPIO.output(PIN_RIGTH_LIGHT,GPIO.HIGH)
        time.sleep(0.03)
        GPIO.output(PIN_RIGTH_LIGHT,GPIO.LOW)
        time.sleep(0.03)
        GPIO.output(PIN_RIGTH_LIGHT,GPIO.HIGH)
        time.sleep(0.05)

        GPIO.output(PIN_LEFT_LIGHT,GPIO.LOW)
        time.sleep(0.03)
        GPIO.output(PIN_LEFT_LIGHT,GPIO.HIGH)
        time.sleep(0.03)
        GPIO.output(PIN_LEFT_LIGHT,GPIO.LOW)
        time.sleep(0.03)
        GPIO.output(PIN_LEFT_LIGHT,GPIO.HIGH)
        time.sleep(0.05)

        self.counter_ += 1
        self.get_logger().info("Counter " + str(self.counter_))


def main(args=None):
    rclpy.init(args=args)
    node = WarningLightsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

