import rclpy
from rclpy.node import Node

import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50

servo1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2600)




class ServoRotationNode(Node):

    def __init__(self):
        super().__init__("servo_rotation")
        self.counter_ = 0
        self.get_logger().info("Servo rotation has been started.")
        self.create_timer(60, self.timer_callback)

    def timer_callback(self):
        fraction = 0.5
        while fraction > 0.0:
            servo1.fraction = fraction
            fraction -= 0.01
            time.sleep(0.2)

        fraction = 0.0
        while fraction < 0.5:
            servo1.fraction = fraction
            fraction += 0.01
            time.sleep(0.2)

        fraction = 0.5
        while fraction < 1.0:
            servo1.fraction = fraction
            fraction += 0.01
            time.sleep(0.2)

        fraction = 1.0
        while fraction > 0.5:
            servo1.fraction = fraction
            fraction -= 0.01
            time.sleep(0.2)


        self.counter_ += 1
        self.get_logger().info("Counter of rotation:" + str(self.counter_))



def main(args=None):
    rclpy.init(args=args)
    node = ServoRotationNode()
    rclpy.spin(node)
    pca.deinit()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

