import RPi.GPIO as GPIO
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from example_interfaces.msg import Int64

# GPIO Pins (BCM)
LEFT_MOTORS_FORWARD_PIN = 23
LEFT_MOTORS_BACKWARD_PIN = 24
LEFT_MOTORS_PWM_PIN = 12
RIGHT_MOTORS_FORWARD_PIN = 22
RIGHT_MOTORS_BACKWARD_PIN = 27
RIGHT_MOTORS_PWM_PIN = 13


MAX_DISTANCE = 50
MIN_DISTANCE = 35

class Motor:
    """
    Provides an interface to a DC motor connected to an L298N motor driver.
    :param int forward:
        The GPIO pin that the forward input of the motor driver chip is
        connected to.
    :param int backward:
        The GPIO pin that the backward input of the motor driver chip is
        connected to.
    :param int pwm:
        The GPIO pin that the PWM pin of the motor driver chip is
        connected to.
    """

    def __init__(self, forward_pin, backward_pin, pwm):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)
        GPIO.setup(pwm, GPIO.OUT)
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        self.pwm_pin = pwm
        self.pwm = GPIO.PWM(pwm, 100)

    def __del__(self):
        if not self.pwm == None:
            self.pwm.stop()
            GPIO.cleanup(self.pwm_pin)
        if not self.forward_pin == None:
            GPIO.cleanup(self.forward_pin)
        if not self.backward_pin == None:
            GPIO.cleanup(self.backward_pin)

    def forward(self, speed):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.backward_pin, GPIO.LOW)
        self.pwm.start(speed * 100)

    def backward(self, speed):
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.HIGH)
        self.pwm.start(speed * 100)

    def stop(self):
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.LOW)
        self.pwm.stop()



class DistanceSubscriber(Node):

    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            'LIDAR/TF02Pro',
            self.listener_callback,
            10)
        self.left_motors = Motor(forward_pin=LEFT_MOTORS_FORWARD_PIN, backward_pin=LEFT_MOTORS_BACKWARD_PIN, pwm=LEFT_MOTORS_PWM_PIN)
        self.right_motors = Motor(forward_pin=RIGHT_MOTORS_FORWARD_PIN, backward_pin=RIGHT_MOTORS_BACKWARD_PIN, pwm=RIGHT_MOTORS_PWM_PIN)
        self.speed_left = 0.65
        self.speed_right = 0.65



        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.data > MAX_DISTANCE:
            self.left_motors.forward(self.speed_left)
            self.right_motors.forward(self.speed_right)
        elif msg.data < MIN_DISTANCE:
            self.left_motors.backward(self.speed_left)
            self.right_motors.backward(self.speed_right)
        else:
            self.left_motors.stop()
            self.right_motors.stop()

        self.get_logger().info('Distance: {0}'.format(str(msg.data)))

def main(args=None):
    rclpy.init(args=args)

    distance_subscriber = DistanceSubscriber()

    rclpy.spin(distance_subscriber)

    # Destroy the node explicitl
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    distance_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

