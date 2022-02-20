# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.msg import Int64

import serial
import time

class DistancePublisher(Node):

    interface_lidar = serial.Serial('/dev/ttyUSB0',115200)

    def __init__(self):
        super().__init__('distance_publisher')
        self.publisher_ = self.create_publisher(Int64, 'LIDAR/TF02Pro', 10)
        timer_period = 0.01  # seconds
        lidar_enableOutput  = [0x5a,0x05,0x07,0x01,0x00]

        if self.interface_lidar.isOpen() :
            self.get_logger().info("DE-LIDAR TF02-Pro sucessfully opened")
            self.interface_lidar.write(lidar_enableOutput)
            time.sleep(0.01)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        else :
            self.get_logger().info("could not open DE-LIDAR TF02-Pro, won't publish anything.")


    def timer_callback(self):
        counter = self.interface_lidar.in_waiting
        bytes_to_read = 9
        if counter > bytes_to_read-1:
            bytes_serial = self.interface_lidar.read(bytes_to_read) 
            self.interface_lidar.reset_input_buffer() # reset buffer

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
                distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
                strength = bytes_serial[4] + bytes_serial[5]*256 # signal strength in next two bytes
                temperature = bytes_serial[6] + bytes_serial[7]*256 # temp in next two bytes
                temperature = (temperature/8) - 256 # temp scaling and offset

                msg = Int64()
                msg.data = distance
                #   f"Strength: {strength:2.0f} / 65535 (16-bit), "
                #   f"Chip Temperature: {temperature:2.1f} C")
                self.publisher_.publish(msg)
                self.get_logger().info('Distance: {0} cm'.format(str(msg.data)))


def main(args=None):
    rclpy.init(args=args)

    distance_publisher = DistancePublisher()

    rclpy.spin(distance_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
