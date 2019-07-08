#!/usr/bin/env python
#  Copyright (c) 2019 Scott Chow, Connor Yates, Christopher Bollinger, Christopher Eriksen
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.

import time
import sys

import rclpy
from rclpy.node import Node
from mstar_msgs.srv import MStarSrv

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool

class ROS1TopicSink(Node):

    def __init__(self, robot_name):

        super().__init__('{}_sink'.format(robot_name))

        self.get_logger().info("Subscribing to topics for robot {}".format(robot_name))
        self.create_subscription(PoseStamped, '/{}/pose'.format(robot_name), lambda msg: None)
        self.create_subscription(Bool, '/{}/switch_found'.format(robot_name), lambda msg: None)
        self.create_subscription(OccupancyGrid, '/{}/costmap'.format(robot_name), lambda msg: None)
        self.create_subscription(LaserScan, '/{}/scan'.format(robot_name), lambda msg: None)
        self.create_subscription(TFMessage, '/{}/tf'.format(robot_name), lambda msg: None)
        self.create_subscription(TFMessage, '/{}/tf_static'.format(robot_name), lambda msg: None)
        self.create_subscription(Twist, '/{}/cmd_vel'.format(robot_name), lambda msg: None)
        self.create_subscription(OccupancyGrid, '/{}/map'.format(robot_name), lambda msg: None)
        self.create_subscription(Odometry, '/{}/odom'.format(robot_name), lambda msg: None)
        self.create_subscription(Image, '/{}/usb_cam/raw_image'.format(robot_name), lambda msg: None)
        self.create_subscription(CameraInfo, '/{}/usb_cam/camera_info'.format(robot_name), lambda msg: None)
        

def main(args=sys.argv[1:]):
    """ Runs an Explorer node """

    rclpy.init(args=args)
    
    sink = ROS1TopicSink(args[0])
    
    rclpy.spin(sink)

    sink.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main()

