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
from ros_mstar.srv import MStarSrv

import sys

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(MStarSrv, mstar_service)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MStarSrv.Request()

    def send_request(self, start1_x, start1_y, goal1_x, goal1_y, start2_x, start2_y, goal2_x, goal2y):
        req.start1_x = start1_x
        req.start1_y = start1_y
        req.goal1_x = goal1_x
        req.goal1_y = goal1_y
        req.start2_x = start2_x
        req.start2_y = start2_y
        req.goal2_x = goal2_x
        req.goal2y = goal2y
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    mstar_service = args[0]
    start1_x = float(args[1])
    start1_y = float(args[2])
    goal1_x = float(args[3])
    goal1_y = float(args[4])
    start2_x = float(args[5])
    start2_y = float(args[6])
    goal2_x = float(args[7])
    goal2y = float(args[8])

    minimal_client = MinimalClientAsync()
    minimal_client.send_request(start1_x, start1_y, goal1_x, goal1_y, start2_x, start2_y, goal2_x, goal2y)

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            if minimal_client.future.result() is not None:
                response = minimal_client.future.result()
                minimal_client.get_logger().info(
                    "Path 1: " + str(response.r1_path))
                minimal_client.get_logger().info(
                    "Path 2: " + str(response.r2_path))
            else:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (minimal_client.future.exception(),))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])