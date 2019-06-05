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

import rclpy
from rclpy.node import Node
from mstar_msgs.srv import MStarSrv

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool

# name of the topic publishing robot 1/2 pose
ROBOT1_POSE_TOPIC = "/r1/pose"
ROBOT2_POSE_TOPIC = "/r2/pose"

# name of the topic to publish robot 1/2 path
ROBOT1_PATH_TOPIC = "/r1/path"
ROBOT2_PATH_TOPIC = "/r2/path"

# name of the topic to monitor for robot 1/2 switch pressing
ROBOT1_SWITCH_TOPIC = "/r1/switch_found"
ROBOT2_SWITCH_TOPIC = "/r2/switch_found"

# name of the MSTAR service
MSTAR_SERVICE_NAME = "/get_multi_robot_plan"

# Locations of the center of each room
ROOM_CENTERS = [[0,0], [-0.13, -1.43], [1.44, -1.18], [3.07, -1.19]]

class ExploreController(Node):

    def __init__(self):

        super().__init__('explorer_ros2')

        # Robot status
        self.robot1_done = False
        self.robot2_done = False

        # Containers for robot 1 and 2 pose
        self.robot1_pose = None
        self.robot2_pose = None

        # Containers for robot 1 and 2 seen switch
        self.robot1_seen_switch = None
        self.robot2_seen_switch = None

        # Subscribers for robot 1 and 2 pose
        self.robot1_pose_sub = self.create_subscription(PoseStamped, ROBOT1_POSE_TOPIC, lambda msg: self.update_pose(1, msg))
        self.robot2_pose_sub = self.create_subscription(PoseStamped, ROBOT2_POSE_TOPIC, lambda msg: self.update_pose(2, msg))
        
        # Publishers for robot 1 and 2 path
        self.robot1_path_pub = self.create_publisher(Path, ROBOT1_PATH_TOPIC)
        self.robot2_path_pub = self.create_publisher(Path, ROBOT2_PATH_TOPIC)

        # Subscribers for robot 1 and 2 switch found
        self.robot1_switch_sub = self.create_subscription(Bool, ROBOT1_SWITCH_TOPIC, lambda msg: self.update_switch_status(1, msg))
        self.robot2_switch_sub = self.create_subscription(Bool, ROBOT2_SWITCH_TOPIC, lambda msg: self.update_switch_status(2, msg))

        # M star server client
        self.mstar_client = self.create_client(MStarSrv, MSTAR_SERVICE_NAME)

        # Assume we start at room 0
        self.explored_rooms = [0]
        self.possible_rooms = [1,2,3]

        # List of room locations where the index of the list corresponds to the room index
        self.room_centers = ROOM_CENTERS

    def update_pose(self, robot_id, pose_msg):
        """ Updates the poses of the robot

        Args:
            robot_id (int): the robot whose pose was received
            pose_msg (Pose): Pose message

        """
        if robot_id == 1:
            self.robot1_pose = pose_msg.pose
        elif robot_id == 2:
            self.robot2_pose = pose_msg.pose
        else:
            self.get_logger().warn("Explore Controller: Invalid Robot ID passed into update_pose.")
    
    def update_switch_status(self, robot_id, switch_msg):
        """ Updates the switch status of each robot

        Args:
            robot_id (int): the robot whose switch status was received
            switch_msg (Bool): Bool msg
        """
        if robot_id == 1:
            self.robot1_seen_switch = switch_msg.data
        elif robot_id == 2:
            self.robot2_seen_switch = switch_msg.data
        else:
            self.get_logger().warn("Explore Controller: Invalid Robot ID passed into update_switch_status.")

    def construct_request_mstar(self, robot1_goal, robot2_goal):
        """ Constructs a request for the M-star service planner

        Args:
            robot1_goal (tuple(x,y)): robot 1's goal
            robot2_goal (tuple(x,y)): robot 2's goal
            
        Returns:
            request (MStarSrv): a request for a plan
        """
        while (self.robot1_pose is None):
            self.get_logger().warn("Waiting to get pose for robot 1")
            rclpy.spin_once(self)
        while (self.robot2_pose is None):
            self.get_logger().warn("Waiting to get pose for robot 2")
            rclpy.spin_once(self)

        request = MStarSrv.Request()
        request.start1_x, request.start1_y = self.robot1_pose.position.x, self.robot1_pose.position.y
        request.start2_x, request.start2_y = self.robot2_pose.position.x, self.robot2_pose.position.y
        request.goal1_x, request.goal1_y = robot1_goal
        request.goal2_x, request.goal2_y = robot2_goal

        return request

    def send_request_mstar(self, request):
        """ Queries the M-star service planner for a response

        Args:
            request (MStarSrv): a request for a plan
        
        Returns:
            robot1_path, robot2_path (Plan, Plan): plans for robots 1 and 2
        
        Notes:
            Client based on:
            https://github.com/ros2/examples/blob/master/rclpy/services/minimal_client/examples_rclpy_minimal_client/client_async_member_function.py
        """
        
        # wait for M-Star service to come up
        while not self.mstar_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("M-star service named {} not available, waiting again...".format(MSTAR_SERVICE_NAME))
        
        # Send Request
        self.future = self.mstar_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            response = self.future.result()
            self.get_logger().warn("Got response from M-star")
        else:
            self.get_logger().warn("Service call to M-star failed: {}".format(self.future.exception()))
 

        # Wait for response
        #while rclpy.ok():
        #    #rclpy.spin_once(self)
        #    if self.future.done():
        #        if self.future.result() is not None:
        #            response = self.future.result()
        #            self.get_logger().warn("Got response from M-star")
        #        else:
        #            self.get_logger().warn("Service call to M-star failed: {}".format(self.future.exception()))
        #    else:
        #        self.get_logger().warn("Waiting for response from M-star...")
        ## Parse response
        robot1_path = response.r1_path
        robot2_path = response.r2_path

        return robot1_path, robot2_path

    def publish_paths(self, robot1_path, robot2_path):
        """ Publishes the paths to robot1 and robot2

        Args:
            robot1_path, robot2_path (Plan, Plan): plans for robots 1 and 2
        """
        self.robot1_path_pub.publish(robot1_path)
        self.robot2_path_pub.publish(robot2_path)
        self.get_logger().warn("Sent paths to each robot!")
        # self.get_logger().warn("Sent paths to each robot!\n Robot 1:{}\n Robot 2:{}".format(robot1_path, robot2_path))
        return

    def wait_for_switch(self):
        """ Blocks thread, waiting for switch response from both robots """
        
        while (self.robot1_seen_switch is None) or (self.robot2_seen_switch is None):
            time.sleep(1)
            self.get_logger().warn("Waiting for both switches to be pressed: Status {}, {}".format(not self.robot1_seen_switch is None, not self.robot2_seen_switch is None))
        
        return
    
    def update_robot_status(self):
        """ Updates whether each robot has reached a switch and which rooms have been explored """

        if not self.robot1_done:
            if self.robot1_seen_switch:
                self.robot1_done = True
            else:
                self.robot1_seen_switch = None
            self.explored_rooms.append(self.robot1_room)

        if not self.robot2_done:
            if self.robot2_seen_switch:
                self.robot2_done = True
            else:
                self.robot2_seen_switch = None
            self.explored_rooms.append(self.robot2_room)

    def run(self):
        """ Handles the state machine logic to send each robot to explore separate rooms """

        self.done = False

        while not self.done:

            # Select rooms for each robot
            try:
                if not self.robot1_done:
                    self.robot1_room = self.possible_rooms.pop()
                
                if not self.robot2_done:
                    self.robot2_room = self.possible_rooms.pop()

            except:
                self.get_logger().warn("Failed to allocate rooms for robots\n\tExplored Rooms: {}")
                raise ValueError("Ran out of rooms to explore")
            
            self.get_logger().warn("Sending Robot 1 to Room {}, Sending Robot 2 to Room {}".format(self.robot1_room, self.robot2_room))

            # Create request to plan
            robot1_goal = self.room_centers[self.robot1_room]
            robot2_goal = self.room_centers[self.robot2_room]
            request = self.construct_request_mstar(robot1_goal, robot2_goal)

            # send the request to MStar and receive plans
            robot1_path, robot2_path = self.send_request_mstar(request)
            self.get_logger().error("Robot 1 Path:\n")
            for msg in robot1_path.poses:
                self.get_logger().error("Pose: {}, {}\n".format(msg.pose.position.x, msg.pose.position.y))

            self.get_logger().error("Robot 2 Path:\n")
            for msg in robot2_path.poses:
                self.get_logger().error("Pose: {}, {}\n".format(msg.pose.position.x, msg.pose.position.y))

            # send plans to each robot
            self.publish_paths(robot1_path, robot2_path)

            # block until responses received for each robot
            self.wait_for_switch()

            # update robot done status
            self.update_robot_status()

            # Check if done
            self.done = self.robot1_done and self.robot2_done

            #self.done = True # TODO: REMOVE THIS WHEN DONE

            if not self.done:
                self.get_logger().warn("Exploration in Progress:\n\tRobot 1 in Room {}, Tag Status {}\n\tRobot2 in Room {}, Tag Status {}\n\n"\
                    .format(self.robot1_room, self.robot1_done, self.robot2_room, self.robot2_done))

        self.get_logger().warn("Exploration complete!\n\tRobot 1 in Room {}\n\tRobot2 in Room {}".format(self.robot1_room, self.robot2_room))

def main(args=None):
    """ Runs an Explorer node """

    rclpy.init(args=args)
    
    explorer = ExploreController()
    
    explorer.run()
    
    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main()

