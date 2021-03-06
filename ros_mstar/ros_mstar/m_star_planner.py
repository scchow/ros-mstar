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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path, MapMetaData, OccupancyGrid
from transforms3d.euler import euler2quat
from mstar_msgs.srv import MStarSrv

import math
import numpy as np
import sys


class RobotGraphVertex:
    def __init__(self, graph_id, costmap_value, MAXCOST=float("inf")):
        self.id = graph_id
        self.costmap_value = costmap_value
        self.dist = MAXCOST
        self.optimal_policy = graph_id
        self.neighbor_ids = []


class RobotGraph:
    def __init__(self, resolution, start_x, start_y, goal_x, goal_y, occupancy_threshold, costmap_msg, USE_COSTMAP_VALUES=True, MAXCOST=float("inf"), logger=None):
        self.logger=logger
        self.costmap = None
        self.resolution = resolution
        self.start_pose = (start_x, start_y)
        self.goal_pose = (goal_x, goal_y)
        self.occupancy_threshold = occupancy_threshold
        self.use_costmap_values = USE_COSTMAP_VALUES
        self.MAXCOST = MAXCOST
        self.edge_costs = self.resolution
        self.costmap = costmap_msg.data
        self.costmap_resolution = costmap_msg.info.resolution
        self.costmap_width = costmap_msg.info.width
        self.costmap_height = costmap_msg.info.height
        self.costmap_origin_x = costmap_msg.info.origin.position.x
        self.costmap_origin_y = costmap_msg.info.origin.position.y
        self.width = int(round(self.costmap_width * (self.costmap_resolution/self.resolution)))
        self.height = int(round(self.costmap_height * (self.costmap_resolution/self.resolution)))
        self.start_id = self.convert_costmap_pose_to_graph_index(self.start_pose[0], self.start_pose[1])
        self.goal_id = self.convert_costmap_pose_to_graph_index(self.goal_pose[0], self.goal_pose[1])

        self.logger.info('Resolution: ' + str(self.resolution))
        self.logger.info('Width: ' + str(self.width))
        self.logger.info('Height: ' + str(self.height))

        # instantiate graph vertices
        self.graph = {}
        for x in range(self.width):
            for y in range(self.height):
                if not self.check_obstacles((x, y)):
                    costmap_value = self.get_costmap_value((x, y))
                    vertex = RobotGraphVertex((x, y), costmap_value, self.MAXCOST)
                    self.graph[(x, y)] = vertex

        # instantiate graph edges
        for x in range(self.width):
            for y in range(self.height):

                if (x, y) in self.graph:
                    vertex = self.graph[(x, y)]
                    # for neighbor in [(x-1, y+1), (x, y+1), (x+1, y+1), (x-1, y), (x+1, y) (x-1, y-1), (x, y-1), (x+1, y-1)]:    # 8 connect
                    for neighbor_id in [(x-1, y), (x, y+1), (x+1, y), (x, y-1)]:    # 4 connect
                        if neighbor_id in self.graph:
                            vertex.neighbor_ids.append(neighbor_id)
                    self.graph[(x, y)] = vertex

        # calculate optimal policy for each vertex
        self.compute_optimal_policy()


    def convert_costmap_pose_to_graph_index(self, x, y):
        x_index = int(round( (x - self.costmap_origin_x)/self.resolution ))
        y_index = int(round( (y - self.costmap_origin_y)/self.resolution ))
        return (x_index, y_index)

    def convert_graph_index_to_costmap_pose(self, x_index, y_index):
        x = self.costmap_origin_x + self.resolution*x_index
        y = self.costmap_origin_y + self.resolution*y_index
        return (x, y)

    def check_obstacles(self, vertex_id):
        costmap_pose = self.convert_graph_index_to_costmap_pose(*vertex_id)
        costmap_index = (int(round((costmap_pose[0] - self.costmap_origin_x)/self.costmap_resolution)), int(round((costmap_pose[1] - self.costmap_origin_y)/self.costmap_resolution)))
        left_boundary_costmap_x_index = max(int(round(((costmap_pose[0] - self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), 0)
        right_boundary_costmap_x_index = min(int(round(((costmap_pose[0] + self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), self.costmap_width-1)
        top_boundary_costmap_y_index = max(int(round(((costmap_pose[1] - self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), 0)
        bottom_boundary_costmap_y_index = min(int(round(((costmap_pose[1] + self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), self.costmap_height-1)

        highest_val = 0
        for x in range(left_boundary_costmap_x_index, right_boundary_costmap_x_index+1):
            for y in range(top_boundary_costmap_y_index, bottom_boundary_costmap_y_index+1):
                cell = x+self.costmap_width*y
                try:
                    value = self.costmap[cell]
                    if value > highest_val:
                        highest_val = value
                except IndexError:
                    self.logger.error('Indexed out of bounds!')
                    self.logger.error('vertex id: {}'.format(vertex_id))
                    self.logger.error('computed idx: {}'.format(cell))
                    self.logger.error('height: {}'.format(self.costmap_height))
                    self.logger.error('width: {}'.format(self.costmap_width))
                    self.logger.error('x: {}'.format(x))
                    self.logger.error('y: {}'.format(y))
                    self.logger.error('costmap origin x: {}'.format(self.costmap_origin_x))
                    self.logger.error('costmap origin y: {}'.format(self.costmap_origin_y))
                    sys.exit()

        if highest_val >= self.occupancy_threshold:
            return True
        else:
            return False
            

    def get_costmap_value(self, vertex_id):
        costmap_pose = self.convert_graph_index_to_costmap_pose(*vertex_id)
        costmap_index = (int(round((costmap_pose[0] - self.costmap_origin_x)/self.costmap_resolution)), int(round((costmap_pose[1] - self.costmap_origin_y)/self.costmap_resolution)))
        left_boundary_costmap_x_index = max(int(round(((costmap_pose[0] - self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), 0)
        right_boundary_costmap_x_index = min(int(round(((costmap_pose[0] + self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), self.costmap_width-1)
        top_boundary_costmap_y_index = max(int(round(((costmap_pose[1] - self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), 0)
        bottom_boundary_costmap_y_index = min(int(round(((costmap_pose[1] + self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), self.costmap_height-1)

        highest_val = 0
        for x in range(left_boundary_costmap_x_index, right_boundary_costmap_x_index+1):
            for y in range(top_boundary_costmap_y_index, bottom_boundary_costmap_y_index+1):
                cell = x+self.costmap_width*y
                try:
                    value = self.costmap[cell]
                except IndexError:
                    self.logger.error('Indexed out of bounds!')
                    self.logger.error('vertex id: {}'.format(vertex_id))
                    self.logger.error('computed idx: {}'.format(cell))
                    self.logger.error('height: {}'.format(self.costmap_height))
                    self.logger.error('width: {}'.format(self.costmap_width))
                    self.logger.error('x: {}'.format(x))
                    self.logger.error('y: {}'.format(y))
                    self.logger.error('costmap origin x: {}'.format(self.costmap_origin_x))
                    self.logger.error('costmap origin y: {}'.format(self.costmap_origin_y))
                    sys.exit()
                if value > highest_val:
                    highest_val = value

        return highest_val

    def edge_cost(self, source_vertex_id, dest_vertex_id):
        dest_vertex = self.graph[dest_vertex_id]
        if source_vertex_id == dest_vertex_id:
            return 0.0
        elif self.use_costmap_values:
            return self.edge_costs + dest_vertex.costmap_value
        else:
            return self.edge_costs


    def compute_optimal_policy(self):
        # run Dijkstra's backward to get shortest path (assume undirected graph) from goal to each node, and reverse backpointers
        # to get optimal policy for each vertex

        self.logger.info('Running Dijkstras')

        unvisited_vertices = []
        unvisited_vertex_costs = []
        unvisited_vertex_ids = []

        # instantiate goal vertex
        goal_vertex = self.graph[self.goal_id]
        goal_vertex.dist = 0
        self.graph[self.goal_id] = goal_vertex

        # add all vertices to unvisited set
        for vertex_id in self.graph.keys():
            vertex = self.graph[vertex_id]
            unvisited_vertex_ids.append(vertex_id)
            unvisited_vertex_costs.append(vertex.dist)

        # while unvisited set is not empty, pop vertex with lowest cost
        while len(unvisited_vertex_ids) != 0:
            vertex_index = np.argmin(np.array(unvisited_vertex_costs))
            vertex_id = unvisited_vertex_ids.pop(vertex_index)
            vertex_cost = unvisited_vertex_costs.pop(vertex_index)
            vertex = self.graph[vertex_id]

            # for each neighbor, check if you can get there more efficiently through popped vertex
            neighbor_ids = vertex.neighbor_ids
            for neighbor_id in neighbor_ids:
                neighbor = self.graph[neighbor_id]
                new_dist = vertex.dist + self.edge_cost(vertex_id, neighbor_id)
                if new_dist < neighbor.dist:
                    neighbor.dist = new_dist
                    neighbor.optimal_policy = vertex_id
                    self.graph[neighbor_id] = neighbor
                    if neighbor.id in unvisited_vertex_ids:
                        neighbor_index = unvisited_vertex_ids.index(neighbor_id)
                        unvisited_vertex_costs[neighbor_index] = neighbor.dist


class JointGraphNode:
    def __init__(self, graph_id, value, MAXCOST=float("inf")):
        self.id = graph_id
        self.collision_set = set()
        self.back_ptr = None
        self.back_set = []
        self.value = value
        self.cost = MAXCOST


class MStarPlanner(Node):

    def __init__(self, resolution, occupancy_threshold, costmap_topic, costmap_out_topic, mstar_service, robot_radius, heuristic, USE_COSTMAP_VALUES, MAXCOST):

        super().__init__('m_star')

        # self.resolution = self.get_parameter('resolution')._value
        # self.occupancy_threshold = self.get_parameter('occupancy_threshold')._value
        # self.costmap_topic = self.get_parameter('costmap_topic')._value
        # self.mstar_service = self.get_parameter('mstar_service')._value
        # self.robot_radius = self.get_parameter('robot_radius')._value
        # self.heuristic = self.get_parameter('heuristic')._value
        # self.USE_COSTMAP_VALUES = self.get_parameter('USE_COSTMAP_VALUES')._value
        # self.MAXCOST = self.get_parameter('MAXCOST')._value

        self.resolution = resolution
        self.occupancy_threshold = occupancy_threshold
        self.costmap_topic = costmap_topic
        self.mstar_service = mstar_service
        self.robot_radius = robot_radius
        self.heuristic = heuristic
        self.USE_COSTMAP_VALUES = USE_COSTMAP_VALUES
        self.MAXCOST = MAXCOST

        self.costmap_msg = None
        self.low_rez_costmap = None
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, costmap_out_topic)
        self.costmap_sub = self.create_subscription(OccupancyGrid, costmap_topic, self.costmap_callback)

        self.publisher_ = self.create_publisher(String, 'topic')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.srv = self.create_service(MStarSrv, self.mstar_service, self.service_callback)


    def costmap_callback(self, msg):
        self.costmap_msg = msg
        self.costmap = msg.data
        self.get_logger().info('Costmap Received!')
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin_x = msg.info.origin.position.x
        self.costmap_origin_y = msg.info.origin.position.y

        self.create_low_rez_costmap()

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def service_callback(self, request, response):
        while self.costmap_msg is None:
            self.get_logger().warn('No Costmap yet, cannot run service!')
            continue
        self.start1_pose = (request.start1_x, request.start1_y)
        self.goal1_pose = (request.goal1_x, request.goal1_y)
        self.start2_pose = (request.start2_x, request.start2_y)
        self.goal2_pose = (request.goal2_x, request.goal2_y)

        self.get_logger().info('Start 1 pose: ' + str(self.start1_pose))
        self.get_logger().info('Goal 1 pose: ' + str(self.goal1_pose))
        self.get_logger().info('Start 2 pose: ' + str(self.start2_pose))
        self.get_logger().info('Goal 2 pose: ' + str(self.goal2_pose))

        # instantiate individual graph for each robot and get optimal policy
        self.robot1 = RobotGraph(self.resolution, request.start1_x, request.start1_y, request.goal1_x, request.goal1_y, self.occupancy_threshold, self.costmap_msg, self.USE_COSTMAP_VALUES, self.MAXCOST, logger=self.get_logger())
        self.robot2 = RobotGraph(self.resolution, request.start2_x, request.start2_y, request.goal2_x, request.goal2_y, self.occupancy_threshold, self.costmap_msg, self.USE_COSTMAP_VALUES, self.MAXCOST, logger=self.get_logger())

        self.get_logger().info('Created Robot Maps')
        self.start_node_id = (self.robot1.start_id[0], self.robot1.start_id[1], self.robot2.start_id[0], self.robot2.start_id[1])
        self.goal_node_id = (self.robot1.goal_id[0], self.robot1.goal_id[1], self.robot2.goal_id[0], self.robot2.goal_id[1])

        self.get_logger().info('Start ID: ' + str(self.start_node_id))
        self.get_logger().info('Goal ID: ' + str(self.goal_node_id))

        # check if starts and goals are valid vertices
        if not((self.robot1.start_id in self.robot1.graph) and (self.robot2.start_id in self.robot2.graph) and (self.robot1.goal_id in self.robot1.graph) and (self.robot2.goal_id in self.robot2.graph)):
            response.r1_path = Path()
            response.r2_path = Path()
            self.get_logger().warn("Start or goal nodes are not valid vertices")
            return response

        # instantiate graph
        self.graph = {}

        # run m* to get paths
        (r1_path, r2_path) = self.get_plan()
        response.r1_path = r1_path
        response.r2_path = r2_path

        self.get_logger().warn("RETURNING RESPONSE FROM SERVICE!")
        return response



    def create_low_rez_costmap(self):

        width = int(round(self.costmap_width * (self.costmap_resolution/self.resolution)))
        height = int(round(self.costmap_height * (self.costmap_resolution/self.resolution)))
        # self.get_logger().info('Ratio of resolutions: {}'.format(
        #     self.costmap_resolution/self.resolution))
        # self.get_logger().info('width, height: {}, {}'.format(width, height))
        # self.get_logger().info('Other costmap width, height: {}, {}'.format(width, height))

        if self.low_rez_costmap is None:
            # instantiate graph vertices
            low_rez_costmap = []
            for y in range(height):
                for x in range(width):
                    if self.check_obstacles((x, y)):
                        low_rez_costmap.append(255)
                    else:
                        costmap_value = self.get_costmap_value((x, y))
                        low_rez_costmap.append(costmap_value)

            self.low_rez_costmap = low_rez_costmap

        occ_grid_out = OccupancyGrid()
        occ_grid_out.header.stamp = self.get_clock().now().to_msg()
        occ_grid_out.header.frame_id = self.costmap_msg.header.frame_id
        occ_grid_out.info.resolution = self.resolution
        occ_grid_out.info.width = width
        occ_grid_out.info.height = height
        occ_grid_out.info.origin = self.costmap_msg.info.origin
        occ_grid_out.data = self.low_rez_costmap

        self.occ_grid_pub.publish(occ_grid_out)

    def check_obstacles(self, vertex_id):
        costmap_pose = self.convert_graph_index_to_costmap_pose(*vertex_id)
        costmap_index = (int(round((costmap_pose[0] - self.costmap_origin_x)/self.costmap_resolution)), int(round((costmap_pose[1] - self.costmap_origin_y)/self.costmap_resolution)))
        left_boundary_costmap_x_index = max(int(round(((costmap_pose[0] - self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), 0)
        right_boundary_costmap_x_index = min(int(round(((costmap_pose[0] + self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), self.costmap_width-1)
        top_boundary_costmap_y_index = max(int(round(((costmap_pose[1] - self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), 0)
        bottom_boundary_costmap_y_index = min(int(round(((costmap_pose[1] + self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), self.costmap_height-1)

        highest_val = 0
        for x in range(left_boundary_costmap_x_index, right_boundary_costmap_x_index+1):
            for y in range(top_boundary_costmap_y_index, bottom_boundary_costmap_y_index+1):
                cell = x+self.costmap_width*y
                try:
                    value = self.costmap[cell]
                    if value > highest_val:
                        highest_val = value
                except IndexError:
                    self.get_logger().error('Indexed out of bounds!')
                    self.get_logger().error('vertex id: {}'.format(vertex_id))
                    self.get_logger().error('computed idx: {}'.format(cell))
                    self.get_logger().error('height: {}'.format(self.costmap_height))
                    self.get_logger().error('width: {}'.format(self.costmap_width))
                    self.get_logger().error('x: {}'.format(x))
                    self.get_logger().error('y: {}'.format(y))
                    self.get_logger().error('costmap origin x: {}'.format(self.costmap_origin_x))
                    self.get_logger().error('costmap origin y: {}'.format(self.costmap_origin_y))
                    sys.exit()

        if highest_val >= self.occupancy_threshold:
            return True
        else:
            return False
            

    def get_costmap_value(self, vertex_id):
        costmap_pose = self.convert_graph_index_to_costmap_pose(*vertex_id)
        costmap_index = (int(round((costmap_pose[0] - self.costmap_origin_x)/self.costmap_resolution)), int(round((costmap_pose[1] - self.costmap_origin_y)/self.costmap_resolution)))
        left_boundary_costmap_x_index = max(int(round(((costmap_pose[0] - self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), 0)
        right_boundary_costmap_x_index = min(int(round(((costmap_pose[0] + self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution)), self.costmap_width-1)
        top_boundary_costmap_y_index = max(int(round(((costmap_pose[1] - self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), 0)
        bottom_boundary_costmap_y_index = min(int(round(((costmap_pose[1] + self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution)), self.costmap_height-1)

        highest_val = 0
        for x in range(left_boundary_costmap_x_index, right_boundary_costmap_x_index+1):
            for y in range(top_boundary_costmap_y_index, bottom_boundary_costmap_y_index+1):
                cell = x+self.costmap_width*y
                try:
                    value = self.costmap[cell]
                except IndexError:
                    self.get_logger().error('Indexed out of bounds!')
                    self.get_logger().error('vertex id: {}'.format(vertex_id))
                    self.get_logger().error('computed idx: {}'.format(cell))
                    self.get_logger().error('height: {}'.format(self.costmap_height))
                    self.get_logger().error('width: {}'.format(self.costmap_width))
                    self.get_logger().error('x: {}'.format(x))
                    self.get_logger().error('y: {}'.format(y))
                    self.get_logger().error('costmap origin x: {}'.format(self.costmap_origin_x))
                    self.get_logger().error('costmap origin y: {}'.format(self.costmap_origin_y))
                    sys.exit()
                if value > highest_val:
                    highest_val = value

        return highest_val


    def convert_costmap_pose_to_graph_index(self, x, y):
        x_index = int(round( (x - self.costmap_origin_x)/self.resolution ))
        y_index = int(round( (y - self.costmap_origin_y)/self.resolution ))
        return (x_index, y_index)

    def convert_graph_index_to_costmap_pose(self, x_index, y_index):
        x = self.costmap_origin_x + self.resolution*x_index
        y = self.costmap_origin_y + self.resolution*y_index
        return (x, y)



    def get_node_from_id(self, graph_id):
        if graph_id in self.graph:
            return self.graph[graph_id]
        else:
            value = self.robot1.graph[(graph_id[0], graph_id[1])].costmap_value + self.robot2.graph[(graph_id[2], graph_id[3])].costmap_value
            node = JointGraphNode(graph_id, value, self.MAXCOST)
            self.graph[graph_id] = node
            return self.graph[graph_id]


    def is_goal(self, node_id):
        self.get_logger().info('Checking goal: ' + str(node_id) + ' == ' +  str(self.goal_node_id))
        return node_id == self.goal_node_id


    def check_collisions(self, node_id):

        (r1_x, r1_y) = self.robot1.convert_graph_index_to_costmap_pose(node_id[0], node_id[1])
        (r2_x, r2_y) = self.robot2.convert_graph_index_to_costmap_pose(node_id[2], node_id[3])

        collision_set = set()
        if (math.sqrt((r2_x - r1_x)**2 + (r2_y - r1_y)**2) <= 2*self.robot_radius):
            collision_set.add((1, 2))
        
        return collision_set



    def check_transition_collides(self, curr_r1_id, curr_r2_id, next_vertex_1_id, next_vertex_2_id):

        if len(self.check_collisions((next_vertex_1_id[0], next_vertex_1_id[1], next_vertex_2_id[0], next_vertex_2_id[1]))) != 0:
            return []

        elif len(self.check_collisions((curr_r1_id[0], curr_r1_id[1], next_vertex_2_id[0], next_vertex_2_id[1]))) != 0:
            return [(curr_r1_id[0], curr_r1_id[1], next_vertex_2_id[0], next_vertex_2_id[1])]

        elif len(self.check_collisions((next_vertex_1_id[0], next_vertex_1_id[1], curr_r2_id[0], curr_r2_id[1]))) != 0:
            return [(next_vertex_1_id[0], next_vertex_1_id[1], curr_r2_id[0], curr_r2_id[1])]

        else:
            return []


    def get_neighbor_ids(self, node_id):

        self.get_logger().info('Looking up neighbors')

        node = self.graph[node_id]
        curr_r1_id = (node_id[0], node_id[1])
        curr_r2_id = (node_id[2], node_id[3])

        if len(node.collision_set) == 0:
            self.get_logger().info('Following optimal policy')
            next_vertex_1_id = self.robot1.graph[curr_r1_id].optimal_policy
            next_vertex_2_id = self.robot2.graph[curr_r2_id].optimal_policy

            self.get_logger().info('R1 optimal policy: ' + str(next_vertex_1_id))
            self.get_logger().info('R2 optimal policy: ' + str(next_vertex_2_id))

            transition_collisions = self.check_transition_collides(curr_r1_id, curr_r2_id, next_vertex_1_id, next_vertex_2_id)
            if len(transition_collisions) == 0:
                self.get_logger().info('Transition looks good')
                return [(next_vertex_1_id[0], next_vertex_1_id[1], next_vertex_2_id[0], next_vertex_2_id[1])]

            else:
                self.get_logger().info('Transition collides')
                return transition_collisions

        # generate pairwise combinations of neighboring states
        else:
            self.get_logger().info('Collisions, expanding search')
            r1_neighbors = self.robot1.graph[curr_r1_id].neighbor_ids
            r2_neighbors = self.robot2.graph[curr_r2_id].neighbor_ids
            r1_neighbors.append(curr_r1_id)
            r2_neighbors.append(curr_r2_id)

            neighbors = []
            for r1_neighbor in r1_neighbors:
                for r2_neighbor in r2_neighbors:
                    if not ((r1_neighbor == curr_r1_id) and (r2_neighbor == curr_r2_id)):
                        if len(self.check_transition_collides(curr_r1_id, curr_r2_id, r1_neighbor, r2_neighbor)) == 0:
                            neighbors.append((r1_neighbor[0], r1_neighbor[1], r2_neighbor[0], r2_neighbor[1]))
            return neighbors



    def back_track_helper(self, node_id):
        node = self.get_node_from_id(node_id)
        if node.back_ptr is None:
            back_trace = [node_id]
            return back_trace
        else:
            back_trace = [node_id] + self.back_track_helper(node.back_ptr)
            return back_trace

    def back_track(self, node_id):
        self.get_logger().info('Goal found, backtracking')
        back_trace = self.back_track_helper(node_id)
        back_trace.reverse()
        self.get_logger().info('backtrace: ' + str(back_trace))
        r1_plan = []
        r2_plan = []
        for trace_node_id in back_trace:
            r1_plan.append(self.robot1.convert_graph_index_to_costmap_pose(trace_node_id[0], trace_node_id[1]))
            r2_plan.append(self.robot2.convert_graph_index_to_costmap_pose(trace_node_id[2], trace_node_id[3]))
        r1_path = self.convert_plan_to_path(r1_plan)
        r2_path = self.convert_plan_to_path(r2_plan)
        return (r1_path, r2_path)


    def backprop(self, node_id, collision_set):
        self.get_logger().info('Running backprop')
        node = self.get_node_from_id(node_id)
        ADDED_COLLISIONS = False
        for collision in collision_set:
            if collision not in node.collision_set:
                node.collision_set.add(collision)
                ADDED_COLLISIONS = True
                self.graph[node_id] = node

        if ADDED_COLLISIONS:
            if node_id not in self.open_set_ids:
                self.open_set_ids.append(node_id)
                self.open_set_costs.append(node.cost + self.heuristic_function(node_id))
            for back_node_id in node.back_set:
                self.backprop(back_node_id, node.collision_set)

    def heuristic_function(self, node_id):

        (curr_r1_x, curr_r1_y) = self.robot1.convert_graph_index_to_costmap_pose(node_id[0], node_id[1])
        (curr_r2_x, curr_r2_y) = self.robot2.convert_graph_index_to_costmap_pose(node_id[2], node_id[3])

        (goal_r1_x, goal_r1_y) = self.goal1_pose
        (goal_r2_x, goal_r2_y) = self.goal2_pose

        if self.heuristic == 'l1':
            r1 = abs(goal_r1_x - curr_r1_x) + abs(goal_r1_y - curr_r1_y)
            r2 = abs(goal_r2_x - curr_r2_x) + abs(goal_r2_y - curr_r2_y)
            return r1 + r2

        # otherwise assume l2
        else:
            r1 = math.sqrt((goal_r1_x - curr_r1_x)**2 + (goal_r1_y - curr_r1_y)**2)
            r2 = math.sqrt((goal_r2_x - curr_r2_x)**2 + (goal_r2_y - curr_r2_y)**2)
            return r1 + r2


    def edge_cost(self, source, dest):
        return self.robot1.edge_cost((source[0], source[1]), (dest[0], dest[1])) + self.robot2.edge_cost((source[2], source[3]), (dest[2], dest[3]))


    def get_plan(self):
        self.open_set_ids = []
        self.open_set_costs = []

        # initialize start node
        start_node = self.get_node_from_id(self.start_node_id)
        start_node.cost = 0.0
        self.graph[self.start_node_id] = start_node

        self.open_set_ids.append(self.start_node_id)
        self.open_set_costs.append(start_node.cost + self.heuristic_function(self.start_node_id))

        while len(self.open_set_ids) != 0:

            lowest_index = np.argmin(np.array(self.open_set_costs))
            node_id = self.open_set_ids.pop(lowest_index)
            node_cost = self.open_set_costs.pop(lowest_index)
            node = self.get_node_from_id(node_id)
            if self.is_goal(node_id):
                return self.back_track(node_id)

            if self.check_collisions(node_id):
                continue

            neighbor_ids = self.get_neighbor_ids(node_id)
            for neighbor_id in neighbor_ids:
                neighbor = self.get_node_from_id(neighbor_id)
                neighbor.back_set.append(node_id)
                neighbor.collision_set = neighbor.collision_set.union(self.check_collisions(neighbor_id))
                self.graph[neighbor_id] = neighbor
                self.backprop(node_id, neighbor.collision_set)

                node = self.get_node_from_id(node_id)
                neighbor = self.get_node_from_id(neighbor_id)
                if node.cost + self.edge_cost(node_id, neighbor_id) < neighbor.cost:
                    neighbor.cost = node.cost + self.edge_cost(node_id, neighbor_id)
                    if neighbor.id in self.open_set_ids:
                        neighbor_index = self.open_set_ids.index(neighbor_id)
                        self.open_set_costs[neighbor_index] = neighbor.cost + self.heuristic_function(neighbor_id)
                    neighbor.back_ptr = node_id
                    self.graph[neighbor_id] = neighbor
                if neighbor_id not in self.open_set_ids:
                    self.open_set_ids.append(neighbor_id)
                    self.open_set_costs.append(neighbor.cost + self.heuristic_function(neighbor_id))

        self.get_logger().warn("M* did not find a valid path combination")
        return (Path(), Path())
                    

    def convert_plan_to_path(self, plan):
        poses = []
        prev_waypoint = plan[0]
        if len(plan) > 1:
            for waypoint in plan[1:]:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = self.costmap_msg.header.frame_id
                pose_stamped.pose.position.x = waypoint[0]
                pose_stamped.pose.position.y = waypoint[1]
                pose_stamped.pose.position.z = 0.0

                if (prev_waypoint[0] == waypoint[0]) and (prev_waypoint[1] == waypoint[1]):
                    yaw = 0.0
                else:
                    yaw = math.atan2(waypoint[1] - prev_waypoint[1], waypoint[0] - prev_waypoint[0])
                pose_stamped.pose.orientation = heading(yaw)
                poses.append(pose_stamped)
                prev_waypoint = waypoint
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.costmap_msg.header.frame_id
        path.poses = poses
        return path


def heading(yaw):
    """A helper function to getnerate quaternions from yaws."""
    q = euler2quat(0.0, 0.0, yaw)
    quat = Quaternion()
    quat.w = q[0]
    quat.x = q[1]
    quat.y = q[2]
    quat.z = q[3]
    return quat


def main(args=sys.argv[1:]):

    rclpy.init(args=args)

    resolution = float(args[0])
    occupancy_threshold = float(args[1])
    costmap_topic = args[2]
    costmap_out_topic = args[3]
    mstar_service = args[4]
    robot_radius = float(args[5])
    heuristic = args[6]
    USE_COSTMAP_VALUES = bool(int(args[7]))
    MAXCOST = float(args[8])

    m_star = MStarPlanner(resolution, occupancy_threshold, costmap_topic, costmap_out_topic, mstar_service, robot_radius, heuristic, USE_COSTMAP_VALUES, MAXCOST)

    rclpy.spin(m_star)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    m_star.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
