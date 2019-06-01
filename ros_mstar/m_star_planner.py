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
from nav_msgs.msg import MapMetaData, OccupancyGrid


class RobotGraphVertex:
    def __init__(self, graph_id, costmap_value, MAXCOST=float("inf")):
        self.id = graph_id
        self.costmap_value = costmap_value
        self.dist = MAXCOST
        self.optimal_policy = None
        self.neighbor_ids = []


class RobotGraph:
    def __init__(self, resolution, start_x, start_y, goal_x, goal_y, occupancy_threshold, costmap_topic, USE_COSTMAP_VALUES=True, MAXCOST=float("inf")):
        self.costmap = None
        self.resolution = resolution
        self.start_pose = (start_x, start_y)
        self.goal_pose = (goal_x, goal_y)
        self.occupancy_threshold = occupancy_threshold
        self.use_costmap_values = USE_COSTMAP_VALUES
        self.MAXCOST = MAXCOST
        self.edge_costs = self.resolution
        costmap_sub = rospy.Subscriber(costmap_topic, OccupancyGrid, self.costmap_callback)         ## FIXME update to ros2

        while self.costmap is None:
            continue

        self.width = int(round(self.costmap_width * (self.costmap_resolution/self.resolution)))
        self.height = int(round(self.costmap_height * (self.costmap_resolution/self.resolution)))
        self.start_id = self.convert_costmap_pose_to_graph_index(self.start_pose[0], self.start_pose[1])
        self.goal_id = self.convert_costmap_pose_to_graph_index(self.goal_pose[0], self.goal_pose[1])

        # instantiate graph vertices
        self.graph = {}
        for r in range(self.width):
            for c in range(self.height):
                if not self.check_obstacles((r, c)):
                    costmap_value = self.get_costmap_value((r, c))
                    vertex = RobotGraphVertex((r, c), costmap_value, self.MAXCOST)
                    self.graph[(r, c)] = vertex

        # instantiate graph edges
        for r in range(self.width):
            for c in range(self.height):

                if (r, c) in self.graph:
                    vertex = self.graph[(r, c)]
                    # for neighbor in [(r-1, c+1), (r, c+1), (r+1, c+1), (r-1, c), (r+1, c) (r-1, c-1), (r, c-1), (r+1, c-1)]:    # 8 connect
                    for neighbor_id in [(r-1, c+1), (r+1, c+1), (r-1, c-1), (r+1, c-1)]:    # 4 connect
                        if neighbor_id in self.graph:
                            vertex.neighbor_ids.append(neighbor_id)
                    self.graph[(r, c)] = vertex


    ## FIXME
    def costmap_callback(self, msg):
        # self.costmap = 
        # self.costmap_width =
        # self.costmap_height =
        # self.costmap_resolution =
        # self.costmap_origin_x = 
        # self.costmap_origin_y = 

    def convert_costmap_pose_to_graph_index(self, x, y):
        x_index = int(round( (x - self.costmap_origin_x)*(self.costmap_resolution/self.resolution) ))
        y_index = int(round( (y - self.costmap_origin_y)*(self.costmap_resolution/self.resolution) ))
        return (x_index, y_index)

    def convert_graph_index_to_costmap_pose(self, x_index, y_index):
        x = self.costmap_origin_x + (self.resolution/self.costmap_resolution)*x_index
        y = self.costmap_origin_y + (self.resolution/self.costmap_resolution)*y_index
        return (x, y)

    def check_obstacles(self, vertex_id):
        costmap_pose = self.convert_graph_index_to_costmap_pose(vertex_id)
        left_boundary_costmap_x_index = int(round(((costmap_pose[0] - self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution))
        right_boundary_costmap_x_index = int(round(((costmap_pose[0] + self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution))
        top_boundary_costmap_y_index = int(round(((costmap_pose[1] - self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution))
        bottom_boundary_costmap_y_index = int(round(((costmap_pose[1] + self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution))

        highest_val = 0
        for x in range(left_boundary_costmap_x_index, right_boundary_costmap_x_index+1):
            for y in range(top_boundary_costmap_y_index, bottom_boundary_costmap_y_index+1):
                cell = x*self.costmap_width + y
                value = self.costmap[cell]
                if value > highest_val:
                    highest_val = value

        if highest_val >= self.occupancy_threshold:
            return True

    def get_costmap_value(self, vertex_id):
        costmap_pose = self.convert_graph_index_to_costmap_pose(vertex_id)
        left_boundary_costmap_x_index = int(round(((costmap_pose[0] - self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution))
        right_boundary_costmap_x_index = int(round(((costmap_pose[0] + self.resolution/2.0) - self.costmap_origin_x)/self.costmap_resolution))
        top_boundary_costmap_y_index = int(round(((costmap_pose[1] - self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution))
        bottom_boundary_costmap_y_index = int(round(((costmap_pose[1] + self.resolution/2.0) - self.costmap_origin_y)/self.costmap_resolution))

        highest_val = 0
        for x in range(left_boundary_costmap_x_index, right_boundary_costmap_x_index+1):
            for y in range(top_boundary_costmap_y_index, bottom_boundary_costmap_y_index+1):
                cell = x*self.costmap_width + y
                value = self.costmap[cell]
                if value > highest_val:
                    highest_val = value

        return highest_val

    def edge_cost(self, source_vertex_id, dest_vertex_id):
        dest_vertex =self.graph[dest_vertex_id]
        if self.use_costmap_values:
            return self.edge_costs + dest_vertex.costmap_value
        else:
            return self.edge_costs


    def compute_optimal_policy(self):
        # run Dijkstra's backward to get shortest path (assume undirected graph) from goal to each node, and reverse backpointers
        # to get optimal policy for each vertex

        unvisited_vertices = []
        unvisited_vertex_costs = []

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
                    neighbor.optimal_policy = vertex
                    self.graph[neighbor_id] = neighbor
                    if neighbor.id in unvisited_vertex_ids:
                        neighbor_index = unvisited_vertex_ids.index(neighbor_id)
                        unvisited_vertex_costs[neighbor_index] = neighbor.dist


class JoinitGraphNode:
    def __init__(self, MAXCOST=float("inf")):
        self.collision_set = set()
        self.back_ptr = None
        self.back_set = []
        self.cost = MAXCOST


class JointGraph:
    def __init__(self):
        #

    def is_goal(self, node):
        # check if node is the goal node

    def check_collisions(self, node):     ## FIXME
        # for a given node, check if any robots collide
        # return set of colliding robots or empty set if no robots collide


    def get_neighbors(self, node):        ## FIXME
        # get neihbors of node that are in explorable states
            # such that transitions between node and neighbor do not collide with map or each other (robots)

    def back_track(self, node):
        if node.back_ptr is None:
            return [node]
        else:
            trace = [node] + self.back_track(node.back_ptr)
            return trace.reverse()

    def backprop(self, node_id, collision_set):        ## FIXME
        node = self.get_node_from_id(node_id)
        ADDED_COLLISIONS = False
        for collision in collision_set:
            if collision not in node.collision_set:
                node.collision_set = node.collision_set.add(collision)
                ADDED_COLLISIONS = True

        if ADDED_COLLISIONS:
            if node.id not in self.open_set_ids:
                self.open_set_ids.append(node.id)
                self.open_set_costs.append(node.cost + heuristic_function(node))
            for back_node in node.back_set:
                backprop(back_node.id, node.collision_set)


class MStarPlanner(Node):

    def __init__(self, MAXCOST=float("inf")):
        super().__init__('m_star')
        self.publisher_ = self.create_publisher(String, 'topic')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.MAXCOST = MAXCOST
        self.create_graphs()

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def create_graphs(self):        ## FIXME

        # robot map   G_r
        ## create row x col sized graph, where the value of each node is the value in the costmap

        ## 4 connect each node if the straight line between the 2 nodes does not collide with an obstacle in the map

        ## associate start and goal pose of robot1 with nodes
        ## associate start and goal pose of robot2 with nodes


        # joint graph  G
        ## create G_r x G_r sized graph where value of G(r1, r2) = G_r(r1) + G_r(r2)
        ## set start and goal nodes


    def get_plan(self):
        self.open_set_ids = []
        self.open_set_costs = []

        self.create_graphs()

        # compute optimal policy for each node in joint graph
        self.compute_optimal_policy()

        ## start_node.id = 
        start_node.back_ptr = 0
        start_node.cost = 0
        self.open_set_ids.append(start_node.id)
        self.open_set_costs.append(start_node.cost + heuristic_function(start_node))

        while len(self.open_set) != 0:

            lowest_index = np.argmin(np.array(self.open_set_costs))
            node_id = self.open_set_ids.pop(lowest_index)
            node_cost = self.open_set_costs.pop(lowest_index)
            node = self.get_node_from_id(node_id)
            if self.is_goal(node):
                return self.back_track(node)

            # if check_collsions(node) is not empty:
                # continue


            # neighbors = get_neighbors(node):
            for neighbor in neighbors:
                neighbor.back_set.append(node)
                neighbor.collision_set = neighbor.collision_set.union(check_collisions(neighbor))
                backprop(node.id, neighbor.collision_set)

                # if node.cost + f(edge_node_neighbor) < neighbor.cost:
                    # neighbor.cost = node.cost + f(edge_node_neighbor)
                    if neighbor.id in open_set_ids:
                        neighbor_index = self.open_set_ids.index(neighbor.id)
                        self.open_set_costs[neighbor_index] = neighbor.cost + heuristic_function(start_node)
                    neighbor.back_ptr = node



def main(args=None):
    # read in param for graph resolution         ## FIXME
    # read in param for robot1 start pose        ## FIXME
    # read in param for robot1 goal pose         ## FIXME
    # read in param for robot2 start pose        ## FIXME
    # read in param for robot2 goal pose         ## FIXME

    rclpy.init(args=args)

    m_star = MStarPlanner()
    plan = m_star.get_plan()

    rclpy.spin(m_star)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    m_star.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
