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

        # calculate optimal policy for each vertex
        self.compute_optimal_policy()


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


class JointGraphNode:
    def __init__(self, graph_id, MAXCOST=float("inf")):
        self.id = graph_id
        self.collision_set = set()
        self.back_ptr = None
        self.back_set = []
        self.value = self.robot1.graph[(graph_id[0], graph_id[1])].costmap_value + self.robot2.graph[(graph_id[2], graph_id[3])].costmap_value
        self.cost = MAXCOST


class MStarPlanner(Node):

    def __init__(self, resolution, start1_x, start1_y, start2_x, start2,_y, goal1_x, goal1_y, goal2_x, goal2_y, occupancy_threshold, costmap_topic, robot_radius, heuristic='l2', 
        USE_COSTMAP_VALUES, MAXCOST=float("inf")):

        super().__init__('m_star')
        self.publisher_ = self.create_publisher(String, 'topic')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.resolution = resolution
        self.start1_pose = (start1_x, start1_y)
        self.goal1_pose = (goal1_x, goal1_y)
        self.start2_pose = (start2_x, start2_y)
        self.goal2_pose = (goal2_x, goal2y)
        self.occupancy_threshold - occupancy_threshold
        self.costmap_topic = costmap_topic
        self.robot_radius = robot_radius
        self.heuristic = heuristic
        self.MAXCOST = MAXCOST

        # instantiate individual graph for each robot and get optimal policy
        self.robot1 = RobotGraph(resolution, start1_x, start1_y, goal1_x, goal1_y, occupancy_threshold, costmap_topic, USE_COSTMAP_VALUES, MAXCOST)
        self.robot2 = RobotGraph(resolution, start2_x, start2_y, goal2_x, goal2_y, occupancy_threshold, costmap_topic, USE_COSTMAP_VALUES, MAXCOST)

        self.width = robot1.width
        self.height = robot1.height
        self.start_node_id = (robot1.start_id[0], robot1.start_id[1], robot2.start_id[0], robot2.start_id[1])
        self.goal_node_id = (robot1.goal_id[0], robot1.goal_id[1], robot2.goal_id[0], robot2.goal_id[1])

        # instantiate graph
        self.graph = {}

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



    def get_node_from_id(self, graph_id):
        if graph_id in self.graph:
            return self.graph[graph_id]
        else:
            node = JointGraphNode(graph_id)
            self.graph[graph_id] = node
            return node


    def is_goal(self, node_id):
        return node_id == self.goal_node_id


    def check_collisions(self, node):

        (r1_x, r1_y) = self.robot1.convert_graph_index_to_costmap_pose(self, node_id[0], node_id[1])
        (r2_x, r2_y) = self.robot2.convert_graph_index_to_costmap_pose(self, node_id[2], node_id[3])

        collision_set = set()
        if (math.sqrt((r2_x - r1_x)**2 + (r2_y - r1_y)**2) <= 2*self.robot_radius):
            set.add((1, 2))
        
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

        node = self.graph[node_id]
        curr_r1_id = (node_id[0], node_id[1])
        curr_r2_id = (node_id[2], node_id[3])

        if len(node.collision_set) == 0:
            next_vertex_1_id = self.robot1.graph[curr_r1_id].optimal_policy
            next_vertex_2_id = self.robot2.graph[curr_r2_id].optimal_policy

            transition_collisions = self.check_transition_collides(curr_r1_id, curr_r2_id, next_vertex_1_id, next_vertex_2_id)
            if len(transition_collisions) == 0:
                return [(next_vertex_1_id[0], next_vertex_1_id[1], next_vertex_2_id[0], next_vertex_2_id[1])]

            else:
                return transition_collisions

        # generate pairwise combinations of neighboring states
        else:
            r1_neighbors = self.robot1.graph[curr_r1_id].neighbors
            r2_neighbors = self.robot2.graph[curr_r2_id].neighbors
            r1_neighbors.append(curr_r1_id)
            r2_neighbors.append(curr_r2_id)

            neighbors = []
            for r1_neighbor in r1_neighbors:
                for r2_neighbor in r2_neighbors:
                    if not ((r1_neighbor == curr_r1_id) and (r2_neighbor == curr_r2_id)):
                        if len(self.check_transition_collides(curr_r1_id, curr_r2_id, r1_neighbor, r2_neighbor)) == 0:
                            neighbors.append((r1_neighbor[0], r1_neighbor[1], r2_neighbor[0], r2_neighbor[1]))
            return neighbors



    def back_track(self, node_id):
        node = self.get_node_from_id(node_id)
        if node.back_ptr is None:
            return [node_id]
        else:
            trace = [node_id] + self.back_track(node.back_ptr)
            back_trace =  trace.reverse()

            r1_plan = []
            r2_plan = []
            for trace_node_id in back_trace:
                r1_plan.append(self.robot1.convert_graph_index_to_costmap_pose(trace_node_id[0], trace_node_id[1]))
                r2_plan.append(self.robot2.convert_graph_index_to_costmap_pose(trace_node_id[2], trace_node_id[3]))
            return (r1_plan, r2_plan)


    def backprop(self, node_id, collision_set):
        node = self.get_node_from_id(node_id)
        ADDED_COLLISIONS = False
        for collision in collision_set:
            if collision not in node.collision_set:
                node.collision_set = node.collision_set.add(collision)
                ADDED_COLLISIONS = True

        if ADDED_COLLISIONS:
            if node_id not in self.open_set_ids:
                self.open_set_ids.append(node_id)
                self.open_set_costs.append(node.cost + heuristic_function(node_id))
            for back_node_id in node.back_set:
                backprop(back_node_id, node.collision_set)


    def self.heuristic_function(node_id):

        (curr_r1_x, curr_r1_y) = self.robot1.convert_graph_index_to_costmap_pose(self, node_id[0], node_id[1])
        (curr_r2_x, curr_r2_y) = self.robot2.convert_graph_index_to_costmap_pose(self, node_id[2], node_id[3])

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
        start_node = get_node_from_id(start_node_id)
        start_node.cost = 0
        self.graph[start_node_id] = start_node

        self.open_set_ids.append(self.start_node_id)
        self.open_set_costs.append(start_node.cost + self.heuristic_function(start_node_id))

        while len(self.open_set) != 0:

            lowest_index = np.argmin(np.array(self.open_set_costs))
            node_id = self.open_set_ids.pop(lowest_index)
            node_cost = self.open_set_costs.pop(lowest_index)
            node = get_node_from_id(node_id)
            if self.is_goal(node_id):
                return self.back_track(node_id)

            if self.check_collsions(node_id) is not empty:
                continue


            neighbor_ids = self.get_neighbor_ids(node_id):
            for neighbor_id in neighbor_ids:
                neighbor = get_node_from_id(neighbor_id)
                neighbor.back_set.append(node_id)
                neighbor.collision_set = neighbor.collision_set.union(self.check_collisions(neighbor_id))
                self.graph[neighbor_id] = neighbor
                backprop(node_id, neighbor.collision_set)

                if node.cost + self.edge_cost(node_id, neighbor_id) < neighbor.cost:
                    neighbor.cost = self.edge_cost(node_id, neighbor_id)
                    if neighbor.id in open_set_ids:
                        neighbor_index = self.open_set_ids.index(neighbor_id)
                        self.open_set_costs[neighbor_index] = neighbor.cost + self.heuristic_function(start_node_id)
                    neighbor.back_ptr = node_id
                    self.graph[neighbor_id] = neighbor

                    

def convert_plan_to_waypoints(plan):
    ## FIXME


def main(args=None):
    # read in param for graph resolution         ## FIXME
    # read in param for robot1 start pose        ## FIXME
    # read in param for robot1 goal pose         ## FIXME
    # read in param for robot2 start pose        ## FIXME
    # read in param for robot2 goal pose         ## FIXME

    rclpy.init(args=args)

    m_star = MStarPlanner()
    m_star = MStarPlanner(resolution, start1_x, start1_y, start2_x, start2,_y, goal1_x, goal1_y, goal2_x, goal2_y, occupancy_threshold, costmap_topic, robot_radius, heuristic, 
        USE_COSTMAP_VALUES, MAXCOST)

    (r1_plan, r2_plan) = m_star.get_plan()
    r1_waypoints = convert_plan_to_waypoints(r1_plan)
    r2_waypoints = convert_plan_to_waypoints(r2_plan)

    rclpy.spin(m_star)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    m_star.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
