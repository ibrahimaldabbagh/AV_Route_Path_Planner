import math
from heapq import heappop, heappush # Import heap functions for priority queue
import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3
from std_msgs.msg import Bool
import time
import csv
from datetime import datetime
class RouteComputerCardinal(Node):
    """
    RouteComputer calculates and publishes a route using the A* algorithm.
    It considers vehicle and target coordinates received through subscriptions,
    and adheres to directional constraints specified in the map data.
    """

    def __init__(self):
        super().__init__('route_computer_cardinal')
        self.get_logger().info("Initializing RouteComputerCardinal")

        # Publisher to send the calculated route
        self.route_publisher_ = self.create_publisher(PoseArray, '/route', 10)

         # Publisher for behavoural planning state machine
        self.route_state_publisher_ = self.create_publisher(Bool,
                                                            '/route_state',
                                                            10) # topic for behvioural planning
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription_spot = self.create_subscription(PoseStamped,
                                                           '/selected_spot',
                                                             self.spot_location_callback,
                                                               10)
        self.subscription_loc = self.create_subscription(PoseStamped,
                                                         '/loc_pose',
                                                         self.location_pose_callback,
                                                         10)
        self.subscription_orient = self.create_subscription(Vector3,
                                                            '/euler_angles',
                                                            self.euler_angles_callback,
                                                            10) # for getting the Yaw of the EV

    # Flags and variables for route calculation
        self.send_route = False
        self.route_state = False
        self.latest_localization = None
        self.vehicle_location = None
        self.target_coordinate = None
        self.vehicle_orientation = None
        self.nodes = []
        self.edges = {}
        self.parking_spots = []
        package_share_directory = get_package_share_directory('adapt_roucomp')
        map_file = os.path.join(package_share_directory, 'config', 'map_cardinal.txt')
        self.load_map(map_file)

        # Initialize metrics logging
        self.initialize_metrics_logging()

    def initialize_metrics_logging(self):
        """Open a CSV file to log metrics"""
        self.metrics_file = open('route_planning_metrics.csv', 'w', newline='')
        self.metrics_writer = csv.writer(self.metrics_file)
        self.metrics_writer.writerow(['timestamp',
                                       'execution_time',
                                       'path_length',
                                       'explored_nodes',
                                       'iterations',
                                       'path_cost'])

    def log_metrics(self, **kwargs):
        """Open a CSV file to log metrics."""
        self.metrics_writer.writerow([datetime.now().isoformat()] + list(kwargs.values()))
        self.metrics_file.flush()

    def load_map(self, filename):
        """
            Parses the file to identify nodes, edges, and parking spots,
            then processes each line accordingly.
            """
        self.get_logger().info(f"Loading map data from {filename}")
        try:
            with open(filename, 'r') as file:
                lines = file.readlines()
                section = None
                for line in lines:
                    line = line.strip()
                    if line == "NODES":
                        section = "NODES"
                    elif line == "EDGES":
                        section = "EDGES"
                    elif line == "PARKING_SPOTS":
                        section = "PARKING_SPOTS"
                    else:

                        if section == "NODES":
                            self.process_node_line(line)
                        elif section == "EDGES":
                            self.process_edge_line(line)
                        elif section == "PARKING_SPOTS":
                            self.process_parking_spot_line(line)
        except Exception as e:
            self.get_logger().error(f"Failed to load map data: {str(e)}")

    def process_node_line(self, line):
        """Process a line from the NODES section and add to nodes list
            with coordinates and ID of each node."""
        parts = line.split(',')
        if len(parts) == 3:
            x, y, node_id = map(float, parts)
            self.nodes.append((x, y, int(node_id)))
            self.get_logger().info(f"Added node: {node_id} at ({x}, {y})")

    def process_edge_line(self, line):
        """Process a line from the EDGES section and add to node edges dictionary
        with it cardinal direction."""
        parts = line.split(',')
        if len(parts) == 3:
            node_id, neighbor_id, direction = int(parts[0]), int(parts[1]), parts[2]
            if node_id not in self.edges:
                self.edges[node_id] = []
            self.edges[node_id].append((neighbor_id, direction))
            self.get_logger().info(f"Added edge from {node_id} to {neighbor_id} with direction {direction}")

    def process_parking_spot_line(self, line):
        """Process a line from the PARKING_SPOTS section and add to parking
        spots list."""
        parts = line.split(',')
        if len(parts) == 2:
            x, y = map(float, parts)
            self.parking_spots.append({"x_coordinate": x, "y_coordinate": y})
            self.get_logger().info(f"Added parking spot at ({x}, {y})")

    def assign_parking_nodes(self):
        """
        Assigns the closest node to each parking spot.
        Calculates the distance between each parking spot and node to find
        the closest node.
        """
        for spot in self.parking_spots:
            spot_position = (spot["x_coordinate"], spot["y_coordinate"])
            closest_node = min(self.nodes, key=lambda node: self.distance(node, spot_position))
            spot["assigned_node"] = closest_node

    def timer_callback(self):
        """
        Timer callback function.
        Triggers route calculation if all required data is available and
        a route has not yet been sent.
        """
        if not self.send_route and self.vehicle_location and self.target_coordinate and self.vehicle_orientation:
            self.get_logger().info("Starting route calculation")
            start_node = min(self.nodes, key=lambda node: self.distance(node, self.vehicle_location))
            goal_node = min(self.nodes, key=lambda node: self.distance(node, self.target_coordinate))
            initial_direction = self.yaw_to_direction(self.vehicle_orientation)
            path = self.a_star(start_node[2], goal_node[2], initial_direction)
            if path:
                self.get_logger().info(f"Route found, publishing")
                self.publish_route(path)
            else:
                self.get_logger().warn("No path found")

    def publish_route(self, path):
        """
        Publishes the calculated route.
        Converts the path to a PoseArray and publishes it on the '/route' topic.
        """
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        for node_index in path:
            node = self.nodes[node_index]
            pose_array.poses.append(self.create_pose(node[0], node[1]))
        self.route_publisher_.publish(pose_array)
        self.send_route = True

        # Publish route_state as True after publishing the route
        self.publish_route_state(True)

    def publish_route_state(self, state):
        """
        Publishes the route state.
        Sends a Bool message to the behavioural planning indicating that
        route has been generated
        """
        route_state_msg = Bool()
        route_state_msg.data = state
        self.route_state_publisher_.publish(route_state_msg)
        self.get_logger().info(f"Published route state: {state}")

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        return pose.pose

    def spot_location_callback(self, msg: PoseStamped):
        """
        Callback function for the '/selected_spot' topic.
        Updates the target coordinate with the received spot location.
        """
        self.target_coordinate = (msg.pose.position.x - 1.0, msg.pose.position.y - 1.5)

    def location_pose_callback(self, msg: PoseStamped):
        """
        Callback function for the '/loc_pose' topic.
        Updates the vehicle location with the received pose.
        """
        self.vehicle_location = (msg.pose.position.x, msg.pose.position.y)

    def euler_angles_callback(self, msg: Vector3):
        """
        Callback function for the '/euler_angles' topic.
        Updates the EV orientation with the received Yaw angle.
        """
        self.vehicle_orientation = msg.z

    def yaw_to_direction(self, yaw):
        """
        Converts a yaw angle to a cardinal direction.
        Returns 'N', 'S', 'E', or 'W' based on the yaw angle.
        All been approximated in the model city
        """
        if -math.pi / 4 <= yaw < math.pi / 4:
            return 'E'
        elif math.pi / 4 <= yaw < 3 * math.pi / 4:
            return 'N'
        elif -3 * math.pi / 4 <= yaw < -math.pi / 4:
            return 'S'
        else:
            return 'W'

    def a_star(self, start_id, goal_id, initial_direction):
        """
        Implements the A* algorithm to find the shortest path from start_id
        to goal_id.
        Considers the initial direction and applies directional constraints
        (turning penalties).

        The A* algorithm works as follows:

        1. Initialize the open set with the start node.
        2. Use dictionaries to track the best known path (came_from),
          the cost from the start to a node (g_score),
        and the estimated total cost from the start to the goal (f_score).
        3. While there are nodes to process in the open set:
            a. Select the node with the lowest f_score.
            b. If this node is the goal (closest node to the the parking spot),
              reconstruct and return the path.
            c. For each neighbor of this node:
                i. Calculate the cost to reach the neighbor through the current
                node.
                ii. If this path is better, update the neighbor's g_score and
                  f_score.
                iii. Add the neighbor to the open set.
        4. If the goal is not reached, return None.
         Parameters:
            - start_id (int): The ID of the start node.
            - goal_id (int): The ID of the goal node.
            - initial_direction (str): The initial cardinal direction
            ('N', 'S', 'E', 'W') the vehicle is facing.

            Returns:
            - path (list): A list of node indices representing the optimal
            path from start to goal.
            - None: If no path is found.
        """
        # Records the start time for performance metrics.
        start_time = time.time()
        # Ensure we have valid nodes and edges loaded
        self.get_logger().info(f"Starting A* from node {start_id} to node {goal_id} with initial direction {initial_direction}")
        if not self.nodes or not self.edges:
            self.get_logger().warn("Node list or edges are empty.")
            return None

        # Try to find index positions based on nodes, assuming nodes store (x, y, id) tuples
        try:
            #Cost from start to a node
            start_index = next(index for index, node in enumerate(self.nodes) if node[2] == start_id)
            goal_index = next(index for index, node in enumerate(self.nodes) if node[2] == goal_id)
        except StopIteration:
            self.get_logger().warn("Start or goal node not found in nodes list.")
            return None

        open_set = [(0, start_index)] # Initialize the open set with the start node
        came_from = {}# Dictionary to track the best known path

        #Cost from start to a node
        g_score = {index: float('inf') for index, _ in enumerate(self.nodes)}
        g_score[start_index] = 0

        # Estimated total cost from start to goal
        f_score = {index: float('inf') for index, _ in enumerate(self.nodes)}
        f_score[start_index] = self.heuristic(self.nodes[start_index], self.nodes[goal_index])

        iterations = 0 # Initialize iteration counter
        explored_nodes = set() # Set to track explored nodes


        while open_set:

            current = heappop(open_set)[1] # Selects the node with the lowest f_score.
            explored_nodes.add(current)
            iterations += 1
            #Checks if the current node is the goal. If so, logs metrics, reconstructs the path, and returns it.
            if current == goal_index:
                path_length = self.calculate_path_length(came_from, current)
                execution_time = time.time() - start_time
                path_cost = g_score[current]
                self.log_metrics(
                    execution_time=execution_time,
                    path_length=path_length,
                    explored_nodes=len(explored_nodes),
                    iterations=iterations,
                    path_cost=path_cost
                )

                path = self.reconstruct_path(came_from, current)
                self.get_logger().info(f"Path found: {[x + 1 for x in path]}")

                return path

            current_node = self.nodes[current]
            current_direction = self.get_edge_direction(came_from.get(current), current) if current in came_from else initial_direction
            self.get_logger().info(f"Evaluating node {current} with direction {current_direction}")

            for (neighbor_id, direction) in self.edges.get(current_node[2], []):
                neighbor_index = next((idx for idx, node in enumerate(self.nodes) if node[2] == neighbor_id), None)

                #For each neighbor of the current node, calculates the tentative g_score including turning penalties.
                if neighbor_index is None:
                    continue
                neighbor_direction = self.get_edge_direction(current, neighbor_index)
                direction_penalty = self.direction_change_penalty(current_direction, neighbor_direction)
                tentative_g_score = g_score[current] + self.distance(current_node, self.nodes[neighbor_index]) + direction_penalty
                self.get_logger().info(f"From node {current} to node {neighbor_index} | Current g_score: {g_score[current]}, Tentative g_score: {tentative_g_score}, Penalty: {direction_penalty}")

                #If the tentative g_score is better, updates came_from, g_score, and f_score, and adds the neighbor to the open set.
                if tentative_g_score < g_score[neighbor_index]:
                    came_from[neighbor_index] = current
                    g_score[neighbor_index] = tentative_g_score
                    f_score[neighbor_index] = tentative_g_score + self.heuristic(self.nodes[neighbor_index], self.nodes[goal_index])
                    heappush(open_set, (f_score[neighbor_index], neighbor_index))
                    self.get_logger().info(f"Updated node {neighbor_index} | New g_score: {g_score[neighbor_index]}, f_score: {f_score[neighbor_index]}")

        #If no path is found after exploring all nodes, logs a warning and returns None.
        self.get_logger().warn("No path found after processing all nodes.")
        return None

    def distance(self, node1, node2):
        """
        Calculates and returns the Euclidean distance between two nodes. This is used as a utility function
        within the A* algorithm.
        """
        # Calculate Euclidean distance between two nodes
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

    def heuristic(self, node, goal):
        return self.distance(node, goal)  # Use Euclidean distance as simple heuristic

    def calculate_path_length(self, came_from, current):
        path_length = 0  # Initialize the path length to 0
        while current in came_from:
            next_node = came_from[current] # Get the next node in the path
            # Add the distance between the current node and the next node to the path length
            path_length += self.distance(self.nodes[current], self.nodes[next_node])
            current = next_node # Move to the next node
        return path_length # Return the total path length

    def reconstruct_path(self, came_from, current):
        path = [current] # Initialize path with the goal node
        while current in came_from:
            current = came_from[current] # Backtrack from the current node to the start node
            path.insert(0, current) # Insert the node at the beginning of the path
        return path # Return the reconstructed path

    def get_edge_direction(self, from_index, to_index):
        """
            Determines the compass direction between two connected nodes based on their coordinates.
            Used to apply directional constraints in A* pathfinding.
            """
        if from_index is None or to_index is None:
            return None
        from_node = self.nodes[from_index] # Get the starting node details
        to_node = self.nodes[to_index] # Get the destination node details
        dx = to_node[0] - from_node[0]  # Calculate the difference in x coordinates
        dy = to_node[1] - from_node[1] # Calculate the difference in y coordinates
        if abs(dx) > abs(dy):
            return 'E' if dx > 0 else 'W' # Return 'E' or 'W' based on x difference
        else:
            return 'N' if dy > 0 else 'S' # Return 'N' or 'S' based on y difference

    def direction_change_penalty(self, from_direction, to_direction):
        """
        Applies a penalty for changing direction based on the type of turn:
        - 0 for continuing in the same direction.
        - 5 for right-angle turns (90 degrees).
        - 10 for U-turns (180 degrees).
        """
        direction_changes = {
            'N': {'W': 5, 'E': 5, 'S': 10},
            'S': {'W': 5, 'E': 5, 'N': 10},
            'E': {'N': 5, 'S': 5, 'W': 10},
            'W': {'N': 5, 'S': 5, 'E': 10}
        }
        return direction_changes[from_direction].get(to_direction, 0)

def main(args=None):
    rclpy.init(args=args)
    route_computer = RouteComputerCardinal()
    rclpy.spin(route_computer)
    route_computer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
