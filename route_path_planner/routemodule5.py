import math
from heapq import heappop, heappush
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid

class RouteComputer(Node):
    """
    RouteComputer class calculates and publishes a route using the A* algorithm
    based on vehicle and target coordinates received through subscriptions.
    """

    def __init__(self):
        super().__init__('route_computer')

        # Publisher to send the route as a list of waypoints
        self.publisher_ = self.create_publisher(PoseArray, '/route', 10)
        self.route_state_publisher_ = self.create_publisher(Bool, '/route_state', 10) # Specific topic for behvioural planning states
        self.timer = self.create_timer(0.1, self.timer_callback)
   
        self.send_route = False
       
        
        # Subscriber to receive the selected parking spot
        self.subscription_spot = self.create_subscription(PoseStamped, '/selected_spot', self.spot_location_callback, 10)
        # Subscriber to receive the vehicle's current location
        self.subscription_loc = self.create_subscription(PoseStamped, '/loc_pose', self.location_pose_callback, 10)

        # Initialize variables to store locations
        self.latest_localization = None
        self.vehicle_location = None
        self.target_coordinate = None
        self.num_waypoints = 10  # Number of waypoints

        # Load map data from file
        self.nodes = []
        self.edges = {}
        self.parking_spots = []
        package_share_directory = get_package_share_directory('adapt_roucomp')
        map_file = os.path.join(package_share_directory, 'config', 'map.txt')
        self.load_map(map_file)

        # Find the closest map nodes to each parking spot
        self.assign_parking_nodes()

        # Initialize publisher count
        self.publisher_count = 0

    def load_map(self, filename):
        """
        Read and parse the map file to initialize nodes, edges, and parking spots.
        """
        with open(filename, 'r') as file:
            lines = file.readlines()
            section = None

            for line_num, line in enumerate(lines, start=1):
                line = line.strip()

                # Skip empty lines
                if not line:
                    continue

                # Check which section we're in
                if line == "NODES":
                    section = "NODES"
                    continue
                elif line == "EDGES":
                    section = "EDGES"
                    continue
                elif line == "PARKING_SPOTS":
                    section = "PARKING_SPOTS"
                    continue

                parts = line.split()
                # Read nodes
                if section == "NODES":
                    if len(parts) != 3:
                        self.get_logger().warn(f"Ignoring line {line_num} in {filename}: {line}")
                        continue
                    x, y, node_id = map(float, parts)
                    self.nodes.append((x, y, int(node_id)))
                # Read edges (connections between nodes)
                elif section == "EDGES":
                    if len(parts) < 2:
                        self.get_logger().warn(f"Ignoring line {line_num} in {filename}: {line}")
                        continue
                    node_id = int(parts[0])
                    neighbors = list(map(int, parts[1:]))
                    self.edges[node_id] = neighbors
                # Read parking spots
                elif section == "PARKING_SPOTS":
                    if len(parts) != 2:
                        self.get_logger().warn(f"Ignoring line {line_num} in {filename}: {line}")
                        continue
                    x, y = map(float, parts)
                    self.parking_spots.append({"x_coordinate": x, "y_coordinate": y, "width": 0.43, "length": 0.9, "status": 0})

    def assign_parking_nodes(self):
        """
        Assign each parking spot to the nearest map node.
        """
        for spot in self.parking_spots:
            spot_position = (spot["x_coordinate"], spot["y_coordinate"])
            closest_node = min(self.nodes, key=lambda node: self.distance(node, spot_position))
            spot["assigned_node"] = closest_node

    def timer_callback(self):
        """
        Timer callback to check if route calculation is necessary and initiate A* pathfinding.
        """
        if not self.send_route and self.vehicle_location and self.target_coordinate:
            pose_array = PoseArray()
            pose_array.header.frame_id = "map"
            start_node = min(self.nodes, key=lambda node: self.distance(node, self.vehicle_location))
            goal_node = min(self.nodes, key=lambda node: self.distance(node, self.target_coordinate))
            path = self.a_star(start_node, goal_node)
            if path:
                for node in path:
                    pose_array.poses.append(self.create_pose(node[0], node[1]))
                self.publisher_.publish(pose_array)
                for idx, pose in enumerate(pose_array.poses):
                    self.get_logger().info('Waypoints for the route:- no: %d, x: %f, y: %f' % (idx, pose.position.x, pose.position.y))
                self.send_route = True
                self.publish_route_state(True)
            else:
                self.get_logger().warn("No path found between the specified nodes.")
                self.publish_route_state(False)
                
    def publish_route_state(self, state):
        """
        Publishes the route state.
        Sends a Bool message to the behavioural planning indicating that route has been generated
        """
        route_state_msg = Bool()
        route_state_msg.data = state
        self.route_state_publisher_.publish(route_state_msg)
        self.get_logger().info(f"Published route state: {state}")
        
    def create_pose(self, x, y):
        """
        Create a PoseStamped message with given x, y coordinates.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        return pose.pose

    def spot_location_callback(self, msg: PoseStamped):
        """
        Callback to store the target parking spot location received via subscription.
        """
        self.target_coordinate = (msg.pose.position.x - 1.0, msg.pose.position.y - 1.5)
        #self.get_logger().info('Received spot location data: (%f, %f, %f)' % (
            #msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        #))

    def location_pose_callback(self, msg: PoseStamped):
        """
        Callback to store the vehicle's current location received via subscription.
        """
        self.vehicle_location = (msg.pose.position.x, msg.pose.position.y)
        #self.get_logger().info('Received vehicle location data: (%f, %f, %f)' % (
            #msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        #))

    def a_star(self, start, goal):
        """
        A* algorithm to find the shortest path from start to goal node.
        """
        start_index = start[2]
        goal_index = goal[2]

        open_set = []
        heappush(open_set, (0, start_index))
        came_from = {}
        g_score = {node[2]: float('inf') for node in self.nodes}
        g_score[start_index] = 0
        f_score = {node[2]: float('inf') for node in self.nodes}
        f_score[start_index] = self.heuristic(start, goal)

        while open_set:
            current = heappop(open_set)[1]
            if current == goal_index:
                return self.reconstruct_path(came_from, current)
            for neighbor_index in self.edges.get(current, []):
                neighbor = self.nodes[neighbor_index - 1]
                tentative_g_score = g_score[current] + self.distance(self.nodes[current - 1], neighbor)
                if tentative_g_score < g_score[neighbor[2]]:
                    came_from[neighbor[2]] = current
                    g_score[neighbor[2]] = tentative_g_score
                    f_score[neighbor[2]] = g_score[neighbor[2]] + self.heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor[2]], neighbor[2]))

        return None

    def heuristic(self, node, goal):
        """
        Heuristic function to estimate the straight-line distance from node to goal.
        """
        return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    def distance(self, node1, node2):
        """
        Straight-line distance between two nodes.
        """
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    def reconstruct_path(self, came_from, current):
        """
        Reconstruct the path from start to goal using the came_from map.
        """
        total_path = [self.nodes[current - 1]]
        while current in came_from:
            current = came_from[current]
            total_path.insert(0, self.nodes[current - 1])
        return total_path


def main(args=None):
    """
    Main function to initialize ROS2 system and start the RouteComputer node.
    """
    rclpy.init(args=args)
    route_computer = RouteComputer()
    rclpy.spin(route_computer)
    route_computer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

