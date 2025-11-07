import unittest
from unittest.mock import patch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Vector3
from std_msgs.msg import Bool
from rclpy.executors import SingleThreadedExecutor
from adapt_roucomp.route_cardinal import RouteComputerCardinal  # Replace with your actual module name

class TestRouteComputerCardinalIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = SingleThreadedExecutor()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch.object(RouteComputerCardinal, 'load_map', return_value=None):
            self.node = RouteComputerCardinal()
            # Manually set nodes and edges for testing
            self.node.nodes = [
                (0.5, 0.7, 1),
                (1.5, 0.7, 2),
                (2.3, 0.4, 3),
                (3.5, 0.7, 4),
                (3.5, 1.5, 5),
                (3.5, 2.7, 6)
            ]
            self.node.edges = {
                1: [(2, 'E')],
                2: [(3, 'E')],
                3: [(4, 'E')],
                4: [(5, 'N')],
                5: [(6, 'N')]
            }
            self.executor.add_node(self.node)

        self.route_received = False
        self.route_state_received = False

        self.node.create_subscription(PoseArray, '/route', self.route_callback, 10)
        self.node.create_subscription(Bool, '/route_state', self.route_state_callback, 10)

    def tearDown(self):
        self.executor.remove_node(self.node)

    def route_callback(self, msg):
        self.route_received = True
        self.node.get_logger().info("Route received")

    def route_state_callback(self, msg):
        self.route_state_received = msg.data
        self.node.get_logger().info(f"Route state received: {msg.data}")

    def test_route_computation_and_publishing(self):
        # Simulate receiving vehicle location
        loc_msg = PoseStamped()
        loc_msg.pose.position.x = 0.5
        loc_msg.pose.position.y = 0.7
        loc_msg.pose.position.z = 0.0
        self.node.location_pose_callback(loc_msg)

        # Simulate receiving spot location
        spot_msg = PoseStamped()
        spot_msg.pose.position.x = 3.5
        spot_msg.pose.position.y = 2.7
        spot_msg.pose.position.z = 0.0
        self.node.spot_location_callback(spot_msg)

        # Simulate receiving vehicle orientation
        euler_msg = Vector3()
        euler_msg.z = 1.0
        self.node.euler_angles_callback(euler_msg)

        # Trigger the timer callback manually
        self.node.timer_callback()

        # Allow time for messages to be processed
        for _ in range(10):
            self.executor.spin_once(timeout_sec=0.1)
            if self.route_received and self.route_state_received:
                break

        # Check if route and state are published correctly
        self.assertTrue(self.route_received)
        self.assertTrue(self.route_state_received)

if __name__ == '__main__':
    unittest.main()
