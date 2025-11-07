"""
Integration test for RouteComputer using ROS 2.
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

try:
    from adapt_route_computer.route_computer import RouteComputer
except ImportError:
    print("Error: Unable to import 'adapt_route_computer.route_computer'. Check if the module is installed and accessible.")

class IntegrationHelperNode(Node):
    """A helper node for integration testing."""
    
    def __init__(self):
        super().__init__('integration_helper_node')
        self.received_route = None
        self.route_subscription = self.create_subscription(
            Path,
            '/route',
            self.route_callback,
            10
        )

    def route_callback(self, msg):
        """Callback function for the route topic."""
        self.received_route = msg

@pytest.fixture(scope="module", autouse=True)
def rclpy_setup_teardown():
    """Fixture to initialize and shutdown rclpy."""
    rclpy.init()
    yield
    rclpy.shutdown()

def test_route_computer_integration():
    """Integration test for RouteComputer node."""
    route_computer_node = RouteComputer()
    helper_node = IntegrationHelperNode()
    executor = SingleThreadedExecutor()
    executor.add_node(route_computer_node)
    executor.add_node(helper_node)

    try:
        # Simulate selected spot message reception
        selected_spot_msg = String()
        selected_spot_msg.data = "spot_A"
        route_computer_node.selected_spot_subscriber.callback(selected_spot_msg)

        # Simulate localization pose message reception
        loc_pose_msg = PoseStamped()
        loc_pose_msg.pose.position.x = 1.0
        loc_pose_msg.pose.position.y = 2.0
        loc_pose_msg.pose.position.z = 0.0
        loc_pose_msg.pose.orientation.x = 0.0
        loc_pose_msg.pose.orientation.y = 0.0
        loc_pose_msg.pose.orientation.z = 0.0
        loc_pose_msg.pose.orientation.w = 1.0
        route_computer_node.loc_pose_subscriber.callback(loc_pose_msg)

        # Run the executor to process the messages
        executor.spin_once(timeout_sec=1)

        # Verify that the route was published
        assert helper_node.received_route is not None
        assert len(helper_node.received_route.poses) > 0

    finally:
        executor.shutdown()
        route_computer_node.destroy_node()
        helper_node.destroy_node()


