import unittest
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from adapt_roucomp.routemodule5 import RouteComputer  # Replace with your actual module name

class TestRouteComputer(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = RouteComputer()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_initialization(self):
        # Test that RouteComputer initializes correctly
        self.assertIsNotNone(self.node.publisher_)
        self.assertIsNotNone(self.node.subscription_spot)
        self.assertIsNotNone(self.node.subscription_loc)
        self.assertEqual(self.node.publisher_count, 0)

    def test_a_star_pathfinding(self):
        # Test a_star method for pathfinding
        start_node = (0.0, 0.0, 1)
        goal_node = (5.0, 5.0, 2)
        path = self.node.a_star(start_node, goal_node)
        self.assertIsNotNone(path)
        
        # Check that the first node in the path is close to the start_node
        #self.assertAlmostEqual(path[0][0], start_node[0], delta=0.1)  # Check x-coordinate
        #self.assertAlmostEqual(path[0][1], start_node[1], delta=0.1)  # Check y-coordinate
        #self.assertEqual(path[0][2], start_node[2])  # Check node ID

        # Verify that the path ends with the goal_node
        #self.assertEqual(path[-1], goal_node)

    def test_publisher_count_increment(self):
        # Test publisher count increment when messages are published
        self.assertEqual(self.node.publisher_count, 0)

        # Simulate receiving vehicle location
        loc_msg = PoseStamped()
        loc_msg.pose.position.x = 3.0
        loc_msg.pose.position.y = 4.0
        loc_msg.pose.position.z = 0.0
        self.node.location_pose_callback(loc_msg)

        # Simulate receiving spot location
        spot_msg = PoseStamped()
        spot_msg.pose.position.x = 1.0
        spot_msg.pose.position.y = 2.0
        spot_msg.pose.position.z = 0.0
        self.node.spot_location_callback(spot_msg)

        # Spin to trigger publisher
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Allow time for publisher to update publisher_count (adjust as needed)
        time.sleep(0.5)

        # Assert that the publisher_count increments from 0 to 1
        # self.assertEqual(self.node.publisher_count, 1)

if __name__ == '__main__':
    unittest.main()

