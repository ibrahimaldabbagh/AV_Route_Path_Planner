import unittest
import math
import rclpy
from route_computer_cardinal import RouteComputerCardinal

class TestRouteComputerCardinal(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.route_computer = RouteComputerCardinal()

    def test_yaw_to_direction(self):
        self.assertEqual(self.route_computer.yaw_to_direction(0), 'E')
        self.assertEqual(self.route_computer.yaw_to_direction(math.pi / 2), 'N')
        self.assertEqual(self.route_computer.yaw_to_direction(-math.pi / 2), 'S')
        self.assertEqual(self.route_computer.yaw_to_direction(math.pi), 'W')
        self.assertEqual(self.route_computer.yaw_to_direction(-math.pi), 'W')

    def test_process_node_line(self):
        line = "1.0,2.0,3"
        self.route_computer.process_node_line(line)
        self.assertIn((1.0, 2.0, 3), self.route_computer.nodes)

    def test_process_edge_line(self):
        line = "1,2,E"
        self.route_computer.process_edge_line(line)
        self.assertIn((2, 'E'), self.route_computer.edges[1])

    def test_a_star_no_nodes(self):
        self.route_computer.nodes = []
        self.route_computer.edges = {}
        result = self.route_computer.a_star(1, 2, 'N')
        self.assertIsNone(result)

    def test_a_star_path_found(self):
        self.route_computer.nodes = [(0, 0, 1), (1, 1, 2)]
        self.route_computer.edges = {1: [(2, 'E')], 2: [(1, 'W')]}
        result = self.route_computer.a_star(1, 2, 'E')
        self.assertIsNotNone(result)
        self.assertEqual(result, [0, 1])

if __name__ == '__main__':
    unittest.main()

