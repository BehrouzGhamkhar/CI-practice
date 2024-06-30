import unittest
from unittest.mock import MagicMock, patch
from robot_sm import MonitorBatteryAndCollision, ApplyMotion, RotateBase, StopMotion, ManuallyMoveToSafeDistance
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TestMonitorBatteryAndCollision(unittest.TestCase):
    def setUp(self):
        self.node = MagicMock()
        self.state = MonitorBatteryAndCollision(self.node)

    def test_check_collision(self):
        data = LaserScan()
        data.ranges = [0.5, 0.3, 0.6]
        self.assertTrue(self.state.check_collision(data))

        data.ranges = [0.5, 0.5, 0.6]
        self.assertFalse(self.state.check_collision(data))

        data.ranges = []
        self.assertFalse(self.state.check_collision(data))

        data.ranges = [-1.0, -2.0]
        self.assertFalse(self.state.check_collision(data))

    @patch('robot_sm.rclpy.spin_once')
    def test_execute_collision(self, mock_spin_once):
        self.state.collision = True
        userdata = MagicMock()
        outcome = self.state.execute(userdata)
        self.assertEqual(outcome, "collision")
        self.assertTrue(userdata.collision_output)

# Define more unit tests for RotateBase, StopMotion, and ManuallyMoveToSafeDistance following the above patterns

if __name__ == '__main__':
    unittest.main()
