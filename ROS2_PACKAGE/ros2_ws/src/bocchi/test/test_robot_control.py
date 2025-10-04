#!/usr/bin/env python3

import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the bocchi package to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from bocchi.publisher import KeyStateManager


class TestRobotControlKeys(unittest.TestCase):
    """Test robot control key mappings and functionality"""

    def setUp(self):
        """Set up test fixtures"""
        self.key_manager = KeyStateManager(debounce_time=0.01)

    def test_wasd_forward_movement(self):
        """Test W key for forward movement"""
        # Press W key (87)
        result = self.key_manager.update_key(87, is_pressed=True)
        self.assertTrue(result)

        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, 0.5)
        self.assertEqual(twist.angular.z, 0.0)
        self.assertEqual(twist.linear.y, 0.0)
        self.assertEqual(twist.linear.z, 0.0)
        self.assertEqual(twist.angular.x, 0.0)
        self.assertEqual(twist.angular.y, 0.0)

    def test_wasd_backward_movement(self):
        """Test S key for backward movement"""
        # Press S key (83)
