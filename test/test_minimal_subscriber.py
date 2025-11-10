import math
import pytest
import types
import numpy as np
from unittest.mock import MagicMock, patch
from geometry_msgs.msg import Twist, PoseStamped
from your_package_name.minimal_subscriber import MinimalSubscriber  # adjust import


@pytest.fixture
def node():
    # Create node without spinning
    with patch('rclpy.node.Node.create_timer'):
        node = MinimalSubscriber()
    return node


def test_goal_callback_sets_goal(node):
    msg = PoseStamped()
    msg.pose.position.x = 2.0
    msg.pose.position.y = 3.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    node.goal_callback(msg)

    assert math.isclose(node.goal_x, 2.0)
    assert math.isclose(node.goal_y, 3.0)
    assert math.isclose(node.goal_yaw, 0.0, abs_tol=1e-6)


def test_quaternion_to_euler_yaw():
    roll, pitch, yaw = MinimalSubscriber.quaternion_to_euler(0, 0, math.sin(math.pi/4), math.cos(math.pi/4))
    assert math.isclose(yaw, math.pi/2, abs_tol=1e-6)


@pytest.mark.parametrize("angle,expected", [
    (math.pi + 0.1, -math.pi + 0.1),
    (-math.pi - 0.1, math.pi - 0.1),
    (0.0, 0.0)
])
def test_angle_wrap(angle, expected):
    wrapped = MinimalSubscriber.angle_wrap(angle)
    assert math.isclose(wrapped, expected, abs_tol=1e-6)


def test_control_loop_navigation_phase(node):
    # Mock transform lookup
    transform_mock = MagicMock()
    transform_mock.transform.translation.x = 0.0
    transform_mock.transform.translation.y = 0.0
    transform_mock.transform.rotation.x = 0.0
    transform_mock.transform.rotation.y = 0.0
    transform_mock.transform.rotation.z = 0.0
    transform_mock.transform.rotation.w = 1.0

    node.tf_buffer.lookup_transform = MagicMock(return_value=transform_mock)
    node.get_clock = MagicMock()
    node.prev_time = types.SimpleNamespace(nanoseconds=0)
    node.get_clock.now = MagicMock(return_value=types.SimpleNamespace(nanoseconds=1e9))  # dt = 1s

    node.goal_x = 1.0
    node.goal_y = 0.0
    node.goal_yaw = 0.0

    published_msgs = []
    node.cmd_pub.publish = lambda msg: published_msgs.append(msg)

    node.control_loop()

    assert len(published_msgs) == 1
    msg = published_msgs[0]
    # The robot should move forward toward the goal
    assert msg.linear.x > 0
    assert abs(msg.angular.z) < 1.0


def test_control_loop_stop_condition(node):
    transform_mock = MagicMock()
    transform_mock.transform.translation.x = 1.0
    transform_mock.transform.translation.y = 1.0
    transform_mock.transform.rotation.x = 0.0
    transform_mock.transform.rotation.y = 0.0
    transform_mock.transform.rotation.z = 0.0
    transform_mock.transform.rotation.w = 1.0

    node.tf_buffer.lookup_transform = MagicMock(return_value=transform_mock)
    node.get_clock = MagicMock()
    node.prev_time = types.SimpleNamespace(nanoseconds=0)
    node.get_clock.now = MagicMock(return_value=types.SimpleNamespace(nanoseconds=1e9))

    # Set goal at same position (stop condition)
    node.goal_x = 1.0
    node.goal_y = 1.0
    node.goal_yaw = 0.0

    published_msgs = []
    node.cmd_pub.publish = lambda msg: published_msgs.append(msg)

    node.control_loop()

    assert len(published_msgs) == 1
    msg = published_msgs[0]
    assert math.isclose(msg.linear.x, 0.0, abs_tol=1e-6)
    assert math.isclose(msg.angular.z, 0.0, abs_tol=1e-6)
    assert node.goal_x is None  # Goal cleared after reaching
