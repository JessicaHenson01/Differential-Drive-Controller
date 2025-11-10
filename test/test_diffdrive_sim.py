import math
import pytest
from unittest.mock import MagicMock, patch
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from example_interfaces.srv import Trigger
from en613_control.diffdrive_sim import Diffdrive, MinimalSubscriber


# ---------- PURE KINEMATICS TESTS ---------- #

def test_forward_kinematics():
    diff = Diffdrive(width=0.5, radius=0.1)
    v = diff.forward([0, 0, 0], [10, 10])  # both wheels same speed
    assert math.isclose(v[0], 1.0, rel_tol=1e-6)  # linear velocity
    assert math.isclose(v[1], 0.0, abs_tol=1e-6)  # no rotation


def test_forward_turning():
    diff = Diffdrive(width=0.5, radius=0.1)
    v = diff.forward([0, 0, 0], [10, 5])  # right faster than left
    assert v[0] > 0
    assert v[1] > 0  # positive angular velocity


def test_inverse_kinematics():
    diff = Diffdrive(width=0.5, radius=0.1)
    ur, ul = diff.inverse([0, 0, 0], [1.0, 0.0])
    assert math.isclose(ur, ul, abs_tol=1e-6)
    assert ur > 0


def test_inverse_turning():
    diff = Diffdrive(width=0.5, radius=0.1)
    ur, ul = diff.inverse([0, 0, 0], [0.5, 1.0])
    assert ur != ul
    assert ur > ul


# ---------- NODE TESTS (mocking rclpy + tf + pub/sub) ---------- #

@pytest.fixture
def node():
    # patch ROS2 components that require runtime
    with patch('rclpy.node.Node.create_timer'), \
         patch('rclpy.node.Node.create_publisher'), \
         patch('rclpy.node.Node.create_subscription'), \
         patch('rclpy.node.Node.create_service'), \
         patch('rclpy.node.Node.get_logger'):
        node = MinimalSubscriber()
    return node


def test_cmd_vel_callback(node):
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = 0.5
    node.cmd_vel_cb(msg)

    assert math.isclose(node.cmd_vx, 1.0)
    assert math.isclose(node.cmd_omega, 0.5)


def test_reset_pose_callback(node):
    req = MagicMock()
    res = MagicMock()
    node.x, node.y, node.theta = 5.0, 2.0, 1.0
    node.left_wheel_angle = 1.0
    node.right_wheel_angle = 1.0
    node.left_wheel_rate = 1.0
    node.right_wheel_rate = 1.0

    resp = node.reset_pose_callback(req, res)

    assert resp.success
    assert node.x == 0.0
    assert node.theta == 0.0
    assert node.left_wheel_rate == 0.0
    assert node.right_wheel_rate == 0.0


def test_update_loop_basic(node):
    # mock all publishers to record outputs
    node.r_pub = MagicMock()
    node.l_pub = MagicMock()
    node.joint_pub = MagicMock()
    node.tf_broadcaster = MagicMock()

    # mock time progression
    node.get_clock = MagicMock()
    node._last_time = MagicMock(nanoseconds=0)
    node.get_clock.now.return_value = MagicMock(nanoseconds=1e9)  # 1 second later

    # give simple motion command
    node.cmd_vx = 1.0
    node.cmd_omega = 0.0
    node.curr_pos = [0.0, 0.0, 0.0]

    node.update_loop()

    # check that publishers were called
    assert node.r_pub.publish.called
    assert node.l_pub.publish.called
    assert node.joint_pub.publish.called
    assert node.tf_broadcaster.sendTransform.called

    # verify that robot moved forward
    assert node.x > 0.0
    assert math.isclose(node.theta, 0.0, abs_tol=1e-6)


def test_update_loop_turning(node):
    node.r_pub = MagicMock()
    node.l_pub = MagicMock()
    node.joint_pub = MagicMock()
    node.tf_broadcaster = MagicMock()
    node.get_clock = MagicMock()
    node._last_time = MagicMock(nanoseconds=0)
    node.get_clock.now.return_value = MagicMock(nanoseconds=1e9)

    node.cmd_vx = 0.0
    node.cmd_omega = 1.0
    node.curr_pos = [0.0, 0.0, 0.0]

    node.update_loop()

    # check rotation updated
    assert abs(node.theta) > 0.0
    assert node.r_pub.publish.called
    assert node.l_pub.publish.called
