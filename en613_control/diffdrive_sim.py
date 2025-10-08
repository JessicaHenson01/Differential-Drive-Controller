import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from example_interfaces.srv import Trigger
from builtin_interfaces.msg import Time
import numpy as np
import math
from tf2_ros import TransformBroadcaster

class Diffdrive:
    def __init__(self, width, radius):
        """
        Constructor for the Differential Drive kinematics class. Sets the vehicle properties.

        Input
        :param width: The width of the vehicle
        :param radius: The radius of the wheels
        """
        self.w = width
        self.r = radius

    def forward(self, x, u):
        """
        Computes the forward kinematics for the Differential Drive system.

        Input
        :param x: The starting state (position) of the system. This is [x,y,theta].
        :param u: The control input to the system. This is the wheel rates for each wheel [ur, ul]

        Output
        :return: v: The resulting velocity vector for the system. This is [v, w]

        """
        ur, ul = u
        v_l = self.r*ul
        v_r = self.r*ur

        v_x = 0.5*(v_l + v_r)
        w = (v_r - v_l)/self.w

        return [v_x, w]
    
    def inverse(self, x, v):
        """
        Computes the inverse kinematics for the Differential Drive system.

        Input
        :param x: The starting state (position) of the system.This is [x,y,theta].
        :param v: The desired velocity vector for the system. This is [Vx, w]

        Output
        :return: u: The necessary control inputs to achieve the desired velocity vector.This is the wheel rates for each wheel  [ur, ul]
        """

        v_x, w = v

        v_r = v_x + (w*self.w/2)
        v_l = v_x - (w*self.w/2)

        ur = v_r/self.r
        ul = v_l/self.r

        return [ur, ul]

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # PARAMETERS
        self.declare_parameter('wheel_base', 0.825)   # distance between wheels (m)
        self.declare_parameter('wheel_radius', 0.4)   # wheel radius (m)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('left_wheel_joint', 'chassis_to_left_wheel')
        self.declare_parameter('right_wheel_joint', 'chassis_to_right_wheel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('chassis_frame', 'chassis')

        w = self.get_parameter('wheel_base').value
        r = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.chassis_frame = self.get_parameter('chassis_frame').value
        self.left_joint_name = self.get_parameter('left_wheel_joint').value
        self.right_joint_name = self.get_parameter('right_wheel_joint').value

        self.m = Diffdrive(width=w,radius=r)

        # robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.curr_pos = [self.x, self.y, self.theta]

        # wheel angles (radians) and wheel angular velocities (rad/s)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.left_wheel_rate = 0.0
        self.right_wheel_rate = 0.0

        # commanded velocities (from /cmd_vel)
        self.cmd_vx = 0.0
        self.cmd_omega = 0.
        

        self.subscription=self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, qos_profile_sensor_data)
        self.subscription
        # Publisher for transformed object positions
        # Publishers for each wheel 
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.l_pub = self.create_publisher(Float64, '/l_wheel_vel_cmd', 10)
        self.r_pub = self.create_publisher(Float64, '/r_wheel_vel_cmd', 10)

         # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

         # service to reset pose
        self.create_service(Trigger, '/robot_pose_reset', self.reset_pose_callback)

        # timer: update loop
        rate = float(self.get_parameter('publish_rate').value)
        timer_period = 1.0 / rate
        self._last_time = self.get_clock().now()
        self.create_timer(timer_period, self.update_loop)

        self.get_logger().info('diffdrive_sim node started (publishing at %.1f Hz)' % rate)



    def cmd_vel_cb(self, msg: Twist):
        # Only linear.x and angular.z are used
        self.cmd_vx = float(msg.linear.x)
        self.cmd_omega = float(msg.angular.z)

    def reset_pose_callback(self, request, response):
        # reset pose and joints back to zeros
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.curr_pos = [self.x, self.y, self.theta]
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.left_wheel_rate = 0.0
        self.right_wheel_rate = 0.0
        response.success = True
        response.message = ''
        self.get_logger().info('Robot pose and joints reset to zero.')
        return response

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0:
            dt = 1e-6
        self._last_time = now

        # compute wheel angular rates required to produce commanded (vx, omega)
        ur, ul = self.m.inverse(self.curr_pos, [self.cmd_vx, self.cmd_omega])
        # set wheel rates
        self.right_wheel_rate = float(ur)
        self.left_wheel_rate = float(ul)

        # publish wheel velocity commands (optional topics)
        self.r_pub.publish(Float64(data=self.right_wheel_rate))
        self.l_pub.publish(Float64(data=self.left_wheel_rate))

        # compute chassis velocities from wheel rates (forward kin)
        v_x, omega = self.m.forward(self.curr_pos, [self.right_wheel_rate, self.left_wheel_rate])

        # integrate pose (assume v_x is forward speed in robot frame)
        # convert to world frame:
        dx = v_x * math.cos(self.theta) * dt
        dy = v_x * math.sin(self.theta) * dt
        dtheta = omega * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta
        # normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # update wheel angles
        self.right_wheel_angle += self.right_wheel_rate * dt
        self.left_wheel_angle += self.left_wheel_rate * dt

        # publish TF transform from 'odom' -> 'chassis'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.chassis_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        # quaternion from yaw
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # publish joint states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.left_joint_name, self.right_joint_name]
        # joint positions are wheel rotations in radians
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        # joint velocity matches wheel angular velocity
        js.velocity = [self.left_wheel_rate, self.right_wheel_rate]
        # effort left empty
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
