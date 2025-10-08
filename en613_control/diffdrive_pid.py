import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros
import math
import numpy as np

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Control parameters (PD)
        self.kp_linear = 0.5 # reduce to increase turning
        self.kd_linear = 0.1 
        self.kp_angular = 4.0 # Increase for a faster sharp turn
        self.kd_angular = 0.8  # Increase to dampen turn

        # Target and state variables
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.prev_time = self.get_clock().now()

        # Run controller loop at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Extract the orientation from the message
        q = msg.pose.orientation
        # Convert the quaternion to euler angles and get the yaw
        _, _, self.goal_yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

        self.get_logger().info(f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_yaw:.2f} rad)")


    def control_loop(self):
        if self.goal_x is None:
            # If there's no goal, do nothing.
            return

        # --- This is your original code to get the robot's pose ---
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'chassis', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        _, _, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        # --- End of your original pose code ---

        # --- This is your original code to calculate dt ---
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 1e-6:
            return
        # --- End of your original dt code ---
        
        # 1. Calculate all possible errors up front
        distance_error = math.sqrt((self.goal_x - x)**2 + (self.goal_y - y)**2)
        path_angle = math.atan2(self.goal_y - y, self.goal_x - x)
        path_angle_error = self.angle_wrap(path_angle - yaw)
        final_angle_error = self.angle_wrap(self.goal_yaw - yaw)

        # 2. Check for the final "stop" condition first. This is the most reliable structure.
        if distance_error < 0.05 and abs(final_angle_error) < 0.05:
            # We are at the goal and facing the correct direction. Command a full stop.
            linear_speed = 0.0
            angular_speed = 0.0
            self.goal_x = None  # Clear the goal to stop the controller on the next loop.
            
        else:
            # If we are not stopped, choose which error to use for control
            if distance_error > 0.1:
                # PHASE 1: NAVIGATION
                current_linear_error = distance_error
                current_angular_error = path_angle_error
            else:
                # PHASE 2: ALIGNMENT
                current_linear_error = 0.0
                current_angular_error = final_angle_error

            # 3. Apply your full PD control law using the chosen errors
            d_error_linear = (current_linear_error - self.prev_error_linear) / dt
            d_error_angular = (current_angular_error - self.prev_error_angular) / dt
            
            linear_speed = self.kp_linear * current_linear_error + self.kd_linear * d_error_linear
            angular_speed = self.kp_angular * current_angular_error + self.kd_angular * d_error_angular

            # Update previous errors for the next loop's derivative calculation
            self.prev_error_linear = current_linear_error
            self.prev_error_angular = current_angular_error

        # --- This is your original code for clipping and publishing ---
        linear_speed = np.clip(linear_speed, -0.5, 0.5)
        angular_speed = np.clip(angular_speed, -1.0, 1.0)

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.cmd_pub.publish(cmd)

        self.prev_time = now
        # --- End of your original code ---
        
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def angle_wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()