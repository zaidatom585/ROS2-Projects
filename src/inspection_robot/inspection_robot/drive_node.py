import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class DifferentialDrive(Node):
    def __init__(self):
        super().__init__('differential_drive')

        # Robot dimensions (tune to match your URDF)
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('wheel_separation', 0.45)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel angles
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0

        # Current velocity
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Time
        self.last_time = self.get_clock().now()

        # Receive velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish wheel positions
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Publish odom -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Update robot at 50 Hz
        self.timer = self.create_timer(0.02, self.update_robot)

        self.get_logger().info('Differential drive controller started')

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_robot(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # Differential drive kinematics
        left_velocity = self.linear_velocity - self.angular_velocity * self.wheel_separation / 2.0
        right_velocity = self.linear_velocity + self.angular_velocity * self.wheel_separation / 2.0

        left_wheel_velocity = left_velocity / self.wheel_radius
        right_wheel_velocity = right_velocity / self.wheel_radius

        # Update wheel positions
        self.left_wheel_position += left_wheel_velocity * dt
        self.right_wheel_position += right_wheel_velocity * dt

        # Update robot pose
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # --- Publish Joint States ---
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_position, self.right_wheel_position]
        joint_state.velocity = [left_wheel_velocity, right_wheel_velocity]
        self.joint_pub.publish(joint_state)

        # --- Publish odom -> base_link TF ---
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # CRITICAL FIX: Explicitly set ALL quaternion components
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)

        # --- Publish Odometry message ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DifferentialDrive()
        rclpy.spin(node)
    except Exception as e:
        print(f"\n*** REAL ERROR: {e} ***\n")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()