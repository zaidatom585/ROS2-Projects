import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.get_logger().info(
            'Obstacle avoidance started'
        )

    def get_sector_min(self, msg, min_angle, max_angle):

        values = []

        angle = msg.angle_min

        for distance in msg.ranges:

            if min_angle <= angle <= max_angle:

                if math.isfinite(distance):

                    if msg.range_min < distance < msg.range_max:
                        values.append(distance)

            angle += msg.angle_increment

        if values:
            return min(values)

        return float('inf')

    def scan_callback(self, msg):

        # FRONT: -30° to +30°
        self.front_distance = self.get_sector_min(
            msg,
            math.radians(-30),
            math.radians(30)
        )

        # LEFT: +30° to +90°
        self.left_distance = self.get_sector_min(
            msg,
            math.radians(30),
            math.radians(90)
        )

        # RIGHT: -90° to -30°
        self.right_distance = self.get_sector_min(
            msg,
            math.radians(-90),
            math.radians(-30)
        )

    def control_loop(self):

        cmd = Twist()

        # Print what LiDAR sees
        self.get_logger().info(
            f'Front: {self.front_distance:.2f} m | '
            f'Left: {self.left_distance:.2f} m | '
            f'Right: {self.right_distance:.2f} m'
        )

        # -------------------------
        # VERY CLOSE
        # -------------------------

        if self.front_distance < 0.4:

            cmd.linear.x = 0.0

            if self.left_distance > self.right_distance:

                cmd.angular.z = 1.0

                self.get_logger().warn(
                    'VERY CLOSE - TURNING LEFT'
                )

            else:

                cmd.angular.z = -1.0

                self.get_logger().warn(
                    'VERY CLOSE - TURNING RIGHT'
                )

        # -------------------------
        # OBSTACLE AHEAD
        # -------------------------

        elif self.front_distance < 1.2:

            cmd.linear.x = 0.05

            if self.left_distance > self.right_distance:

                cmd.angular.z = 0.8

                self.get_logger().warn(
                    'OBSTACLE AHEAD - TURNING LEFT'
                )

            else:

                cmd.angular.z = -0.8

                self.get_logger().warn(
                    'OBSTACLE AHEAD - TURNING RIGHT'
                )

        # -------------------------
        # CLEAR
        # -------------------------

        else:

            cmd.linear.x = 0.25
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()