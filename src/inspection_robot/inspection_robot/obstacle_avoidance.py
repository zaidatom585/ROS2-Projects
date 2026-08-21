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

        # Remember the last turning direction
        self.last_turn = 1.0

        self.get_logger().info(
            'Improved obstacle avoidance started'
        )

    def get_min_distance(self, ranges, start, end):

        valid_ranges = []

        for distance in ranges[start:end]:

            if math.isfinite(distance):

                if 0.1 < distance < 10.0:
                    valid_ranges.append(distance)

        if valid_ranges:
            return min(valid_ranges)

        return float('inf')

    def scan_callback(self, msg):

        ranges = msg.ranges
        count = len(ranges)

        center = count // 2

        # Front: -30° to +30°
        front_start = center - 30
        front_end = center + 30

        # Left: +30° to +90°
        left_start = center + 30
        left_end = center + 90

        # Right: -90° to -30°
        right_start = center - 90
        right_end = center - 30

        self.front_distance = self.get_min_distance(
            ranges,
            front_start,
            front_end
        )

        self.left_distance = self.get_min_distance(
            ranges,
            left_start,
            left_end
        )

        self.right_distance = self.get_min_distance(
            ranges,
            right_start,
            right_end
        )

    def control_loop(self):

        cmd = Twist()

        front = self.front_distance
        left = self.left_distance
        right = self.right_distance

        # =====================================
        # 1. EMERGENCY STOP
        # =====================================

        if front < 0.30:

            cmd.linear.x = 0.0

            if left > right:
                cmd.angular.z = 1.2
                self.last_turn = 1.0
            else:
                cmd.angular.z = -1.2
                self.last_turn = -1.0

            self.get_logger().warn(
                f'EMERGENCY: obstacle {front:.2f} m'
            )

        # =====================================
        # 2. CLOSE OBSTACLE
        # =====================================

        elif front < 0.60:

            cmd.linear.x = 0.03

            if left > right:
                cmd.angular.z = 0.9
                self.last_turn = 1.0
            else:
                cmd.angular.z = -0.9
                self.last_turn = -1.0

            self.get_logger().info(
                f'Obstacle detected: {front:.2f} m'
            )

        # =====================================
        # 3. OBSTACLE APPROACHING
        # =====================================

        elif front <= 1.0:

            cmd.linear.x = 0.12

            if left > right:
                cmd.angular.z = 0.45
                self.last_turn = 1.0
            else:
                cmd.angular.z = -0.45
                self.last_turn = -1.0

        # =====================================
        # 4. CLEAR PATH
        # =====================================

        else:

            cmd.linear.x = 0.25

            # Small steering correction if one
            # side is significantly more open.
            difference = left - right

            if difference > 1.0:
                cmd.angular.z = 0.15

            elif difference < -1.0:
                cmd.angular.z = -0.15

            else:
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()