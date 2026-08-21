import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class MazeNavigator(Node):

    def __init__(self):
        super().__init__('maze_navigator')

        # =========================================================
        # WAYPOINTS
        # =========================================================
        #
        # The long wall blocks the direct route to the goal.
        #
        # We therefore:
        #
        # 1. Move underneath the wall
        # 2. Cross to the other side
        # 3. Move toward the final goal
        #
        # =========================================================

        self.waypoints = [
            (5.2, -3.2),
            (6.8, -3.2),
            (8.0, 0.0),
        ]

        self.current_waypoint = 0

        # =========================================================
        # ROBOT STATE
        # =========================================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.scan = None

        self.started = False
        self.finished = False

        # =========================================================
        # SUBSCRIBERS
        # =========================================================

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # =========================================================
        # PUBLISHER
        # =========================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'MAZE NAVIGATOR STARTED'
        )

        self.get_logger().info(
            'Waypoint 1: (5.2, -3.2)'
        )

        self.get_logger().info(
            'Waypoint 2: (6.8, -3.2)'
        )

        self.get_logger().info(
            'Goal:       (8.0,  0.0)'
        )

        self.get_logger().info(
            '========================================'
        )

    # =============================================================
    # ODOMETRY
    # =============================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # =============================================================
    # LIDAR
    # =============================================================

    def scan_callback(self, msg):

        self.scan = msg

    # =============================================================
    # ANGLE NORMALIZATION
    # =============================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # =============================================================
    # LIDAR SECTOR
    # =============================================================

    def sector_min(self, start_deg, end_deg):

        if self.scan is None:
            return 10.0

        start = math.radians(start_deg)
        end = math.radians(end_deg)

        start_index = int(
            (start - self.scan.angle_min)
            / self.scan.angle_increment
        )

        end_index = int(
            (end - self.scan.angle_min)
            / self.scan.angle_increment
        )

        start_index = max(
            0,
            start_index
        )

        end_index = min(
            len(self.scan.ranges),
            end_index
        )

        values = []

        for r in self.scan.ranges[
            start_index:end_index
        ]:

            if math.isnan(r):
                continue

            if math.isinf(r):
                continue

            if r < self.scan.range_min:
                continue

            values.append(r)

        if not values:
            return self.scan.range_max

        return min(values)

    # =============================================================
    # STOP ROBOT
    # =============================================================

    def stop_robot(self):

        cmd = Twist()

        self.cmd_pub.publish(cmd)

    # =============================================================
    # MOVE TOWARD CURRENT WAYPOINT
    # =============================================================

    def navigate_to_waypoint(self):

        target_x, target_y = self.waypoints[
            self.current_waypoint
        ]

        dx = target_x - self.x
        dy = target_y - self.y

        distance = math.sqrt(
            dx * dx +
            dy * dy
        )

        # =========================================================
        # WAYPOINT REACHED
        # =========================================================

        if distance < 0.25:

            self.stop_robot()

            self.get_logger().info(
                f'Waypoint {self.current_waypoint + 1} reached: '
                f'({target_x:.2f}, {target_y:.2f})'
            )

            self.current_waypoint += 1

            # =====================================================
            # FINAL GOAL REACHED
            # =====================================================

            if self.current_waypoint >= len(self.waypoints):

                self.finished = True

                self.stop_robot()

                self.get_logger().info(
                    '========================================'
                )

                self.get_logger().info(
                    '              GOAL REACHED!'
                )

                self.get_logger().info(
                    f'Final position: '
                    f'({self.x:.2f}, {self.y:.2f})'
                )

                self.get_logger().info(
                    '========================================'
                )

            return

        # =========================================================
        # TARGET ANGLE
        # =========================================================

        target_angle = math.atan2(
            dy,
            dx
        )

        angle_error = self.normalize_angle(
            target_angle - self.yaw
        )

        # =========================================================
        # LIDAR
        # =========================================================

        front = self.sector_min(
            -20,
            20
        )

        front_left = self.sector_min(
            20,
            70
        )

        front_right = self.sector_min(
            -70,
            -20
        )

        left = self.sector_min(
            70,
            110
        )

        right = self.sector_min(
            -110,
            -70
        )

        # =========================================================
        # COMMAND
        # =========================================================

        cmd = Twist()

        # =========================================================
        # EMERGENCY OBSTACLE
        # =========================================================

        if front < 0.45:

            cmd.linear.x = -0.03

            if front_left > front_right:

                cmd.angular.z = 1.0

            else:

                cmd.angular.z = -1.0

        # =========================================================
        # CLOSE OBSTACLE
        # =========================================================

        elif front < 0.75:

            cmd.linear.x = 0.04

            if front_left > front_right:

                cmd.angular.z = 0.8

            else:

                cmd.angular.z = -0.8

        # =========================================================
        # OBSTACLE AHEAD
        # =========================================================

        elif front < 1.10:

            cmd.linear.x = 0.08

            if front_left > front_right:

                cmd.angular.z = 0.65

            else:

                cmd.angular.z = -0.65

        # =========================================================
        # NORMAL NAVIGATION
        # =========================================================

        else:

            # Proportional heading controller

            cmd.angular.z = 1.5 * angle_error

            # Limit turning

            cmd.angular.z = max(
                min(
                    cmd.angular.z,
                    0.8
                ),
                -0.8
            )

            # =====================================================
            # SPEED CONTROL
            # =====================================================

            if abs(angle_error) > 1.0:

                # Turn mostly in place

                cmd.linear.x = 0.0

            elif abs(angle_error) > 0.6:

                cmd.linear.x = 0.08

            elif abs(angle_error) > 0.3:

                cmd.linear.x = 0.16

            else:

                cmd.linear.x = 0.28

        # =========================================================
        # SIDE SAFETY
        # =========================================================

        if left < 0.30:

            cmd.angular.z -= 0.35

        if right < 0.30:

            cmd.angular.z += 0.35

        # =========================================================
        # FINAL LIMITS
        # =========================================================

        cmd.linear.x = max(
            min(
                cmd.linear.x,
                0.28
            ),
            -0.04
        )

        cmd.angular.z = max(
            min(
                cmd.angular.z,
                1.0
            ),
            -1.0
        )

        # =========================================================
        # PUBLISH
        # =========================================================

        self.cmd_pub.publish(cmd)

        # =========================================================
        # STATUS
        # =========================================================

        self.get_logger().info(
            f'WP={self.current_waypoint + 1}/'
            f'{len(self.waypoints)} '
            f'Pos=({self.x:.2f},{self.y:.2f}) '
            f'Target=({target_x:.2f},{target_y:.2f}) '
            f'D={distance:.2f} '
            f'YawErr={math.degrees(angle_error):.1f} '
            f'Front={front:.2f}',
            throttle_duration_sec=1.0
        )

    # =============================================================
    # CONTROL LOOP
    # =============================================================

    def control_loop(self):

        if self.finished:

            self.stop_robot()

            return

        if self.scan is None:

            return

        self.navigate_to_waypoint()


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    rclpy.init(args=args)

    node = MazeNavigator()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Don't publish after shutdown has started.
        # Just destroy the node cleanly.

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()