import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_image
        )

        self.get_logger().info(
            'Camera sensor started'
        )

    def publish_image(self):

        image = Image()

        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'camera_link'

        image.height = 480
        image.width = 640
        image.encoding = 'rgb8'
        image.is_bigendian = 0
        image.step = 640 * 3

        # Black test image
        image.data = bytes(
            480 * 640 * 3
        )

        self.publisher.publish(image)


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

