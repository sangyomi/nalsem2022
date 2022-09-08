import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class MechaAutoshipImageSub(Node):
    def __init__(self):
        super().__init__("mecha_autoship_image_sub_node")
        self.subscription = self.create_subscription(
            Image, "Image", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info("Receiving video frame")
        origin_image = self.br.imgmsg_to_cv2(data)
        cv2.imshow("origin image", origin_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MechaAutoshipImageSub()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
