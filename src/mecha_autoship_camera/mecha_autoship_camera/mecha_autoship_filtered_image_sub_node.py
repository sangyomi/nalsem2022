import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
import cv2
from cv_bridge import CvBridge


class MechaAutoshipFilteredImageSub(Node):
    def __init__(self):
        super().__init__("mecha_autoship_filtered_image_sub_node")

        self.image_subscription = self.create_subscription(
            Image, "Image", self.image_listener_callback, 10
        )
        self.roi_subscription = self.create_subscription(
            RegionOfInterest, "ROI", self.roi_listener_callback, 10
        )
        self.image_subscription  # prevent unused variable warning
        self.roi_subscription

        self.roi = []
        self.image = []
        self.br = CvBridge()


    def roi_listener_callback(self, data):
        self.get_logger().info("Receiving filter roi")
        print(data)
        self.roi = [data.x_offset, data.y_offset, data.width, data.height]
        if len(self.image) != 0 and len(self.roi) != 0:
            filter_image = self.image
            cv2.rectangle(
                filter_image,
                (self.roi[0], self.roi[1]),
                (self.roi[0] + self.roi[2], self.roi[1] + self.roi[3]),
                (0, 0, 255),
                3,
            )
            cv2.imshow("filtered image", filter_image)
            cv2.waitKey(1)

    def image_listener_callback(self, data):
        self.get_logger().info("Receiving frame")
        self.image = self.br.imgmsg_to_cv2(data)
        if len(self.image) != 0 and len(self.roi) != 0:
            filter_image = self.image
            cv2.rectangle(
                filter_image,
                (self.roi[0], self.roi[1]),
                (self.roi[0] + self.roi[2], self.roi[1] + self.roi[3]),
                (0, 0, 255),
                3,
            )
            cv2.imshow("filtered image", filter_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MechaAutoshipFilteredImageSub()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
