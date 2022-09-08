import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


def gstreamer_pipeline(
    sensor_id=0, width=600, height=400, framerate=30, flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (sensor_id, width, height, framerate, flip_method, width, height,)
    )


class MechaAutoshipImagePub(Node):
    def __init__(self):
        super().__init__("mecha_autoship_image_pub_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("camera_id", None),
                ("image_width", None),
                ("image_height", None),
            ],
        )
        _camera_id = (
            self.get_parameter_or(
                "camera_id", Parameter("camera_id", Parameter.Type.INTEGER, 0)
            )
            .get_parameter_value()
            .integer_value
        )
        _image_width = (
            self.get_parameter_or(
                "image_width", Parameter("image_width", Parameter.Type.INTEGER, 600)
            )
            .get_parameter_value()
            .integer_value
        )
        _image_height = (
            self.get_parameter_or(
                "image_height", Parameter("image_height", Parameter.Type.INTEGER, 400)
            )
            .get_parameter_value()
            .integer_value
        )
        self.get_logger().info("camera_id: %s" % (_camera_id))
        self.get_logger().info("image_width: %s" % (_image_width))
        self.get_logger().info("image_height: %s" % (_image_height))

        self.publisher_ = self.create_publisher(Image, "Image", 1)
        publish_timer_period = 0.1
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_timer_callback)

        camera_timer_period = 0.001
        self.camera_timer = self.create_timer(camera_timer_period, self.camera_timer_callback)
        self.frame_id = "camera"
        self.cap = cv2.VideoCapture(
            gstreamer_pipeline(
                sensor_id=_camera_id, width=_image_width, height=_image_height
            ),
            cv2.CAP_GSTREAMER,
        )
        self.br = CvBridge()
        self.frame = []

    def publish_timer_callback(self):
        if len(self.frame) != 0:
            msg = self.br.cv2_to_imgmsg(self.frame)
            msg.header.frame_id = str(self.frame_id)
            msg.header.stamp = super().get_clock().now().to_msg()
            self.publisher_.publish(msg)
            self.get_logger().info("Publishing video frame")

    def camera_timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.frame = frame
        else:
            self.get_logger().info("Fail to get video frame")


def main(args=None):
    rclpy.init(args=args)
    node = MechaAutoshipImagePub()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
