import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, RegionOfInterest
import cv2
from cv_bridge import CvBridge
import math


class MechaAutoshipImageColorFilter(Node):
    def __init__(self):
        super().__init__("mecha_autoship_image_color_filter_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("filter_color", None),
                ("custom_h_low", None),
                ("custom_h_high", None),
                ("custom_s_low", None),
                ("custom_s_high", None),
                ("custom_v_low", None),
                ("custom_v_high", None),
                ("filter_min_area", None),
                ("filter_shape", None),
                ("filter_epsilon", None),
                ("filter_ratio", None),
            ],
        )
        self.filter_color = (
            self.get_parameter_or(
                "filter_color", Parameter("filter_color", Parameter.Type.STRING, "red")
            )
            .get_parameter_value()
            .string_value
        )
        self.custom_h_low = (
            self.get_parameter_or(
                "custom_h_low", Parameter("custom_h_low", Parameter.Type.INTEGER, 600)
            )
            .get_parameter_value()
            .integer_value
        )
        self.custom_h_high = (
            self.get_parameter_or(
                "custom_h_high", Parameter("custom_h_high", Parameter.Type.INTEGER, 400)
            )
            .get_parameter_value()
            .integer_value
        )
        self.custom_s_low = (
            self.get_parameter_or(
                "custom_s_low", Parameter("custom_s_low", Parameter.Type.INTEGER, 600)
            )
            .get_parameter_value()
            .integer_value
        )
        self.custom_s_high = (
            self.get_parameter_or(
                "custom_s_high", Parameter("custom_s_high", Parameter.Type.INTEGER, 400)
            )
            .get_parameter_value()
            .integer_value
        )
        self.custom_v_low = (
            self.get_parameter_or(
                "custom_v_low", Parameter("custom_v_low", Parameter.Type.INTEGER, 600)
            )
            .get_parameter_value()
            .integer_value
        )
        self.custom_v_high = (
            self.get_parameter_or(
                "custom_v_high", Parameter("custom_v_high", Parameter.Type.INTEGER, 400)
            )
            .get_parameter_value()
            .integer_value
        )
        self.filter_min_area = (
            self.get_parameter_or(
                "filter_min_area", Parameter("filter_min_area", Parameter.Type.INTEGER, 10)
            )
            .get_parameter_value()
            .integer_value
        )
        self.filter_shape = (
            self.get_parameter_or(
                "filter_shape", Parameter("filter_shape", Parameter.Type.INTEGER, 0)
            )
            .get_parameter_value()
            .integer_value
        )
        self.filter_epsilon = (
            self.get_parameter_or(
                "filter_epsilon",
                Parameter("filter_epsilon", Parameter.Type.DOUBLE, 0.02),
            )
            .get_parameter_value()
            .double_value
        )
        self.filter_ratio = (
            self.get_parameter_or(
                "filter_ratio", Parameter("filter_ratio", Parameter.Type.DOUBLE, 0.85)
            )
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info("filter_color: %s" % (self.filter_color))

        self.subscription = self.create_subscription(
            Image, "Image", self.listener_callback, 1
        )
        self.br = CvBridge()

        self.publisher_ = self.create_publisher(RegionOfInterest, "ROI", 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.origin_img = []

    def listener_callback(self, data):
        self.get_logger().info("Receiving video frame")
        self.origin_img = self.br.imgmsg_to_cv2(data)

    def timer_callback(self):
        if len(self.origin_img) != 0:
            filter_img = self.get_image_by_color(self.origin_img, self.filter_color)
            cv2.imshow("test", filter_img)
            cv2.waitKey(1)
            x, y, w, h = self.get_image_shape(filter_img)

            roi = RegionOfInterest()
            roi.x_offset = x
            roi.y_offset = y
            roi.height = w
            roi.width = h
            print("###################################################")
            print(roi)
            self.publisher_.publish(roi)
            self.get_logger().info("Publishing color point")

    def get_image_by_color(self, img, color):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        result_img = []

        if color == "red":
            # H(Hue)가 360이 아닌 180으로 범위를 지정한 이유는 OpenCV 이미지 변수들은 8bit로 설정되어 있어서 최대 255까지만 표현할 수 있기 때문이다.
            red_mask1 = cv2.inRange(hsv_img, (0, 100, 0), (20, 255, 255))
            red_mask2 = cv2.inRange(hsv_img, (160, 100, 0), (180, 255, 255))
            result_img = red_mask1 + red_mask2

        elif color == "green":
            result_img = cv2.inRange(hsv_img, (40, 100, 0), (80, 255, 255))

        elif color == "blue":
            result_img = cv2.inRange(hsv_img, (100, 100, 0), (140, 255, 255))

        elif color == "custom":
            result_img = cv2.inRange(
                hsv_img,
                (self.custom_h_low, self.custom_s_low, self.custom_v_low),
                (self.custom_h_high, self.custom_s_high, self.custom_v_high),
            )

        return result_img

    def get_image_shape(self, img):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # 경계선 개수만큼 반복
        for cnt in contours:
            cnt_length = cv2.arcLength(cnt, True)
            cnt_area = cv2.contourArea(cnt)

            # 경계선의 너비가 최소 영역 이상일 때만 반복문 이어서 실행(조건 미달시 다음 경계선 좌표로 반복문 실행)
            if cnt_area < self.filter_min_area:
                continue

            approx = cv2.approxPolyDP(cnt, self.filter_epsilon * cnt_length, True)

            # 꼭짓점의 개수가 3개거나 4개일 때
            if len(approx) in [3, 4] and len(approx) == self.filter_shape:
                return self.get_image_center_xyhy(approx)

            # 꼭짓점의 개수가 4개 초과일 때
            elif len(approx) > 4:
                # ratio가 1에 가까울수록 원형
                ratio = 4 * math.pi * cnt_area / pow(cnt_length, 2)

                # 원 모양일 때
                if ratio > self.filter_ratio and self.filter_shape == 0:
                    return self.get_image_center_xyhy(approx)

                # 사각형 이상의 다각형일 때
                elif self.filter_shape == 5:
                    return self.get_image_center_xyhy(approx)

        return (0, 0, 0, 0)

    def get_image_center_xyhy(self, cnt):
        x, y, w, h = cv2.boundingRect(cnt)

        moment = cv2.moments(cnt)
        moment_x = int(moment["m10"] / moment["m00"])
        moment_y = int(moment["m01"] / moment["m00"])

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        return (x, y, w, h)


def main(args=None):
    rclpy.init(args=args)
    node = MechaAutoshipImageColorFilter()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
