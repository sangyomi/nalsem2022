
import cv2
import math


filter_min_area = 50
filter_epsilon = 0.1


# Jetson에서 CSI 카메라 사용을 위한 Gstreamer 사용 함수
def gstreamer_pipeline(
    sensor_id=0,
    width=600,
    height=400,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            width,
            height,
            framerate,
            flip_method,
            width,
            height,
        )
    )


# 이미지를 hsv값을 기준으로 필터링
# H(Hue) : 색도 0 - 180
# S(Saturation) : 채도 0 - 255
# V(Value) : 명도 0 - 255
def get_image_by_color(img, color):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h, w, _ = hsv_img.shape
    print("hsv color : " + str(hsv_img[int(h / 2)][int(w / 2)]))
    result_img = []

    if color == "red":
        # H(Hue)가 360이 아닌 180으로 범위를 지정한 이유는
        # OpenCV 이미지 변수들은 8bit로 설정되어 있어서 최대 255까지만 표현 가능
        red_mask1 = cv2.inRange(hsv_img, (0, 100, 0), (20, 255, 255))
        red_mask2 = cv2.inRange(hsv_img, (160, 100, 0), (180, 255, 255))
        result_img = red_mask1 + red_mask2

    elif color == "green":
        result_img = cv2.inRange(hsv_img, (40, 100, 0), (80, 255, 255))

    elif color == "blue":
        result_img = cv2.inRange(hsv_img, (100, 100, 0), (140, 255, 255))

    elif color == "custom":
        result_img = cv2.inRange(hsv_img, (0, 100, 0), (80, 255, 255))

    return result_img


# 색체 탐지된 이미지 모양 추정
def get_image_shape(img):
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnt_list = []

    # 외곽선 개수만큼 반복
    for cnt in contours:
        # 외곽선 길이, 너비 구하기
        cnt_length = cv2.arcLength(cnt, True)
        cnt_area = cv2.contourArea(cnt)

        # 외곽선의 너비가 최소 영역 이상일 때만 반복문 이어서 실행(조건 미달시 다음 외곽선으로 반복문 실행)
        if cnt_area < filter_min_area:
            continue

        # 외곽선 근사화
        approx = cv2.approxPolyDP(cnt, filter_epsilon * cnt_length, True)

        # 꼭짓점의 개수가 3개거나 4개일 때
        if len(approx) in [3, 4]:
            cnt_list.append(approx)

        # 꼭짓점의 개수가 4개 초과일 때
        elif len(approx) > 4:
            # ratio가 1에 가까울수록 원형
            ratio = 4 * math.pi * cnt_area / pow(cnt_length, 2)

            # 원 모양일 때
            if ratio > 0.85:
                cnt_list.append(approx)

            # 사각형 이상의 다각형일 때
            else:
                cnt_list.append(approx)

    return cnt_list



cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024) # 가로
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 576) # 세로


while cap.isOpened():
    ret, frame = cap.read()

    if ret:
        # 원본 이미지 + 가운데 표식
        origin_image = frame.copy()
        h, w, _ = origin_image.shape
        cv2.circle(origin_image, (int(w / 2), int(h / 2)), 3, (0, 0, 200), -1)
        cv2.imshow("origin image", origin_image)

        # HSV로 필러링된 이미지
        hsv_image = get_image_by_color(frame, "red")
        cv2.imshow("hsv image", hsv_image)

        # 확인된 모양 테두리 표시
        filter_image = frame.copy()
        cnt_list = get_image_shape(hsv_image)
        print(cnt_list)

        for cnt in cnt_list:
            cv2.drawContours(filter_image, cnt, -1, (200, 0, 0), thickness=5)
        cv2.imshow("filter image", filter_image)

        cv2.waitKey(1)
