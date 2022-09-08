import cv2
import numpy as np
import time

def nothing(x):
    # any operation
    pass

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024) # 가로
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 576) # 세로

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 75, 180, nothing)
cv2.createTrackbar("L-S", "Trackbars", 225, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 105, 255, nothing)
cv2.createTrackbar("U-H", "Trackbars", 145, 180, nothing)
cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 190, 255, nothing)

font = cv2.FONT_HERSHEY_COMPLEX

def get_image_center_xyhy(cnt):
    x, y, w, h = cv2.boundingRect(cnt)

    return (x)

def camera():
    while True:
        D = 5
        _, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        l_h = cv2.getTrackbarPos("L-H", "Trackbars")
        l_s = cv2.getTrackbarPos("L-S", "Trackbars")
        l_v = cv2.getTrackbarPos("L-V", "Trackbars")
        u_h = cv2.getTrackbarPos("U-H", "Trackbars")
        u_s = cv2.getTrackbarPos("U-S", "Trackbars")
        u_v = cv2.getTrackbarPos("U-V", "Trackbars")

        lower_red = np.array([l_h, l_s, l_v])
        upper_red = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel)

        # Contours detection
        if int(cv2.__version__[0]) > 3:
            # Opencv 4.x.x
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            # Opencv 3.x.x
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 400:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                if len(approx) == 3 and D == 1:
                    cv2.putText(frame, "Triangle", (x, y), font, 1, (0, 0, 0))
                    print("triangle")
                elif len(approx) == 4 and D == 2:
                    cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))
                    print("rectangle")
                elif len(approx) == 12 and D == 3:
                    cv2.putText(frame, "Cross", (x, y), font, 1, (0, 0, 0))
                    print("cross")
                elif len(approx) == 10 and D == 4:
                    cv2.putText(frame, "Star", (x, y), font, 1, (0, 0, 0))
                    print("star")               
                elif 7 < len(approx) < 20 and D == 5:
                    cv2.putText(frame, "Circle", (x, y), font, 1, (0, 0, 0))
                    print("circle")

        cv2.circle(frame, (int(1024 / 2), int(576 / 2)), 3, (0, 0, 200), -1)
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1)
        if key == 27:
            break

while True :   
    X = camera()
    print(X)


cap.release()
cv2.destroyAllWindows()