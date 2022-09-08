# 필요한 패키지 import
import cv2 # OpenCV(실시간 이미지 프로세싱) 모듈

# 카메라 index 번호
camera = 1

# VideoCapture : 카메라 열기
capture = cv2.VideoCapture(camera)

# 원본 동영상 크기 정보
w = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
h = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("원본 동영상 너비(가로) : {}, 높이(세로) : {}".format(w, h))

# 동영상 크기 변환
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # 가로
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # 세로

# 변환된 동영상 크기 정보
w = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
h = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("변환된 동영상 너비(가로) : {}, 높이(세로) : {}".format(w, h))

while True:
    # read : 프레임 읽기
    # [return]
    # 1) 읽은 결과(True / False)
    # 2) 읽은 프레임
    retval, frame = capture.read()

    # 읽은 프레임이 없는 경우 종료
    if not retval:
        break
    
    # 프레임 출력
    cv2.imshow("resize_frame", frame)
    
    # 'q' 를 입력하면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 동영상 파일 또는 카메라를 닫고 메모리를 해제
capture.release()

# 모든 창 닫기
cv2.destroyAllWindows()