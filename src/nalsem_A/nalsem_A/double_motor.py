from adafruit_servokit import ServoKit
import board
import busio
import time


class Motor:
    # 초기화
    def __init__(self):
        print("Motor init")

        # i2c 통신을 젯슨 나노의 27,28번 핀으로 정의합니다
        i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
        print("clear first")
        # servo_kit 객체를 생성하고 거기에 서보모터 드라이버를 연결합니다
        """
        +추가설명
        보통 부품같은 것들을 사용하기위해 코드를 짤 때는
        제일 먼저 내가 사용할 부품에 대한 객체를 만들어줍니다.
        즉 내가 부품을 2개 사용한다면 2개를 만들고, 5개를 사용하면 5개 각각에 해당하는 객체를 만듭니다.
        그리고 이번 선박 프로젝트에서는 1개의 i2c 드라이버 모듈을 사용하므로 1개의 객체만 만들어도 됩니다.

        channel 관련해서 질문도 있었습니다.
        channel은 단순히 이 모듈에 서보모터를 몇개까지 연결하고
        각각 따로 제어할 수 있는가를 나타내는 것이라고 보시면 됩니다.
        즉 16개의 서보모터를 동시에 제어할 수 있는 모듈이니 채널은 16을 사용하시면 됩니다
        """
        self.servo_kit = ServoKit(channels=16, i2c=i2c_bus0)
        print("clear second")
        """클래스 이름을 servo_kit으로 수정하였고 밑의 클래스는 지웠습니다!"""


        """
        +추가설명
        위에서는 이 i2c 드라이버 모듈 설정에 관한 코드였다면
        이제 여기서 각각의 모터 제어에 관해서 코드가 나옵니다
        self.servo_kit 이라는 i2c 드라이버 객체의 servo 라는 객체를 활용하는데
        여기서 servo[번호]가 바로 모터의 핀번호입니다(0부터 시작)
        즉 첫번째 핀과 두번째 핀에 BLDC 모터를 2개 연결했다면
        그냥 self.servo_kit.servo[0] 과 self.servo_kit.servo[1]을 사용하시면 됩니다
        """

        """클래스 이름이 수정되었고 밑의 클래스는 삭제하였습니다!"""    
        self.servo_kit.servo[0].angle = 90
        self.servo_kit.servo[1].angle = 90
        
        print("clear third")


        # 2초정도 기다립니다
        time.sleep(2)
       # self.servo_kit.servo[0].set_pulse_width_range(, )
       # self.servo_kit.servo[1].set_pulse_width_range(0, 180)
      
        print("clear fourth")

    # 모터 동작 함수

    """모터 동작 함수는 좌우측 BLDC 2개의 속도를 인자값으로 받기 때문에 함수를 간단화하였습니다"""
    def motor_move(self, speedleft: int, speedright: int):
        self.servo_kit.servo[0].angle = speedleft
        self.servo_kit.servo[1].angle = speedright
     
        # 에러를 방지하기 위해 0.02초 지연합니다
        time.sleep(1)
        print("all clear")
   

    # 종료
    def __del__(self):
        print("Motor stop")
        self.servo_kit.servo[0].angle = 90
        self.servo_kit.servo[1].angle = 90
     
        del self.servo_kit
 
 



#        self.servo_kit.servo[0].angle = 90 - (speedleft * (90/100))
#        self.servo_kit.servo[1].angle = 90 - (speedright * (90/100))
#        self.motor_move(degree, speed)
