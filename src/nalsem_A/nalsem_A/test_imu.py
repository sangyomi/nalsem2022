import numpy as np
import nalsem_imu
import time
from math import *

while True:

    time.sleep(0.8)

    A = nalsem_imu.find_heading()

    print(A)

