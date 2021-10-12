#HW 3
# test three servos
# James Staley

import time
import random
import sys

import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit

TURTLE_DOWN = 70
TURTLE_UP = 40

TWO_LOWER_LIMIT = 80
TWO_UPPER_LIMIT = 110
TWO_INC = 15

ONE_LOWER_LIMIT = 80
ONE_UPPER_LIMIT = 140
ONE_INC = 30

SLEEP_INC = 0.05

ONE_ZERO_POINT = ONE_LOWER_LIMIT + (ONE_UPPER_LIMIT - ONE_LOWER_LIMIT) / 2.
TWO_ZERO_POINT = TWO_LOWER_LIMIT + (TWO_UPPER_LIMIT - TWO_LOWER_LIMIT) / 2.

class FastBoy():
    def __init__(self):
        kit = ServoKit(channels=16)
        self.axis0 = kit.servo[0] 
        self.axis1 = kit.servo[1]
        self.axis2 = kit.servo[2]

        # self.X_OFFSET = 0.1043
        # self.Y_OFFSET = 0.0350

    def up(self):
        self.axis0.angle = TURTLE_UP
    def down(self):
        self.axis0.angle = TURTLE_DOWN

    def goto(self, th0,th1):
        self.axis0.angle = max(ONE_LOWER_LIMIT, min(ONE_UPPER_LIMIT, th0))
        self.axis1.angle = max(TWO_LOWER_LIMIT, min(TWO_UPPER_LIMIT, th1))

    def make_letters(self):
        self.up()
        time.sleep(SLEEP_INC)
        self.goto(ONE_ZERO_POINT, TWO_ZERO_POINT)
        time.sleep(SLEEP_INC)
        self.down()
        time.sleep(SLEEP_INC)
        self.goto(ONE_ZERO_POINT, TWO_ZERO_POINT - TWO_INC)
        time.sleep(SLEEP_INC)
        self.up()
        time.sleep(SLEEP_INC)
        self.goto(ONE_ZERO_POINT - ONE_INC, TWO_ZERO_POINT)
        time.sleep(SLEEP_INC)
        self.down()
        time.sleep(SLEEP_INC)
        self.goto(ONE_ZERO_POINT + ONE_INC, TWO_ZERO_POINT)
        time.sleep(SLEEP_INC)
        self.up()
        


if __name__ == "__main__":
    fb = FastBoy()
    fb.make_letters()
    # if len(sys.argv) < 4:
    #     print("ERROR: usage is $python test_motor.py {servo0_cmd} {servo1_cmd} {servo2_cmd}")
    #     print("Please enter a value for each of the three servos.")
    #     sys.exit()

    # s0 = sys.argv[1]
    # s1 = sys.argv[2]
    # s2 = sys.argv[3]

    # kit = ServoKit(channels=16)
    # axis0 = kit.servo[0] 
    # axis1 = kit.servo[1]
    # axis2 = kit.servo[2]

    # s0 = int(s0)
    # s1 = int(s1)
    # s2 = int(s2)

    # start = 89

    # s0 = max(start-20, min(start+20, s0))
    # s1 = max(start-20, min(start+20, s1))
    # s2 = max(start-20, min(start+20, s2))


    # print(f"Setting servo positions to angles (degrees) {s0}, {s1}, {s2}")


    # axis0.angle = s0
    # axis1.angle = s1
    # axis2.angle = s2

    

