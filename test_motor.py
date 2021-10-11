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

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("ERROR: usage is $python test_motor.py {servo0_cmd} {servo1_cmd} {servo2_cmd}")
        print("Please enter a value for each of the three servos.")
        sys.exit()

    s0 = sys.argv[1]
    s1 = sys.argv[2]
    s2 = sys.argv[3]

    print(f"Setting servo positions to angles (degrees) {s0}, {s1}, {s2}")

    kit = ServoKit(channels=16)
    axis0 = kit.servo[0] 
    axis1 = kit.servo[1]
    axis2 = kit.servo[2]

    axis0.angle = s0
    axis1.angle = s1
    axis2.angle = s2

    

