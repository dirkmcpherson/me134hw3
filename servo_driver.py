#HW 3
# Arm driver
# James Staley

import time
import random
import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit

class Driver():
    def __init__(self):
        # super(self).__init__()
        # Set channels to the number of servo channels on your kit.
        # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
        self.kit = ServoKit(channels=16)

        self.axis0 = self.kit.servo[0] 
        self.axis1 = self.kit.servo[1]
        self.axis2 = self.kit.servo[2]

        # The global starting points for each letter we draw
        self.base_points = [(0,0), (0, 1), (0, 2)]

    def letter_to_points(self, letter):
        # pass
        # return a list of points for drawing this letter
        return [(random.random(), random.random()) for i in range(10)]

    def goto_point(self, p): # ignoring z since its up down
        print("going to point ", p)
        # inverse kinematics to go from f(x,y) = [theta0, theta1]
        # we actually have a direct mapping from theta1 to x and theta2 to y

        theta0, theta1 = 0., 0. # TODO: the work of the inverse kinematics

    def draw_letter(self, letter):
        points = self.letter_to_points(letter)
        for p in points:
            self.goto_point(p)
            time.sleep(0.05)

    def run(self, word):
        print("Writing ", word)
        if (len(word) > len(self.base_points)):
            print("ERROR: Can not print more than %d letters." % len(word))
            return

        for idx, l in enumerate(word):
            print("   writing ", l)
            self.goto_point(self.base_points[idx])
            self.draw_letter(l)


if __name__ == "__main__":
    d = Driver()
    word = "js"
    
    d.run(word)

    

