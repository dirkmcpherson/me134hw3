#HW 3
# Arm driver
# James Staley

import time
import random
import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit

from kinematics import Solver
import numpy as np

class Driver():
    def __init__(self):
        # super(self).__init__()
        # Set channels to the number of servo channels on your kit.
        # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
        # self.kit = ServoKit(channels=16)

        self.axis0 = self.kit.servo[0] 
        self.axis1 = self.kit.servo[1]
        self.axis2 = self.kit.servo[2]

        # The global starting points for each letter we draw
        X_OFFSET = 0.35
        self.base_points = [(X_OFFSET,0), (X_OFFSET, 1), (X_OFFSET, 2)]

        self.solver = Solver()

    def letter_to_points(self, letter):
        # pass
        # return a list of points for drawing this letter

        # return [(random.random(), random.random()) for i in range(10)]
        if letter == "|":
            return [(0.,0.1*i) for i in range(10)]
        elif letter == "-":
            return [(0.1*i, 0.) for i in range(10)]
        else:
            raise ValueError("UNSUPPORTED LETTER")

    def cap_and_convert_theta(self, theta_rad):
        '''
        Convert theta to degrees and cap its value
        '''
        MAX = 180.
        MIN = 0.
        theta_deg = max(MIN, min(MAX, np.rad2deg(theta_rad)))
        print("Converted %.1f radians to %.1f degrees" %(theta_rad,theta_deg))

        return theta_deg
        
    
    def goto_point(self, p): # ignoring z since its up down
        print("         going to point ", p)
        # inverse kinematics to go from f(x,y) = [theta0, theta1]
        # we actually have a direct mapping from theta1 to x and theta2 to y
        theta0, theta1 = self.solver.get_goal_thetas(p)

        print("         Solve for thetas: %.1f, %.1f" % (theta0, theta1))

        # angle = self.cap_and_convert_theta(np.array(theta0, dtype=float))
        # angle = self.cap_and_convert_theta(np.array(theta1, dtype=float))

        # TODO: There definitely need to be a mapping from the derived thetas and the what's sent to the servos
        self.axis0.angle = self.cap_and_convert_theta(np.array(theta0, dtype=float))
        self.axis1.angle = self.cap_and_convert_theta(np.array(theta1, dtype=float))

        
    def draw_letter(self, letter, reference_point):
        '''
        Draw a letter from a list of points.
        Draw w/r to a base point
        '''
        points = self.letter_to_points(letter)
        for p in points:
            rel_point = (p[0] + reference_point[0], p[1] + reference_point[1])
            self.goto_point(p)
            time.sleep(0.1)

    def run(self, word):
        print("Writing ", word)
        if (len(word) > len(self.base_points)):
            print("ERROR: Can not print more than %d letters." % len(word))
            return

        for idx, l in enumerate(word):
            print("   writing ", l)
            base_point = self.base_points[idx]
            print("   going to base point %d: (%.1f, %.1f)" % (idx, base_point[0], base_point[1]))
            self.goto_point(base_point)
            self.draw_letter(l, base_point)


if __name__ == "__main__":
    d = Driver()
    word = "|-" # not implemented. Just draw a straight vertical line and a straight horizontal line
    
    d.run(word)

    

