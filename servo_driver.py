#HW 3
# Arm driver
# James Staley

ON_RASPERRY_PI = False # Debug off the raspberry pi

import time
import random

if (ON_RASPERRY_PI):
    import board
    import busio
    import adafruit_pca9685
    from adafruit_servokit import ServoKit

from kinematics import Solver
import numpy as np
import math

class Driver():
    def __init__(self):
        if (ON_RASPERRY_PI):
            self.kit = ServoKit(channels=16)
            self.axis0 = self.kit.servo[0] 
            self.axis1 = self.kit.servo[1]
            self.axis2 = self.kit.servo[2]
        else:
            print("WARNING: Running in Debug Configuration. WILL NOT WORK ON RASPBERRY PI.")

        # The global starting points for each letter we draw
        # Offsets from our (0,0)
        X_OFFSET = 0.35
        Y_OFFSET = 0.
        self.base_points = [
            (X_OFFSET, Y_OFFSET + 0), 
            (X_OFFSET, Y_OFFSET + 1), 
            (X_OFFSET, Y_OFFSET + 2)
            ]

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

    def get_thetas(self, point):
        theta0, theta1 = self.solver.get_goal_thetas(point)

        while (theta0) > 2*math.pi:
            theta0 -= 2*math.pi

        while (theta1) > 2*math.pi:
            theta1 -= 2*math.pi

        print("         Solve for thetas: %.1f, %.1f" % (theta0, theta1))

        theta0 = self.cap_and_convert_theta(np.array(theta0, dtype=float))
        theta1 = self.cap_and_convert_theta(np.array(theta1, dtype=float))

        return theta0, theta1

    def goto_point(self, p): # ignoring z since its up down
        print("         going to point ", p)
        # inverse kinematics to go from f(x,y) = [theta0, theta1]
        # we actually have a direct mapping from theta1 to x and theta2 to y
        theta0, theta1 = self.get_thetas(p)

        # TODO: There definitely need to be a mapping from the derived thetas and the what's sent to the servos
        if (ON_RASPERRY_PI):
            self.axis0.angle = theta0
            self.axis1.angle = theta1
        else:
            angle = theta0
            angle = theta1

        
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

    def produce_discrete_table(self):
        '''
        For debugging, produce a table of x,y,theta0,theta1 values
        '''
        X = np.linspace(0, 2., 10)
        Y = np.linspace(0, 3., 10)
        print("X    Y   th0   th2")
        for x in X:
            for y in Y:
                th0, th2 = self.get_thetas([x,y])
                print(f"{x:.2f}     {y:.2f}     {th0:.2f}      {th2:.2f}")


if __name__ == "__main__":
    d = Driver()
    word = "|-" # not implemented. Just draw a straight vertical line and a straight horizontal line
    
    if ON_RASPERRY_PI:
        d.run(word)
    else:
        d.produce_discrete_table()

    

