#HW 3
# Arm driver
# James Staley

ON_RASPERRY_PI = True # Debug off the raspberry pi
DEBUG = True

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
import sys

TURTLE_DOWN = 70
TURTLE_UP = 40

TWO_LOWER_LIMIT = 80
TWO_UPPER_LIMIT = 110

ONE_LOWER_LIMIT = 80
ONE_UPPER_LIMIT = 140

ZERO_POINT = 89

class Driver():
    def __init__(self):
        # The global starting points for each letter we draw
        # Offsets from our (0,0) when theta0=theta2=0
        self.solver = Solver()
        self.X_OFFSET = 0.2140
        self.Y_OFFSET = 0.0350
        self.prev_theta0 = None
        self.prev_theta1 = None
        self.prev_theta2 = None

        if (ON_RASPERRY_PI):
            self.kit = ServoKit(channels=16)
            self.axis0 = self.kit.servo[0] 
            self.axis1 = self.kit.servo[1]
            self.axis2 = self.kit.servo[2]

            self.goto_point((self.X_OFFSET, self.Y_OFFSET))
        else:
            print("WARNING: Running in Debug Configuration. WILL NOT WORK ON RASPBERRY PI.")



        # Let's try to write in the y range of [-0.1, 0.1] and the x range of 0.1, 0.15] so we get a straight box
        self.base_points = [
            (self.X_OFFSET, self.Y_OFFSET)
            # (self.X_OFFSET + 0.1, self.Y_OFFSET + 0.33), 
            # (self.X_OFFSET + 0.1, self.Y_OFFSET + -0.33), 
            # (self.X_OFFSET + 0.1, self.Y_OFFSET + -0.1)
            ]


    def letter_to_points(self, letter):
        # pass
        # return a list of points for drawing this letter

        # TODO map ranges min max of writing area

        # return [(random.random(), random.random()) for i in range(10)]
        if letter == "|":
            return [(0.,0.005*i) for i in range(10)] # [0 to 0.05] relative
        elif letter == "-":
            return [(0.0066*i, 0.) for i in range(10)] # [0 to 0.066] relative
        else:
            raise ValueError("UNSUPPORTED LETTER")

    def cap_and_convert_theta(self, theta_rad):
        '''
        Convert theta to degrees and cap its value
        '''
        MAX = 180.
        MIN = 0.
        if (theta_rad > math.pi):
            theta_rad -= 2*math.pi

        theta_deg = np.rad2deg(theta_rad)
        theta_deg += ZERO_POINT
        theta_deg = max(MIN, min(MAX, theta_deg))
        if DEBUG:
            print("         Converted %.1f radians to %.1f degrees" %(theta_rad,theta_deg))

        return theta_deg

    def get_thetas(self, point):
        theta0, theta1 = self.solver.get_goal_thetas(point)

        while (theta0) > 2*math.pi:
            theta0 -= 2*math.pi

        while (theta1) > 2*math.pi:
            theta1 -= 2*math.pi

        if DEBUG:
            print("         Solved for thetas: %.1f, %.1f" % (theta0, theta1))

        theta0 = self.cap_and_convert_theta(np.array(theta0, dtype=float))
        theta1 = self.cap_and_convert_theta(np.array(theta1, dtype=float))

        return theta0, theta1

    def goto_point(self, p): # ignoring z since its up down
        print("Going to point ", p)
        # inverse kinematics to go from f(x,y) = [theta0, theta1]
        # we actually have a direct mapping from theta1 to x and theta2 to y
        theta0, theta1 = self.get_thetas(p)

        # TODO: There definitely need to be a mapping from the derived thetas and the what's sent to the servos
        if (ON_RASPERRY_PI):
            if (self.prev_theta0 is None or abs(self.prev_theta0 - theta0) > 0.1):
                fangle0 = max(ONE_LOWER_LIMIT, min(ONE_UPPER_LIMIT, theta0))
                if DEBUG:
                    print(f"     moving servo0 to {fangle0:0.1f}")
                self.axis0.angle = fangle0
                self.prev_theta0 = fangle0
            else:
                if DEBUG:
                    print("       Skipping 0 because its close to current position")

            if (self.prev_theta1 is None or abs(self.prev_theta1 - theta1) > 0.1):
                fangle1 = max(TWO_LOWER_LIMIT, min(TWO_UPPER_LIMIT, theta1))
                if DEBUG:
                    print(f"     moving servo1 to {fangle1:0.1f}")
                self.axis1.angle = fangle1
                self.prev_theta1 = fangle1
            else:
                if DEBUG:
                    print("       Skipping 1 because its close to current position")
        else:
            print(f"theta0 {theta0:.2f} theta1 {theta1:.2f}")


        
    def draw_letter(self, letter, reference_point):
        '''
        Draw a letter from a list of points.
        Draw w/r to a base point
        '''
        self.axis2.angle = TURTLE_DOWN
        points = self.letter_to_points(letter)
        for p in points:
            rel_point = (p[0] + reference_point[0], p[1] + reference_point[1])
            self.goto_point(rel_point)
            time.sleep(0.1)
        self.axis2.angle = TURTLE_UP

    def run(self, word):
        print("Writing ", word)
        # if (len(word) > len(self.base_points)):
        #     print("ERROR: Can not print more than %d letters." % len(word))
        #     return

        for idx, l in enumerate(word):
            print("   writing ", l)
            base_point = self.base_points[0]
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

    def get_zero_position(self):
        # Based on forward kinematics what're the thetas associated with getting the eef to (0,0)
        p = self.solver.zero_theta_position()
        l3 = self.solver.l3_length_at(0.)
        print(f"Zero theta puts eef at {p[0]:.4f}, {p[1]:.4f}. l3 starts out at length {l3:.4f}.")


if __name__ == "__main__":
    d = Driver()

    if len(sys.argv) == 3:
        x,y = float(sys.argv[1]), float(sys.argv[2])
        d.goto_point((x,y))
        sys.exit()
        

    if ON_RASPERRY_PI:
        # d.goto_point((0,0))
        # word = "|-" 
        word = "|-|"
        d.run(word)
    else:
        d.get_zero_position()
        # d.produce_discrete_table()
        test_points = []
        test_points.append((d.X_OFFSET, d.Y_OFFSET)) # starting point for 0thetas
        test_points.append((d.X_OFFSET - 0.005, d.Y_OFFSET)) # left a little
        test_points.append((d.X_OFFSET + 0.005, d.Y_OFFSET)) # right a little 
        
        
        test_points.append((d.X_OFFSET, d.Y_OFFSET)) # down a little
        test_points.append((d.X_OFFSET, d.Y_OFFSET + 0.0025)) # up a little
        test_points.append((d.X_OFFSET, d.Y_OFFSET - 0.0025)) # up a little
        for p in test_points:
            d.goto_point(p)
        # d.goto_point((float(sys.argv[1]), float(sys.argv[2])))

    

