import sympy as sym
from sympy.matrices import rot_axis3
from IPython import embed
# from math import sin, cos
import math
import numpy as np

# self.theta0 = sym.Symbol("theta0", real=True)
# self.theta1 = sym.Symbol("theta1", real=True)
# self.theta2 = sym.Symbol("theta2", real=True)
# self.theta3 = sym.Symbol("theta3", real=True)

def create_A(z_rotation, dx, dy, dz=0):
    '''
    Creates a homogeneous transform matrix that rotates theta about z and translates by dx,dy,dz 
    '''
    A = rot_axis3(z_rotation)
    A = A.col_insert(3, sym.Matrix([dx, dy, dz]))
    A = A.row_insert(3, sym.Matrix([[0, 0, 0, 1]]))

def create_A_from_table(a, alpha, d, theta):
    '''
    Use the DH parameters from link n to create a transform from origin n-1 to origin n
    From lecture slide 25
    '''
    # embed()
    A = sym.Matrix([
        [sym.cos(theta), -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha), a*sym.cos(theta)],
        [sym.sin(theta), sym.cos(theta)*sym.cos(alpha), -sym.cos(theta)*sym.sin(alpha), a*sym.sin(theta)],
        [0, sym.sin(alpha), sym.cos(alpha), d],
        [0,0,0,1]
    ])

    return A

'''
Generate the transform matrix from frame 3 (eef) to frame 0 (servo) for the simplified kinematic version
link    a   alpha   d   theta
1       l1    0     0     theta0
2       l2    0     0     -pi/2
3       l3    0     0     pi/2
'''
## Create the symbols where everyone can see them
theta0, theta1, l1, l2, l3 = sym.symbols('theta0 theta1 l1 l2 l3') # theta0,1 are control parameters

def generate_symbolic_transform_matrix_3_0():
    A_0_1 = create_A_from_table(l1, 0, 0, theta0)
    A_1_2 = create_A_from_table(l2, 0, 0, -math.pi/2)
    A_2_3 = create_A_from_table(l3, 0, 0, math.pi/2)

    return A_0_1.multiply(A_1_2).multiply(A_2_3)

def generate_subs_transform_matrix_3_0(theta0, theta2):
    l1 = .1
    l2 = .2
    l4 = .3
    l3 = 2*l4*math.cos(theta2)

    A_0_1 = create_A_from_table(l1, 0, 0, theta0)
    A_1_2 = create_A_from_table(l2, 0, 0, -math.pi/2)
    A_2_3 = create_A_from_table(l3, 0, 0, math.pi/2)

    return A_0_1.multiply(A_1_2).multiply(A_2_3)

class Solver():
    def __init__(self):
        self.theta0 = sym.Symbol("theta0", real=True)
        self.theta2 = sym.Symbol("theta2", real=True)

        self.px = sym.Symbol("px", real=True)
        self.py = sym.Symbol("py", real=True)

        # self.l1 = sym.Symbol("l1", real=True)
        # self.l2 = sym.Symbol("l2", real=True)
        # self.l3 = sym.Symbol("l3", real=True)
        # self.l4 = sym.Symbol("l4", real=True)

        ## TEMP VALUES FOR LS
        l1 = .1
        l2 = .2
        l4 = .3

        self.e1 = sym.Eq(self.px, l1*sym.cos(self.theta0) + l2*sym.sin(self.theta0) + sym.cos(self.theta0))
        self.e2 = sym.Eq(self.py, l1*sym.sin(self.theta0) - l2*sym.cos(self.theta0) - sym.cos(self.theta0) + 2*l4*sym.cos(self.theta2)*sym.sin(self.theta0))


    def get_goal_thetas(self, point):
        px, py = point

        l1 = .1
        l2 = .2
        l4 = .3

        e1 = l1*sym.cos(self.theta0) + l2*sym.sin(self.theta0) + sym.cos(self.theta0) - px
        e2 = l1*sym.sin(self.theta0) - l2*sym.cos(self.theta0) - sym.cos(self.theta0) + 2*l4*sym.cos(self.theta2)*sym.sin(self.theta0) - py

        # e1 = self.e1.subs(px, self.px)
        # e2 = self.e2.subs(py, self.py)

        print("Attempting to solve ik for x,y : %f, %f" % (px,py))
        solution = sym.nsolve(
        # solution = sym.solve(
            [e1, e2],
            [
                # self.l1,
                # self.l2,
                # self.l3,
                # self.l4,
                self.theta0,
                self.theta2,
                # self.px,
                # self.py
            ],
            [0.1, 0.2]
        )

        print(solution)
        return solution[0], solution[1]



# def test_two_segment_arm():
    # l1 = 1
    # l2 = 2
    # theta1 = math.pi / 4
    # theta2 = math.pi / 6


    # A_0_1 = create_A_from_table(l1, 0, 0, theta1)
    # A_1_2 = create_A_from_table(l2, 0, 0, theta2)

    # O = sym.Matrix([0,0,0,1]) # origin of frame

    # p0 = [l1*cos(theta1), l1*sin(theta1), 0, 1] # Expected result
    # p1 = [p0[0] + l2*cos(theta1+theta2),p0[1] +l2*sin(theta1+theta2), 0, 1]
    # print(A_0_1.multiply(O))
    # print(p0)
    # print("==================")
    # print(A_0_1.multiply(A_1_2).multiply(O))
    # print(p1)

if __name__ == "__main__":
    # test with 2 length arm
    # test_two_segment_arm()

    T_0_3 = generate_symbolic_transform_matrix_3_0()
    # print(T_0_3)

    point = np.array([0, 0])

    s = Solver()
    theta0, theta2 = s.get_goal_thetas(point)

    print("Solved to get %.2f, %2.f" % (theta0, theta2))

    T_0_3 = generate_subs_transform_matrix_3_0(theta0, theta2)
    fk_point = T_0_3.multiply(sym.Matrix([0,0,0,1]))

    print("ForwardK expects ", fk_point)




