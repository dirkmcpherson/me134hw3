import sympy as sym
from sympy.matrices import rot_axis3
from IPython import embed
from math import sin, cos
import math

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
    A = sym.Matrix([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
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
def generate_simplified_transform_matrix_3_0():
    ### DEFINE STRUCTURAL CONSTANTS ####
    l1 = 0.1
    l2 = 0.1
    ### END DEFINE STRUCTURAL CONSTANTS ####

    theta0, l3 = sym.symbols('theta0 l3')
    A_0_1 = create_A_from_table(l1, 0, 0, theta0)
    A_1_2 = create_A_from_table(l2, 0, 0, -math.pi/2)
    A_2_3 = create_A_from_table(l3, 0, 0, math.pi/2)

    return A_0_1.multiply(A_1_2).multiply(A_2_3)


# def test_two_segment_arm():
    l1 = 1
    l2 = 2
    theta1 = math.pi / 4
    theta2 = math.pi / 6


    A_0_1 = create_A_from_table(l1, 0, 0, theta1)
    A_1_2 = create_A_from_table(l2, 0, 0, theta2)

    O = sym.Matrix([0,0,0,1]) # origin of frame

    p0 = [l1*cos(theta1), l1*sin(theta1), 0, 1] # Expected result
    p1 = [p0[0] + l2*cos(theta1+theta2),p0[1] +l2*sin(theta1+theta2), 0, 1]
    print(A_0_1.multiply(O))
    print(p0)
    print("==================")
    print(A_0_1.multiply(A_1_2).multiply(O))
    print(p1)

if __name__ == "__main__":
    # test with 2 length arm
    # test_two_segment_arm()

    T_0_3 = generate_simplified_transform_matrix_3_0()
    embed()



