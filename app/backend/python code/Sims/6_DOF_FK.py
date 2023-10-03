import numpy as np
import sympy as sym
from sympy import Matrix
from sympy import *
import math
init_printing()

# Define constants
A0, A1, A2, A3, A4, A5, A6 = sym.symbols("A0 A1 A2 A3 A4 A5 A6", real=True)
Theta0, Theta1, Theta2, Theta3, Theta4, Theta5 = sym.symbols("Theta0 Theta1 Theta2 Theta3 Theta4 Theta5", real=True)
PI = sym.pi

def rotation_z(theta, matrix):
    rot_matrix = Matrix([
        [sym.cos(theta), -sym.sin(theta), 0],
        [sym.sin(theta), sym.cos(theta), 0],
        [0, 0, 1]
    ]) * matrix
    return rot_matrix

def rotation_y(theta, matrix):
    rot_matrix = Matrix([
        [sym.cos(theta), 0, sym.sin(theta)],
        [0, 1, 0],
        [-sym.sin(theta), 0, sym.cos(theta)]
    ]) * matrix
    return rot_matrix

def rotation_x(theta, matrix):
    rot_matrix = Matrix([
        [1, 0, 0],
        [0, sym.cos(theta), -sym.sin(theta)],
        [0, sym.sin(theta), sym.cos(theta)]
    ]) * matrix
    return rot_matrix


# Calculate theta0, theta1, and theta2 using your formulas
a0 = 0
a1 = 350
a2 = 250

x =  100
y =  100
z =  100

Transform0_6 = Matrix([[-1, 0, 0],
                       [0, -1, 0],
                       [0, 0, 1]])

theta0 = math.degrees(math.atan2(y, x))
rotLength = math.sqrt(x**2 + y**2)
length0_3 = math.sqrt(rotLength**2 + (z - a0)**2)
beta = math.atan2((z - a0), length0_3)
alpha = math.acos((a2**2 - length0_3**2 - a1**2) / (-2 * length0_3 * a1))
theta1 = math.degrees(beta + alpha)
theta2 = math.degrees(math.atan2((z - a0) - a1 * math.sin(math.radians(theta1)),
                                 length0_3 - a1 * math.cos(math.radians(theta1))))

# Define the transformations as NumPy arrays
base_matrix_sym = sym.eye(3)

# Calculate the transformation matrix
frame1_sym = rotation_x(sym.pi/2, base_matrix_sym) * rotation_z(Theta0, base_matrix_sym) 
frame2_sym = rotation_z(Theta1, frame1_sym)
frame3_sym = rotation_x(Theta2, frame2_sym)
Transform0_3 = frame3_sym

# Substitute the numerical values into the matrix
Transform0_3_with_values = Transform0_3.subs({Theta0: math.radians(theta0), Theta1: math.radians(theta1), Theta2: math.radians(theta2)})

Transform0_3_with_values_inv = Transform0_3_with_values.inv()

Finle_M = Transform0_3_with_values_inv*Transform0_6
# for row in range(3):
#     for column in range(3):
#         value = Transform3_6[row, column].evalf()
#         print(f'Transform3_6[{row}, {column}]: \n{value}\n')

# print(Finle_M)

t5 = math.acos(Finle_M[2,2].evalf())
t6 = math.acos(Finle_M[2,0].evalf()/-math.sin(t5))
t4 = math.acos(Finle_M[1,2].evalf()/math.sin(t5))

# print(f'theta5: {t5}, radians')
# print(f'theta6: {t6}, radians')
# print(f'theta6: {t4}, radians')

R3_6_check = [[-math.sin(t4)*math.cos(t5)*math.cos(t6) - math.cos(t4)*math.sin(t6), math.sin(t4)*math.cos(t5)*math.sin(t6) - math.cos(t4)*math.cos(t6), -math.sin(t4)*math.sin(t5)],
                [-math.cos(t4)*math.cos(t5)*math.cos(t6) - math.sin(t4)*math.sin(t6), -math.cos(t4)*math.cos(t5)*math.sin(t6) - math.sin(t4)*math.cos(t6), -math.cos(t4)*math.sin(t5)],
                [-math.sin(t5)*math.cos(t6), math.sin(t5)*math.sin(t6) + math.cos(t5), math.cos(t5)]]



print(Finle_M)

print('\n\n')

print(R3_6_check)





