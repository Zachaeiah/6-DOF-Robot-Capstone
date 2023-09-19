import numpy as np
import sympy as sym
from sympy import Matrix
from sympy import *
init_printing()

# Define constants
A0, A1, A2, A3, A4, A5, A6 = sym.symbols("A0 A1 A2 A3 A4 A5 A6", real=True)
Theta0, Theta1, Theta2, Theta3, Theta4, Theta5 = sym.symbols("Theta0 Theta1 Theta2 Theta3 Theta4 Theta5", real=True)
PI = sym.pi

def rotation_z(theta, matrix):
    rot_matrix = Matrix([
        [sym.cos(theta), -sym.sin(theta), 0, 0],
        [sym.sin(theta), sym.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]) * matrix
    return rot_matrix

def rotation_y(theta, matrix):
    rot_matrix = Matrix([
        [sym.cos(theta), 0, sym.sin(theta), 0],
        [0, 1, 0, 0],
        [-sym.sin(theta), 0, sym.cos(theta), 0],
        [0, 0, 0, 1]
    ]) * matrix
    return rot_matrix

def rotation_x(theta, matrix):
    rot_matrix = Matrix([
        [1, 0, 0, 0],
        [0, sym.cos(theta), -sym.sin(theta), 0],
        [0, sym.sin(theta), sym.cos(theta), 0],
        [0, 0, 0, 1]
    ]) * matrix
    return rot_matrix

def translate(dx, dy, dz, matrix):
    trans_matrix = Matrix([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ]) * matrix
    return trans_matrix

# Define the theta values as NumPy arrays
theta_values = {
    Theta0: 0.5,  # Replace with the actual values
    Theta1: 0.3,
    Theta2: 0.7,
    Theta3: 0.9,
    Theta4: 0.2,
    Theta5: 0.4
}

distances0_3 = {A0:123, A1:535, A2:114, A3:210}
distances3_6 = {A4:232, A5:110, A6:140}

# Define the transformations as NumPy arrays
base_matrix_np = sym.eye(4)

frame1_np = translate(0, 0, A0, base_matrix_np) * rotation_x(-PI, base_matrix_np) * rotation_z(Theta0, base_matrix_np)
frame2_np = translate(0, -A1, 0, frame1_np) * rotation_z(Theta1, frame1_np)
frame3_np = translate(0, -A3, A2, frame2_np) * rotation_x(PI, frame2_np) * rotation_x(Theta2, frame2_np)
Tranform0_3 = frame3_np.subs(distances0_3)
print(Tranform0_3)
# Tranform0_3_inv = Tranform0_3.inv()
# print("done 0_3")

# frame4_np = translate(0, 0, A4, base_matrix_np) * rotation_x(-PI, base_matrix_np) * rotation_z(Theta3, frame3_np)
# frame5_np = translate(0, -A6, A5, frame4_np) * rotation_x(PI, frame4_np) * rotation_z(Theta4, frame4_np)
# frame6_np = rotation_z(Theta5, frame5_np)
# Tranform3_6 = frame6_np.subs(distances3_6)
# print("done 3_5")

# Tranform0_6 = Matrix([
#         [1, 0, 0, 0],
#         [0, 1, 0, 0],
#         [0, 0, 1, 0],
#         [0, 0, 0, 1]
#     ])

# result = Tranform0_3_inv*Tranform0_6
# print(result)




