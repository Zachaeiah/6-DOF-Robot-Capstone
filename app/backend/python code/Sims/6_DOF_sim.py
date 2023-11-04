import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to perform 3D rotation
def rotate_3d(points, angles):
    theta_x, theta_y, theta_z = angles
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(theta_x), -np.sin(theta_x)],
                   [0, np.sin(theta_x), np.cos(theta_x)]])
    Ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                   [0, 1, 0],
                   [-np.sin(theta_y), 0, np.cos(theta_y)]])
    Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                   [np.sin(theta_z), np.cos(theta_z), 0],
                   [0, 0, 1]])
    R = np.dot(Rz, np.dot(Ry, Rx))
    return np.dot(points, R.T)

# Given parameters
radius = 1.0  # Example circle radius
theta = np.linspace(0, 2 * np.pi, 100)
points_before_rotation = np.column_stack([radius * np.cos(theta), radius * np.sin(theta), np.zeros_like(theta)])

# Define rotation angles for each axis
angles = (np.pi/10, 0, 0)  # Example rotation angles for X, Y, and Z axes

# Perform 3D rotation
points_after_rotation = rotate_3d(points_before_rotation, angles)

# Scale the 3D circle by its radius
points_after_rotation *= radius

# Project onto the XY plane for shadow
points_on_main_plane = points_after_rotation[:, :2]

# Set z-coordinate of shadow points to 0
points_on_main_plane = np.column_stack([points_on_main_plane, np.zeros_like(points_on_main_plane[:, 0])])

# Create 3D plot
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot 3D circle
ax.plot(points_after_rotation[:, 0], points_after_rotation[:, 1], points_after_rotation[:, 2]+radius, label='3D Circle', color='b')

# Plot 2D shadow
ax.plot(points_on_main_plane[:, 0], points_on_main_plane[:, 1], points_on_main_plane[:, 2], label='Shadow', color='r')

# Customize the plot
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('3D Circle and Its Shadow')

plt.show()
