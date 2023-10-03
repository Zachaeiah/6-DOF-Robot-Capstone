import numpy as np
import math
import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt



# Define rotation matrices using NumPy
def rotation_z(theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta, cos_theta, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotation_y(theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [cos_theta, 0, sin_theta, 0],
        [0, 1, 0, 0],
        [-sin_theta, 0, cos_theta, 0],
        [0, 0, 0, 1]
    ])

def rotation_x(theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [1, 0, 0, 0],
        [0, cos_theta, -sin_theta, 0],
        [0, sin_theta, cos_theta, 0],
        [0, 0, 0, 1]
    ])

def tranlate(dx, dy, dz):
    return np.array([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ])

class RobotArm:
    def __init__(self):
        pass
    def Calulate_FK(self, angles):
        a0 = 241
        a1 = 535
        a2 = 114
        a3 = 210
        a4 = 232
        a5 = 110
        a6 = 246

        Frame_0 = np.eye(4)
        Frame_1 = tranlate(0, 0, a0)@rotation_z(angles[0])@rotation_x(np.pi/2)@rotation_z(angles[1])@Frame_0
        Frame_2 = Frame_1@tranlate(0, a1, 0)@rotation_z(angles[2])
        Frame_3 = Frame_2@tranlate(0, a3, -a2)@rotation_x(-np.pi/2)@rotation_z(angles[3])
        Frame_4 = Frame_3@tranlate(0, 0, a4)@rotation_x(np.pi/2)@rotation_z(angles[4])
        Frame_5 = Frame_4@tranlate(0, a6, -a5)@rotation_x(-np.pi/2)@rotation_z(angles[5])

        self.frames = [Frame_0, Frame_1, Frame_2, Frame_3, Frame_4, Frame_5]
    
    def plot_frames(self):
        origins = [frame[:3, 3] for frame in self.frames]
        x_axes = [frame[:3, 0] for frame in self.frames]
        y_axes = [frame[:3, 1] for frame in self.frames]
        z_axes = [frame[:3, 2] for frame in self.frames]

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        colors = ['r', 'g', 'b', 'c', 'm', 'y']

        for i, origin in enumerate(origins):
            self.ax.scatter(origin[0], origin[1], origin[2], c=colors[i], marker='o', label=f'Origin {i}')

        for i in range(len(self.frames) - 1):
            self.ax.quiver(origins[i][0], origins[i][1], origins[i][2],
                          origins[i + 1][0] - origins[i][0],
                          origins[i + 1][1] - origins[i][1],
                          origins[i + 1][2] - origins[i][2],
                          color='black', arrow_length_ratio=0.1, alpha=0.2)

        unit_vector_length = 100

        for i in range(len(self.frames)):
            self.ax.quiver(origins[i][0], origins[i][1], origins[i][2],
                          x_axes[i][0] * unit_vector_length, x_axes[i][1] * unit_vector_length, x_axes[i][2] * unit_vector_length,
                          color='r')
            self.ax.quiver(origins[i][0], origins[i][1], origins[i][2],
                          y_axes[i][0] * unit_vector_length, y_axes[i][1] * unit_vector_length, y_axes[i][2] * unit_vector_length,
                          color='g')
            self.ax.quiver(origins[i][0], origins[i][1], origins[i][2],
                          z_axes[i][0] * unit_vector_length, z_axes[i][1] * unit_vector_length, z_axes[i][2] * unit_vector_length,
                          color='b')

        self.ax.set_xlim([-700, 700])
        self.ax.set_ylim([-700, 700])
        self.ax.set_zlim([-700, 700])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.legend()

# Create a function to update the robot arm plot based on input angles
def update_robot_arm_plot():
    angles = [
        float(angle_entries[i].get()) for i in range(len(angle_entries))
    ]
    display_robot_arm_plot(angles)

# Create a function to display the robot arm plot in a Tkinter window
def display_robot_arm_plot(angles):
    robot_arm = RobotArm()
    robot_arm.Calulate_FK(angles)
    robot_arm.plot_frames()
    
    if hasattr(root, 'canvas_widget'):
        root.canvas_widget.get_tk_widget().destroy()

    canvas = FigureCanvasTkAgg(robot_arm.fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    root.canvas_widget = canvas

# Define your a_values
a_values = [241, 535, 114, 210, 232, 110, 246]

# Create a tkinter window with the robot arm plot and input textboxes

root = tk.Tk()
root.title("Robot Arm Plot")

# Create a frame to contain the textboxes and the "Update Plot" button
input_frame = tk.Frame(root)
input_frame.pack(side=tk.LEFT, padx=10, pady=10)

angle_entries = []
for i in range(6):
    label = tk.Label(input_frame, text=f"Angle {i + 1}:")
    label.pack()
    entry = tk.Entry(input_frame)
    entry.pack()
    angle_entries.append(entry)

update_button = tk.Button(input_frame, text="Update Plot", command=update_robot_arm_plot)
update_button.pack()

# Create a frame to contain the plot
plot_frame = tk.Frame(root)
plot_frame.pack(side=tk.LEFT)

root.mainloop()