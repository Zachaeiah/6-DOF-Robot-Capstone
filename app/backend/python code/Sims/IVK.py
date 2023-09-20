import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RobotController:
    def __init__(self, urdf_file_path, active_links_mask):
        self.robot_chain = ikpy.chain.Chain.from_urdf_file(urdf_file_path, active_links_mask=active_links_mask)
        self.fig, self.ax = plot_utils.init_3d_figure()
        self.fig.set_figheight(9)
        self.fig.set_figwidth(13)
        self.lines = []

    def compute_inverse_kinematics(self, target_position, target_orientation, orientation_mode="Y"):
        ik = self.robot_chain.inverse_kinematics(target_position, target_orientation, orientation_mode=orientation_mode)
        return ik.tolist()

    def visualize_robot(self, joint_angles, target_position):
        self.ax.clear()
        self.robot_chain.plot(joint_angles, self.ax, target=target_position)
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(0, 0.6)
        plt.pause(0.01)

    def animate(self, target_list):
        def update(frame):
            target_position, target_orientation = target_list[frame]
            ik_solution = self.compute_inverse_kinematics(target_position, target_orientation)
            self.visualize_robot(ik_solution, target_position)

        ani = FuncAnimation(self.fig, update, frames=len(target_list), repeat=False)
        plt.show()

def main():
    urdf_file_path = r"E:\\Capstone\\app\backend\\python code\Sims\\ROBOT.urdf"
    active_links_mask = [True, True, False, True, True, False, True, True]

    controller = RobotController(urdf_file_path, active_links_mask)

    # List of target positions and orientations
    target_list = [
        ([1, 0, 0.5], [0, 1, 0]),
        ([0.5, 0, 0.5], [0, 0, 1]),
        # Add more targets as needed
    ]

    controller.animate(target_list)

if __name__ == "__main__":
    main()
    
