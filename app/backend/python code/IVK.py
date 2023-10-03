import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from intrerpolation import *

class RobotController:
    def __init__(self, urdf_file_path, active_links_mask):
        self.robot_chain = ikpy.chain.Chain.from_urdf_file(urdf_file_path, active_links_mask=active_links_mask)
        self.fig, self.ax = plot_utils.init_3d_figure()
        self.fig.set_figheight(9)
        self.fig.set_figwidth(13)
        self.lines = []

    def compute_inverse_kinematics(self, target_position, target_orientation, orientation_mode="Y"):
        """
        Compute inverse kinematics for the robot chain.

        Args:
            target_position (list or numpy.ndarray): The target position in 3D space (x, y, z).
            target_orientation (list or numpy.ndarray): The target orientation in 3D space (e.g., [0, 0, -1] for Z-axis).
            orientation_mode (str): The orientation mode ('Y' for Y-axis by default).

        Returns:
            list: A list of joint angles representing the inverse kinematics solution.

        Raises:
            ValueError: If the input parameters are not valid.
        """
        # Input validation
        if not isinstance(target_position, (list, np.ndarray)) or len(target_position) != 3:
            raise ValueError("Invalid target_position. It must be a list or numpy array of length 3.")
        
        if not isinstance(target_orientation, (list, np.ndarray)) or len(target_orientation) != 3:
            raise ValueError("Invalid target_orientation. It must be a list or numpy array of length 3.")
        
        if orientation_mode not in ["X", "Y", "Z"]:
            raise ValueError("Invalid orientation_mode. It must be 'X', 'Y', or 'Z'.")

        # Compute inverse kinematics
        ik = self.robot_chain.inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation,
            orientation_mode=orientation_mode
        )
        
        return ik.tolist()



    def visualize_robot(self, joint_angles, target_position):
        self.ax.clear()
        self.robot_chain.plot(joint_angles, self.ax, target=target_position)
        
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(0, 0.6)
        plt.xlabel("X")
        plt.ylabel("Y")
        self.ax.set_zlabel("Z")
        #plt.pause(0.001)

    def animate(self, target_list):
        def update(frame):
            target_position, target_orientation = target_list[frame]
            ik_solution = self.compute_inverse_kinematics(target_position, target_orientation)
            self.visualize_robot(ik_solution, target_position)

        ani = FuncAnimation(self.fig, update, frames=len(target_list), repeat=False)
        plt.show()


def main():
    urdf_file_path = r"E:\\Capstone\\app\backend\\python code\\ROBOT.urdf"
    active_links_mask = [True, True, True, True, True, True, True, True]

    #active_links_mask = [True, True]

    controller = RobotController(urdf_file_path, active_links_mask)

    # Create an instance of the PathPlanner class
    planner = PathPlanner()

    # Define start and end points for the paths
    start_point_linear = (2, 2, 2)
    end_point_linear = (-1, -1, -1)

    start_point_circular = (-1, -0.2, 0.1)
    end_point_circular = (-1, 0.2, 0.5)

    # Set the resolution for both paths
    resolution = 30

    # In your main function:
    circular_path_points = planner.generate_path(start_point_circular, end_point_circular, resolution, linear=False)
    
    target_list = []
    for i in range(0,len(circular_path_points), 1):
        pos = [circular_path_points[i][0], circular_path_points[i][1], circular_path_points[i][2]]
        point = (pos, [0,0,-1])
        target_list.append(point)

    # Now you can use target_list for animation
    controller.animate(target_list)

    # controller.animate(target_list)

if __name__ == "__main__":
    main()
    
