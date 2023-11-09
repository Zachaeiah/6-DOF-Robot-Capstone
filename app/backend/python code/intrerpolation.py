import numpy as np
from VelocityPFP import *
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pyquaternion import Quaternion

class PathPlanner:
    def __init__(self, maxTCPVel, maxDerivativeTCP):
        """
        Initializes the PathPlanner class.

        This class is used to generate and visualize 3D paths, both linear and circular.
        """
        self.saved_paths = []
        self.VelocityType = VelocityTypes(maxTCPVel, maxDerivativeTCP)  # Example values
    
    def setVelocityPFP(self, Vid: int):
        self.Vid = Vid
    
    
    def interpolate_angles(self, start_angle, end_angle):
        """
        Returns a list of angles interpolated between the start and end angles, taking the shortest path.

        Args:
            start_angle (float): The starting angle in degrees.
            end_angle (float): The ending angle in degrees.

        Returns:
            List[float]: A list of interpolated angles.
        """
        t_values = None
        if self.Vid == 0:
            t_values = self.VelocityType.linear()
        elif self.Vid == 1:
            t_values = self.VelocityType.sigmoid()
        elif self.Vid == 2:
            t_values = np.linspace(0, 1, self.VelocityType.max_value)
        else:
            raise ValueError("Invalid Vid value. Please set Vid to either 0 or 2.")
            

        diff = end_angle - start_angle
        print(diff)
        if diff >= 0:
            end_angle -= 360
        elif diff < 0:
            end_angle += 360

        # Ensure that the start and end angles are within the range [0, 360)
        start_angle = start_angle % 360
        end_angle = end_angle % 360

        step_size = diff / (len(t_values) - 1)
        return [(start_angle + i * (end_angle - start_angle)) for i in t_values]


    def points_on_linear_path_3d(self, start_point: tuple, end_point: tuple):
        """
        Generate points on a linear 3D path between two given points.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).
            resolution (int): The number of points to generate along the path.

        Returns:
            np.ndarray: An array of interpolated points along the linear path.
        """
        t_values = None
        if self.Vid == 0:
            t_values = self.VelocityType.linear()
        elif self.Vid == 1:
            t_values = self.VelocityType.sigmoid()
        elif self.Vid == 2:
            t_values = np.linspace(0, 1, self.VelocityType.max_value)
        else:
            raise ValueError("Invalid Vid value. Please set Vid to either 0 or 2.")

        
        start_point = np.array(start_point)
        end_point = np.array(end_point)
        interpolated_points = [tuple(start_point + t * (end_point - start_point)) for t in t_values]

        return np.array(interpolated_points)


    def points_on_circular_path_3d(self, start_point: tuple, end_point: tuple):
        """
        Generate points on a circular 3D path between two given points.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).

        Returns:
            np.ndarray: An array of interpolated points along the circular path.
        """
        start_point = np.array(start_point)
        end_point = np.array(end_point)

        init_radius = np.linalg.norm(start_point[:2])
        init_angle = np.arctan2(start_point[1], start_point[0])
        final_radius = np.linalg.norm(end_point[:2])
        final_angle = np.arctan2(end_point[1], end_point[0])

        angle_steps = self.interpolate_angles(np.degrees(init_angle), np.degrees(final_angle))

        radii = np.linspace(init_radius, final_radius, len(angle_steps))
        heights = np.linspace(start_point[2], end_point[2], len(angle_steps))

        angles = np.radians(angle_steps)
        cos_angles = np.cos(angles)
        sin_angles = np.sin(angles)

        points = np.column_stack((radii * cos_angles, radii * sin_angles, heights))

        return points

    def generate_path(self, start_point, end_point, linear: bool):
        """
        Generate a 3D path between two points and store it.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).
            resolution (int): The number of points to generate along the path.
            linear (bool): If True, generate a linear path; otherwise, generate a circular path.

        Returns:
            np.ndarray: An array of interpolated points along the path.
            str: A color used for plotting the path.
        """
        self.start_point = start_point
        self.end_point = end_point
        self.linear = linear

        if linear:
            self.points = self.points_on_linear_path_3d(self.start_point, self.end_point)
        else:
            self.points = self.points_on_circular_path_3d(self.start_point, self.end_point)


        # Store the generated path and its color
        self.saved_paths.append(self.points)

        return self.points

    def plot_3d_path(self, label_start=True, label_end=True):
        """
        Plot and visualize the 3D paths.

        Args:
            label_start (bool): If True, label the start point.
            label_end (bool): If True, label the end point.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Initialize variables to store min and max values for each axis
        x_min, x_max = float('inf'), float('-inf')
        y_min, y_max = float('inf'), float('-inf')
        z_min, z_max = float('inf'), float('-inf')

        for path in self.saved_paths:
            x_coords, y_coords, z_coords = path.T

            # Customize marker size and transparency
            ax.scatter(x_coords, y_coords, z_coords, s=20, marker='o', alpha=0.5)

            # Update min and max values for each axis
            x_min = min(x_min, np.min(x_coords))
            x_max = max(x_max, np.max(x_coords))
            y_min = min(y_min, np.min(y_coords))
            y_max = max(y_max, np.max(y_coords))
            z_min = min(z_min, np.min(z_coords))
            z_max = max(z_max, np.max(z_coords))
        
        ax.scatter([0], [0], color="red")

        # Customize azimuth (horizontal viewing angle) and elevation (vertical viewing angle)
        ax.view_init(azim=-45, elev=20)

        # Add grid lines
        ax.grid(True)

        # Add titles and labels
        if self.linear:
            ax.set_title('3D linear path plan')
        else:
            ax.set_title('3D circular path plan')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Show the plot
        plt.show()

    def clear_saved_paths(self):
        """Clear the list of saved paths."""
        self.saved_paths = []

def main():
        
    # Create an instance of the PathPlanner class
    planner = PathPlanner(100, 5)
    planner.setVelocityPFP(2)

    # Define start and end points for the paths
    start_point_linear = (10, -5, -20)
    end_point_linear = (20, 20, 20)

    # Generate a linear path and store the points and color
    planner.generate_path(start_point_linear, end_point_linear, linear=False)

    # Generate a circular path and store the points and color
    #circular_path_points, circular_path_color = planner.generate_path(start_point_circular, end_point_circular, resolution, linear=False)

    # Plot the 3D paths
    planner.plot_3d_path(label_start=True, label_end=True)

    # Clear the saved paths (optional)
    planner.clear_saved_paths()

    # Show the plots
    plt.show()

if __name__ == "__main__":
    main()
