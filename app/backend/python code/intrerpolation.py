import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
from itertools import cycle

class PathPlanner:
    def __init__(self):
        """
        Initializes the PathPlanner class.

        This class is used to generate and visualize 3D paths, both linear and circular.
        """
        self.color_cycle = cycle(plt.cm.viridis(np.linspace(0, 1, 100)))  # Choose a colormap and number of colors
        self.saved_paths = []

    def slerp(self, quat1, quat2, t):
        """
        Spherical Linear Interpolation (SLERP) between two quaternions.

        Args:
            quat1 (Quaternion): The starting quaternion.
            quat2 (Quaternion): The ending quaternion.
            t (float): The interpolation parameter ranging from 0 to 1.

        Returns:
            Quaternion: The interpolated quaternion.
        """
        return Quaternion.slerp(quat1, quat2, t)

    def points_on_linear_path_3d(self, start_point: tuple, end_point: tuple, resolution: int):
        """
        Generate points on a linear 3D path between two given points.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).
            resolution (int): The number of points to generate along the path.

        Returns:
            np.ndarray: An array of interpolated points along the linear path.
        """
        if resolution < 2:
            raise ValueError("Resolution must be at least 2.")

        t_values = np.linspace(0, 1, resolution)
        start_point = np.array(start_point)
        end_point = np.array(end_point)
        interpolated_points = [tuple(start_point + t * (end_point - start_point)) for t in t_values]

        return np.array(interpolated_points)

    def points_on_circular_path_3d(self, start_point: tuple, end_point: tuple, resolution: int):
        """
        Generate points on a circular 3D path between two given points.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).
            resolution (int): The number of points to generate along the path.

        Returns:
            np.ndarray: An array of interpolated points along the circular path.
        """
        start_x, start_y, start_z = start_point
        end_x, end_y, end_z = end_point

        start_angle = np.arctan2(start_y, start_x)
        end_angle = np.arctan2(end_y, end_x)

        start_radius = np.linalg.norm(start_point[:2])
        end_radius = np.linalg.norm(end_point[:2])

        # Create quaternions to represent the rotations
        start_quaternion = Quaternion(axis=[0, 0, 1], radians=start_angle)
        end_quaternion = Quaternion(axis=[0, 0, 1], radians=end_angle)

        # Interpolate quaternions for rotation
        t_values = np.linspace(0, 1, resolution)
        interpolated_quaternions = [self.slerp(start_quaternion, end_quaternion, t) for t in t_values]

        # Interpolate radii and heights
        radii = np.linspace(start_radius, end_radius, resolution)
        heights = np.linspace(start_z, end_z, resolution)

        # Initialize arrays to store points
        points = np.zeros((resolution, 3))

        # Apply the rotations to the initial vector [start_radius, 0, 0]
        for i, quaternion in enumerate(interpolated_quaternions):
            rotated_vector = quaternion.rotate(np.array([radii[i], 0, 0]))
            points[i] = rotated_vector + np.array([0, 0, heights[i]])

        return points

    def generate_path(self, start_point, end_point, resolution, linear: bool):
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
        self.resolution = resolution
        self.linear = linear

        if linear:
            self.points = self.points_on_linear_path_3d(self.start_point, self.end_point, self.resolution)
        else:
            self.points = self.points_on_circular_path_3d(self.start_point, self.end_point, self.resolution)

        # Get the next color from the colormap cycle
        color = next(self.color_cycle)

        # Store the generated path and its color
        self.saved_paths.append((self.points, color))

        return self.points, color

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

        for path, color in self.saved_paths:
            x_coords, y_coords, z_coords = path.T

            # Customize marker size and transparency
            ax.scatter(x_coords, y_coords, z_coords, c=color, s=20, marker='o', alpha=0.5)

            # Update min and max values for each axis
            x_min = min(x_min, np.min(x_coords))
            x_max = max(x_max, np.max(x_coords))
            y_min = min(y_min, np.min(y_coords))
            y_max = max(y_max, np.max(y_coords))
            z_min = min(z_min, np.min(z_coords))
            z_max = max(z_max, np.max(z_coords))

        # Customize azimuth (horizontal viewing angle) and elevation (vertical viewing angle)
        ax.view_init(azim=-45, elev=20)

        # Add grid lines
        ax.grid(True)

        # Set axes limits to fit the largest path
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_zlim(z_min, z_max)

        # Add titles and labels
        if self.linear:
            ax.set_title('3D linear path plan')
        else:
            ax.set_title('3D circular path plan')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        if label_start:
            # Label for the start point
            ax.text(*self.saved_paths[0][0][0], 'Start', color='red', fontsize=12, ha='center')

        if label_end:
            # Label for the end point
            ax.text(*self.saved_paths[-1][0][-1], 'End', color='blue', fontsize=12, ha='center')

        # Show the plot
        plt.show()

    def clear_saved_paths(self):
        """Clear the list of saved paths."""
        self.saved_paths = []

def main():
        
    # Create an instance of the PathPlanner class
    planner = PathPlanner()

    # Define start and end points for the paths
    start_point_linear = (2, 2, 2)
    end_point_linear = (-1, -1, -1)

    start_point_circular = (-1, -1, -1)
    end_point_circular = (2, 2, 2)

    # Set the resolution for both paths
    resolution = 500

    # Generate a linear path and store the points and color
    linear_path_points, linear_path_color = planner.generate_path(start_point_linear, end_point_linear, resolution, linear=True)

    # Generate a circular path and store the points and color
    circular_path_points, circular_path_color = planner.generate_path(start_point_circular, end_point_circular, resolution, linear=False)

    # Plot the 3D paths
    planner.plot_3d_path(label_start=True, label_end=True)

    # Clear the saved paths (optional)
    planner.clear_saved_paths()

    # Show the plots
    plt.show()

if __name__ == "__main__":
    main()
