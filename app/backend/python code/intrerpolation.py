import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import cycle

class PathPlanner:
    def __init__(self):
        self.color_cycle = cycle(plt.cm.viridis(np.linspace(0, 1, 100)))  # Choose a colormap and number of colors
        self.saved_paths = []
    
    @staticmethod
    def points_on_linear_path_3d(start_point: tuple, end_point: tuple, resolution: int):
        if resolution < 2:
            raise ValueError("Resolution must be at least 2.")

        t_values = np.linspace(0, 1, resolution)
        start_point = np.array(start_point)
        end_point = np.array(end_point)
        interpolated_points = [tuple(start_point + t * (end_point - start_point)) for t in t_values]

        return np.array(interpolated_points)

    @staticmethod
    def points_on_circular_path_3d(start_point: tuple, end_point: tuple, resolution: int):
        start_x, start_y, start_z = start_point
        end_x, end_y, end_z = end_point

        start_angle = np.degrees(np.arctan2(start_y, start_x))
        end_angle = np.degrees(np.arctan2(end_y, end_x))

        start_radius = np.linalg.norm(start_point[:2])
        end_radius = np.linalg.norm(end_point[:2])

        if start_angle < 0:
            start_angle += 360
        if end_angle < 0:
            end_angle += 360

        angles = np.linspace(start_angle, end_angle, resolution)

        radii = np.linspace(start_radius, end_radius, resolution)
        heights = np.linspace(start_z, end_z, resolution)

        # Precompute trigonometric values
        angle_radians = np.radians(angles)
        cos_angles = np.cos(angle_radians)
        sin_angles = np.sin(angle_radians)

        # Create arrays to store points
        points = np.zeros((resolution, 3))

        # Fill the points array with precomputed values
        points[:, 0] = radii * cos_angles
        points[:, 1] = radii * sin_angles
        points[:, 2] = heights

        return points

    def generate_path(self, start_point, end_point, resolution, linear: bool):
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
        if  self.linear:
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
        self.saved_paths = []

def generate_points_pattern(num_iterations):
    points_pattern = []
    
    for i in range(1, num_iterations + 1):
        start_point = tuple([i] * 3)
        end_point = tuple([-i] * 3)
        points_pattern.append([start_point, end_point])
    
    return points_pattern

def main():
    #Create an instance of the PathPlanner class
    planner = PathPlanner()

    # Generate and store the first path
    start_point1 = (1, 1, 1)
    end_point1 = (-1, -1, -1)
    resolution = 50
    planner.generate_path(start_point1, end_point1, resolution, True)

    # Generate and store the second path
    start_point2 = (-1, -1, -1)
    end_point2 = (2, 2, 2)
    planner.generate_path(start_point2, end_point2, resolution, False)

    # Plot all saved paths
    planner.plot_3d_path()

    # To clear saved paths:
    planner.clear_saved_paths()

if __name__ == '__main__':
    main()



