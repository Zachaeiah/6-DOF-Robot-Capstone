import numpy as np
from VelocityPFP import *
import timeit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pyquaternion import Quaternion

class PathPlanner:
    def __init__(self, max_acc: float, sys_max_vel: float) -> None:
        """Initialize the PathPlanner.

        Args:
            max_acc (float): Maximum acceleration for the motion profile.
            sys_max_vel (float): System's maximum velocity.

        Attributes:
            linear (bool): Indicates whether the motion is linear.
            saved_paths (list): A list to store saved paths.
            max_acc (float): Maximum acceleration for the motion profile.
            sys_max_vel (float): System's maximum velocity.
            linearVelocityProfile (MotionProfileGenerator): Instance of MotionProfileGenerator
                responsible for generating linear velocity profiles.
        """

        self.linear = True
        self.saved_paths = []
        self.max_acc = max_acc
        self.sys_max_vel = sys_max_vel
        self.linearVelocityProfile = MotionProfileGenerator(max_acc, sys_max_vel, "linearVelocityProfile")

    def XY_angle(self, vector1: np.array, vector2: np.array) -> float:
        """Calculate the XY angle between two vectors.

        Args:
            vector1 (np.array): First vector represented as a NumPy array.
            vector2 (np.array): Second vector represented as a NumPy array.

        Returns:
            float: Angle in radians between the XY components of the two vectors.
        """
        # Calculate the dot product
        dot_product = np.dot(vector1[:-1], vector2[:-1])

        # Calculate the magnitudes of the vectors
        magnitude_vector1 = np.linalg.norm(vector1[:-1])
        magnitude_vector2 = np.linalg.norm(vector2[:-1])

        # Calculate the angle in radians
        angle_radians = np.arccos(dot_product / (magnitude_vector1 * magnitude_vector2))

        # Calculate the cross product to determine orientation
        cross_product = np.cross(vector1[:-1], vector2[:-1])

        # Check the z-component of the cross product
        if cross_product > 0:
            # Vector2 is counterclockwise (CCW) with respect to vector1
            return angle_radians
        else:
            # Vector2 is clockwise (CW) with respect to vector1
            return -angle_radians


    def points_on_linear_path_3d(self, start_point: tuple, end_point: tuple) -> list:
        """Generate points on a linear 3D path between two given points.

        Args:
            start_point (tuple): Starting point in 3D space.
            end_point (tuple): Ending point in 3D space.

        Returns:
            list: List of 3D points representing the linear path.
        """
        
        start_point = np.array(start_point)
        end_point = np.array(end_point)

        self.linearVelocityProfile.Set_displacement(np.linalg.norm(end_point - start_point))
        sys_time = self.linearVelocityProfile.move_time

        # Generate time values for motion profile
        time_values = np.arange(0, sys_time, 0.01)
        self.linearVelocityProfile.Generator_profile(time_values)
        t_values = self.linearVelocityProfile.displacement

        t_values = np.interp(t_values, (t_values.min(), t_values.max()), (0, 1))

        interpolated_points = start_point + np.outer(t_values, end_point - start_point)

        return interpolated_points   
        

    def calculate_initial_final_angle_magnitude(self, start: np.array, end: np.array) -> tuple:
        """Calculate initial and final angles and magnitudes between two vectors.

        Args:
            start (np.array): Initial vector represented as a NumPy array.
            end (np.array): Final vector represented as a NumPy array.

        Returns:
            tuple: A tuple containing initial angle, final angle, initial magnitude, and final magnitude.
        """
        
        angle_init = np.degrees(np.arctan2(start[1], start[0]))
        angle_final = angle_init + np.degrees(self.XY_angle(start, end))

        magnitude_init = np.linalg.norm(start[:2])
        magnitude_final = np.linalg.norm(end[:2])

        return angle_init, angle_final, magnitude_init, magnitude_final


    def calculate_trajectory_circular_path_3d(self, start: np.array, end: np.array, max_acc: float, max_vel: float) -> tuple:
        """Calculate the trajectory for a circular 3D path between two points.

        Args:
            start (np.array): Starting position represented as a NumPy array.
            end (np.array): Ending position represented as a NumPy array.
            max_acc (float): Maximum acceleration for the motion profiles.
            max_vel (float): Maximum velocity for the motion profiles.

        Raises:
            ValueError: Raised for invalid input values.
            Exception: Raised for unexpected errors during trajectory calculation.

        Returns:
            tuple: A tuple containing the calculated values for angle, magnitudes, and height.
        """
        
        try:
            angle_init, angle_final, magnitude_init, magnitude_final = self.calculate_initial_final_angle_magnitude(start, end)

            angle_profile = MotionProfileGenerator(max_acc, max_vel, "angle_profile")
            magnitude_profile = MotionProfileGenerator(max_acc, max_vel, "magnitude_profile")
            height_profile = MotionProfileGenerator(max_acc, max_vel, "height_profile")

            # Set displacements for each profile
            height_profile.Set_displacement(abs(end[-1] - start[-1]))
            magnitude_profile.Set_displacement(abs(magnitude_final - magnitude_init))
            angle_profile.Set_displacement(abs(angle_final - angle_init))

            # Calculate system time based on the maximum time among profiles
            sys_time = max(angle_profile.move_time, magnitude_profile.move_time, height_profile.move_time)

            # Set move time for each profile
            height_profile.Set_move_time(sys_time)
            magnitude_profile.Set_move_time(sys_time)
            angle_profile.Set_move_time(sys_time)

            # Generate time values for motion profiles
            time_values = np.arange(0, sys_time, 0.1)

            # Generate motion profiles for each dimension
            height_profile.Generator_profile(time_values)
            magnitude_profile.Generator_profile(time_values)
            angle_profile.Generator_profile(time_values)

             # Calculate final values for each dimension
            angle_values = angle_init + angle_profile.displacement if angle_final - angle_init > 0 else angle_init - angle_profile.displacement
            magnitudes_values = magnitude_init + magnitude_profile.displacement if magnitude_final - magnitude_init > 0 else magnitude_init - magnitude_profile.displacement
            height_values = start[2] + height_profile.displacement if end[-1] - start[-1] > 0 else start[2] - height_profile.displacement

            return angle_values, magnitudes_values, height_values

        except ValueError as ve:
            print(f"ValueError during trajectory calculation: {ve}")
            # You can handle this error further or raise it to the calling code.
            raise ve
        except Exception as e:
            print(f"An unexpected error occurred during trajectory calculation: {e}")
            # You can handle this error further or raise it to the calling code.
            raise e


    def points_on_circular_path_3d(self, start_point: tuple, end_point: tuple)-> np.ndarray:
        """
        Generate points on a circular 3D path between two given points.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).

        Returns:
            np.ndarray: An array of interpolated points along the circular path.
        """
        angle_values, magnitudes_values, z_coordinates = self.calculate_trajectory_circular_path_3d(start_point, end_point, self.max_acc, self.sys_max_vel)

        # Calculate polar coordinates
        x_coordinates = magnitudes_values * np.cos(np.radians(angle_values))
        y_coordinates = magnitudes_values * np.sin(np.radians(angle_values))

        return np.column_stack((x_coordinates, y_coordinates, z_coordinates))


    def generate_path(self, start_point: np.array, end_point: np.array, linear: bool) -> np.ndarray:
        """
        Generate a 3D path between two points and store it.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).
            linear (bool): If True, generate a linear path; otherwise, generate a circular path.

        Returns:
            np.ndarray: An array of interpolated points along the path.
        """
        self.start_point = start_point
        self.end_point = end_point

        if linear:
            self.points = self.points_on_linear_path_3d(start_point, end_point)
        else:
            self.points = self.points_on_circular_path_3d(start_point, end_point)

        self.saved_paths.append(self.points.copy())

        return self.points

    def generate_path(self, start_point: np.array, end_point: np.array, linear: bool) -> np.ndarray:
        """
        Generate a 3D path between two points and store it.

        Args:
            start_point (tuple): The starting point (x, y, z).
            end_point (tuple): The ending point (x, y, z).
            linear (bool): If True, generate a linear path; otherwise, generate a circular path.

        Returns:
            np.ndarray: An array of interpolated points along the path.
        """
        self.start_point = start_point
        self.end_point = end_point
        self.linear = linear

        if self.linear:
            self.points = self.points_on_linear_path_3d(self.start_point, self.end_point)
        else:
            self.points = self.points_on_circular_path_3d(self.start_point, self.end_point)

        # Convert points to a NumPy array before storing
        self.saved_paths.append(self.points.copy())

        return self.points


    
    def plot_3d_path(self) -> None:
        """
        Plot and visualize the 3D paths.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Initialize variables to store min and max values for each axis
        x_min, x_max = float('inf'), float('-inf')
        y_min, y_max = float('inf'), float('-inf')
        z_min, z_max = float('inf'), float('-inf')

        for path in self.saved_paths:
            x_coords, y_coords, z_coords = zip(*path)

            # Customize marker size and transparency
            ax.scatter(x_coords, y_coords, z_coords, s=20, marker='o', alpha=0.5)

            # Update min and max values for each axis
            x_min, x_max = min(x_min, np.min(x_coords)), max(x_max, np.max(x_coords))
            y_min, y_max = min(y_min, np.min(y_coords)), max(y_max, np.max(y_coords))
            z_min, z_max = min(z_min, np.min(z_coords)), max(z_max, np.max(z_coords))

        ax.scatter([0], [0], color="red")

        # Customize azimuth (horizontal viewing angle) and elevation (vertical viewing angle)
        ax.view_init(azim=-45, elev=20)

        # Add grid lines
        ax.grid(True)

        ax.set_title(f'3D path plan')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Show the plot
        plt.show()

    def clear_saved_paths(self)->None:
        """Clear the list of saved paths."""
        self.saved_paths = []

def main():
    # Initialize PathPlanner with maximum acceleration
    max_acc = 25
    max_vel = 50
    start =  (-0.51, -0.50, 0.0)
    end = (0.0, 0.7, 0.6)

    path_planner = PathPlanner(max_acc, max_vel)

    # Generate and plot a linear path
    path_planner.generate_path(start, end, linear=True)
    

    # Generate and plot a circular path
    path_planner.generate_path(start, end, linear=False)
    

    # Plot all saved paths
    path_planner.plot_3d_path()

if __name__ == "__main__":
    main()
