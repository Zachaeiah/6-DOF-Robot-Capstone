import numpy as np
import math
import matplotlib.pyplot as plt
from typing import List, Tuple


class MotionProfileGenerator:
    """
    MotionProfileGenerator class for generating motion profiles.

    Attributes:
        max_acc (int): Maximum acceleration.
        sys_max_vel (int): System maximum velocity.
        MotionProfilename (str): Motion profile name.
        is_moving (bool): Flag indicating whether the motion profile is valid.
        move_time (float): Total time for the motion profile.
        displacement (float): Displacement for the motion profile.
        max_vel (float): Maximum velocity for the motion profile.
        time_values (List[float]): List of time values.
        accelerations (List[float]): List of accelerations.
        velocity (List[float]): List of velocities.
    """

    def __init__(self, max_acc: int = 50, sys_max_vel: int = 50, MotionProfilename: str = ""):
        """
        Initialize the MotionProfileGenerator.

        Args:
            max_acc (int, optional): Maximum acceleration. Defaults to 50.
            sys_max_vel (int, optional): System maximum velocity. Defaults to 50.
            MotionProfilename (str, optional): Motion profile name. Defaults to "".
        """
        self.is_moving: bool = True
        self.max_acc: int = max_acc
        self.sys_max_vel: int = sys_max_vel
        self.move_time: float = 0
        self.displacement: float = 0
        self.MotionProfilename: str = MotionProfilename
        self.max_vel: float = 0
        self.time_values: List[float] = []
        self.accelerations: List[float] = []
        self.velocity: List[float] = []


    def Set_move_time(self, move_time: float) -> None:
        """
        Set the total time for the motion profile.

        Args:
            move_time (float): Total time for the motion profile.
        """
        if move_time <= 0:
            self.is_moving = False
        else:
            self.move_time = move_time
            try:
                under_sqrt: float = (self.max_acc ** 2) * (move_time ** 2) - (6 * self.max_acc * self.displacement)
                noom: float = self.max_acc * move_time - math.sqrt(under_sqrt)
                self.max_vel = min(noom / 3, self.sys_max_vel)
            except ValueError as ve:
                print(f"ValueError in {self.__repr__()} - Set_move_time: {ve}")
                # Handle the ValueError as needed or re-raise it
                raise
            except Exception as e:
                print(f"An unexpected error occurred in {self.__repr__()} - Set_move_time: {e}")
                # Handle the unexpected error as needed or re-raise it
                raise
    

    def Set_displacement(self, displacement: float) -> bool:
        """
        Set the displacement for the motion profile.

        Args:
            displacement (float): Displacement for the motion profile.

        Returns:
            bool: True if the displacement is valid and motion is possible, False otherwise.
        """
        if displacement <= 0:
            self.is_moving = False
        else:
            self.displacement = displacement
            self.max_vel = self.calculate_max_velocity()
            self.move_time = self.calculate_move_time()


    def Generator_profile(self, time_values: List[float] = [None]) -> None:
        """
        Generate motion profiles for given time values.

        Args:
            time_values (List[float]): List of time values.
        """

        self.time_values = time_values
        # self.accelerations = self.generate_accelerations_profile()
        # self.velocity = self.generate_velocity_profile()
        self.displacement = self.generate_displacement_profile()


    def calculate_max_velocity(self) -> float:
        """
        Calculate the maximum velocity.

        Returns:
            float: Maximum velocity.
        """
        if not self.is_moving:
            return 0
        return min(self.sys_max_vel, math.sqrt(6 * self.max_acc * self.displacement) / 3)


    def calculate_move_time(self) -> float:
        """
        Calculate the total time required for the motion.

        Returns:
            float: Total time.
        """
        if not self.is_moving:
            return 0
        try:
            return ((3 * self.max_vel) / (2 * self.max_acc)) + (self.displacement / self.max_vel)
        except Exception as e:
            print(f"An unexpected error occurred in {self.__repr__()} - calculate_move_time: {e}")
            # Handle the unexpected error as needed or re-raise it
            raise

   
    def generate_accelerations_profile(self) -> List[float]:
        """
        Generate acceleration profile over time.

        Returns:
            List[float]: List of accelerations.
        """
        if not self.is_moving:
            return np.zeros(len(self.time_values))
        
        coff1: float = (16 * self.max_acc ** 3) / (9 * self.max_vel ** 2)
        coff2: float = (8 * self.max_acc ** 2) / (3 * self.max_vel)

        T_1: float = (3 * self.max_vel) / (2 * self.max_acc)
        T_2: float = self.move_time - T_1

        accelerations: List[float] = np.zeros(len(self.time_values))
        
        for t, x in enumerate(self.time_values):
            if 0 <= x < T_1:
                accelerations[t] = -coff1 * x ** 2 + coff2 * x
            elif T_1 <= x < T_2:
                accelerations[t] = 0
            elif T_2 <= x <= self.move_time:
                accelerations[t] = coff1 * (x - T_2) ** 2 - coff2 * (x - T_2)

        return accelerations


    def generate_velocity_profile(self) -> List[float]:
        """
        Generate velocity profile over time.

        Returns:
            List[float]: List of velocities.
        """
        if not self.is_moving:
            return np.zeros(len(self.time_values))
        
        coff1: float = (16 * self.max_acc ** 3) / (27 * self.max_vel ** 2)
        coff2: float = (4 * self.max_acc ** 2) / (3 * self.max_vel)

        T_1: float = (3 * self.max_vel) / (2 * self.max_acc)
        T_2: float = self.move_time - T_1

        velocities: List[float] = np.zeros(len(self.time_values))

        for t, x in enumerate(self.time_values):
            if 0 <= x < T_1:
                velocities[t] = -coff1 * x ** 3 + coff2 * x ** 2
            elif T_1 <= x < T_2:
                velocities[t] = self.max_vel
            elif T_2 <= x <= self.move_time:
                velocities[t] = coff1 * (x - T_2) ** 3 - coff2 * (x - T_2) ** 2 + self.max_vel

        return velocities

 
    def generate_displacement_profile(self) -> np.ndarray:
        """
        Generate displacement profile over time.

        Returns:
            np.ndarray: Array of displacements.
        """
        if not self.is_moving:
            return np.zeros_like(self.time_values)

        coff1 = (4 * self.max_acc ** 3) / (27 * self.max_vel ** 2)
        coff2 = (4 * self.max_acc ** 2) / (9 * self.max_vel)

        T_1 = (3 * self.max_vel) / (2 * self.max_acc)
        T_2 = self.move_time - T_1

        displacements = np.zeros_like(self.time_values)

        # Create boolean masks for each condition
        mask1 = (0 <= self.time_values) & (self.time_values < T_1)
        mask2 = (T_1 <= self.time_values) & (self.time_values < T_2)
        mask3 = (T_2 <= self.time_values) & (self.time_values <= self.move_time)

        # Apply vectorized calculations using masks
        displacements[mask1] = -coff1 * self.time_values[mask1] ** 4 + coff2 * self.time_values[mask1] ** 3
        displacements[mask2] = self.max_vel * self.time_values[mask2] - (3 * self.max_vel ** 2) / (4 * self.max_acc)
        displacements[mask3] = (
            coff1 * (self.time_values[mask3] - T_2) ** 4 - coff2 * (self.time_values[mask3] - T_2) ** 3 +
            self.max_vel * self.time_values[mask3] - (3 * self.max_vel ** 2) / (4 * self.max_acc)
        )

        return displacements
    

    def __repr__(self) -> str:
        return f"{self.MotionProfilename}"


def calculate_polar_coordinates(angle, magnitude):
    """
    Calculate Cartesian coordinates from polar coordinates.

    Args:
        angle (float): Angle in degrees.
        magnitude (float): Magnitude of the vector.

    Returns:
        tuple: Cartesian coordinates (x, y).
    """
    x = magnitude * np.cos(np.radians(angle))
    y = magnitude * np.sin(np.radians(angle))
    return x, y

def plot_profiles(time_values, accelerations, velocities, displacements):
    """
    Plot the motion profiles over time.

    Args:
        time_values (numpy.ndarray): Array of time values.
        accelerations (numpy.ndarray): Array of accelerations.
        velocities (numpy.ndarray): Array of velocities.
        displacements (numpy.ndarray): Array of displacements.
    """
    plt.plot(time_values, accelerations, color="purple", label="Acceleration")
    plt.plot(time_values, velocities, color="black", label="Velocity")
    plt.plot(time_values, displacements, color="red", label="Displacement")
    plt.title('Motion Profiles over Time')
    plt.xlabel("Time")
    plt.ylabel("Profiles")
    plt.grid(True)
    plt.legend()
    plt.show()

def radius_function(angle, t0, t1, r0, r1):
    """Calculate the radius at a given angle using linear interpolation."""
    angle_rad = np.radians(angle)
    return r0 + (r1 - r0) * (angle_rad - np.radians(t0)) / (np.radians(t1) - np.radians(t0))

def integrand(angle, t0, t1, r0, r1):
    """Calculate the integrand for arc length calculation."""
    dangle_rad = np.radians(t1 - t0)
    dr_dangle = (r1 - r0) / dangle_rad
    r_angle = radius_function(angle, t0, t1, r0, r1)
    return np.sqrt(r_angle**2 + dr_dangle**2)

def calculate_arc_length(angle_values, t0, t1, r0, r1):
    """Calculate the arc length of a curve using numerical integration."""
    dangle = np.radians(angle_values[1] - angle_values[0])
    integrand_values = integrand(angle_values, t0, t1, r0, r1)
    arc_length = np.trapz(integrand_values, dx=dangle)
    return arc_length

def calculate_initial_final_angle_magnitude(start, end):
    """Calculate initial and final angles and magnitudes."""
    angle_init = math.degrees(math.atan2(start[1], start[0]))
    angle_final = math.degrees(math.atan2(end[1], end[0]))

    magnitude_init = math.sqrt(start[1]**2 + start[0]**2)
    magnitude_final = math.sqrt(end[1]**2 + end[0]**2)

    return angle_init, angle_final, magnitude_init, magnitude_final

def calculate_trajectory_values(start, end, max_acc, max_vel):
    """Calculate trajectory values and profiles with error handling."""
    try:
        # Calculate initial and final angles and magnitudes
        angle_init, angle_final, magnitude_init, magnitude_final = calculate_initial_final_angle_magnitude(start, end)

        # Initialize motion profile generators
        angle_profile = MotionProfileGenerator(max_acc, max_vel, "angle_profile")
        magnitude_profile = MotionProfileGenerator(max_acc, max_vel,"magnitude_profile")
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
        #magnitudes_values = magnitude_init + magnitude_profile.displacement if magnitude_final - magnitude_init > 0 else magnitude_init - magnitude_profile.displacement
        height_values = start[2] + height_profile.displacement if end[-1] - start[-1] > 0 else start[2] - height_profile.displacement

        magnitudes_values = np.linspace(magnitude_init, magnitude_final, len(angle_values))
        return angle_values, magnitudes_values, height_values

    except ValueError as ve:
        print(f"ValueError during trajectory calculation: {ve}")
        # You can handle this error further or raise it to the calling code.
        raise ve
    except Exception as e:
        print(f"An unexpected error occurred during trajectory calculation: {e}")
        # You can handle this error further or raise it to the calling code.
        raise e

def plot_trajectory_3d(start, end, angle_values, magnitudes_values, height_values):
    """Plot the 3D trajectory."""
    # Calculate polar coordinates
    X, Y = calculate_polar_coordinates(angle_values, magnitudes_values)

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory, goals, and origin
    ax.scatter(X, Y, height_values, label="Trajectory")
    ax.scatter([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], label="Goals")
    ax.scatter([0], [0], label="Origin")
    
    # Customize plot
    ax.grid(True)
    ax.legend()
    plt.show()

def main():
    # Example values
    start = (5, 5, -5)
    end = (-2, -2, 5)
    max_acc = 25
    max_vel = 50

    # Calculate trajectory values
    angle_values, magnitudes_values, height_values = calculate_trajectory_values(start, end, max_acc, max_vel)

    # Plot the 3D trajectory
    plot_trajectory_3d(start, end, angle_values, magnitudes_values, height_values)

if __name__ == "__main__":
    main()
