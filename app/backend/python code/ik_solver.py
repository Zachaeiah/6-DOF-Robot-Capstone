# Import necessary libraries
import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R
import random
import time

class RobotArm:
    def __init__(self, urdf_file_path: str = None, initial_position: list = None) -> None:
        """Initialize the RobotArm object.

        Args:
            urdf_file_path (str): Path to the URDF file.

        Raises:
            ValueError: If an error occurs during initialization.
        """
        try:
            # Create a robot chain from the URDF file
            self.my_chain = ikpy.chain.Chain.from_urdf_file(urdf_file_path, active_links_mask=[False, True, True, False, True, True, False, True, True])
            self.last_angles = None
            self.initial_position = initial_position
        except Exception as e:
            raise ValueError(f"Error initializing the robot arm: {e}")

    def calculate_ik(
        self,
        target_positions: list,
        target_orientations: list = [np.eye(3)],
        orientation_modes: list = ["all"],
        batch_size: int = 1
    ) -> iter:
        """Perform inverse kinematics calculations.

        Args:
            target_positions (list): List of target positions.
            target_orientations (list): List of target orientations.
            orientation_modes (list): List of orientation modes.
            batch_size (int, optional): Batch size for calculations. Defaults to 1.

        Yields:
            list: List of inverse kinematics angles or an empty list if an error occurs.
        """
        for i in range(0, len(target_positions), batch_size):
            positions_batch = target_positions[i:i + batch_size]
            orientations_batch = target_orientations[i:i + batch_size]
            orientation_batch = orientation_modes[i:i + batch_size]
            for target_position, target_orientation, orientation_mode in zip(positions_batch, orientations_batch, orientation_batch):
                try:
                    # Use last angles if available
                    if self.last_angles is not None:
                        ik = self.my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode=orientation_mode, initial_position=self.last_angles)
                    else:
                        ik = self.my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode=orientation_mode)
                    fk = self.my_chain.forward_kinematics(ik)
                    achieved_position = fk[:3, 3]
                    achieved_orientation = fk[:3, :3]

                    # Calculate error between target and achieved position/orientation
                    position_error = target_position - achieved_position
                    orientation_error = target_orientation - achieved_orientation


                    self.last_angles = ik
                    yield (ik, position_error, orientation_error)
                except Exception as e:
                    print(f"An unexpected error occurred during inverse kinematics calculation: {e}")
                    yield []  # Return an empty list if an error occurs

    def calculate_fk(
        self,
        joint_angles: list[list[0, float, float, 0, float, float, 0, float, float]],
        batch_size: int = 5
    ) -> iter:
        """Calculate forward kinematics for a given set of joint angles.

        Args:
            joint_angles (List[List[float]]): List of joint angles. Each inner list represents a set of joint angles.
            batch_size (int, optional): Size of the batch for processing joint angles. Defaults to 5.

        Returns:
            Iterator: An iterator yielding positions, orientation, and homogeneous transformation matrices for each set of joint angles.

        Yields:
            Iterator: Tuple containing positions, orientation, and homogeneous transformation matrices for each set of joint angles.
        """
        for i in range(0, len(joint_angles), batch_size):
            joint_batch = joint_angles[i:i + batch_size]
            for joints in joint_batch:
                try:

                    #stor the last angle
                    self.last_angles = joints

                    # Calculate forward kinematics
                    end_effector = self.my_chain.forward_kinematics(joints)
                    orientation = end_effector[:3, :3]
                    positions = end_effector[:3, 3]

                    yield positions, orientation, end_effector  # Include all three values in the yield statement

                except Exception as e:
                    print(f"An unexpected error occurred during forward kinematics calculation: {e}")
                    yield np.array([]), np.array([]), np.array([])  # Return empty arrays if an error occurs

    def animate_fk(
    self,
    joint_angles_trajectory: list,
    interval: float = 0.1,
    ax=None,
    fig=None
    ) -> None:
        """Animate the forward kinematics of the robot arm using ikpy plotting.

        Args:
            joint_angles_trajectory (list): List of joint angles over time.
            interval (float, optional): Time interval between updates in seconds. Defaults to 0.1.
            ax (matplotlib.axes._subplots.Axes3DSubplot, optional): Axes to use for plotting. Defaults to None.
            fig (matplotlib.figure.Figure, optional): Figure to use for plotting. Defaults to None.
        """
        if ax is None and fig is None:
            fig, ax = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(12, 8))


        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(-1, 1)
        ax.set_zlim3d(0, 2)

        # Plot the robot arm once for initialization
        self.my_chain.plot(joint_angles_trajectory[0], ax, target=self.my_chain.forward_kinematics(joint_angles_trajectory[0]))

        def update(frame):
            ax.clear()  # Clear the previous frame
            joint_angles = joint_angles_trajectory[frame]

            # Plot the robot arm with the updated joint angles
            target = self.my_chain.forward_kinematics(joint_angles)
            self.my_chain.plot(joint_angles, ax, target = target)
            self.last_angles = joint_angles

            # Add arrows at the origin for the axes
            ax.quiver(0, 0, 0, 1, 0, 0, color=(0.0 , 0.5 , 0.0), arrow_length_ratio=0.05)
            ax.quiver(0, 0, 0, 0, 1, 0, color=(0.36, 0.96, 0.96), arrow_length_ratio=0.05)
            ax.quiver(0, 0, 0, 0, 0, 1, color=(1.0 , 0.64, 0.0), arrow_length_ratio=0.05)


            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            plt.title(f'Frame {frame}\n{target[:3, 3]}')

        ani = animation.FuncAnimation(fig, update, frames=len(joint_angles_trajectory), interval=interval, repeat=False)
        plt.show()
    
    def animate_ik(
        self,
        target_positions: list,
        target_orientations: list,
        orientation_modes: list,
        interval: int = 1,
        save_as_gif: bool = False,
        file_name: str = "robot_animation.gif",
        ax=None,
        fig=None
    ) -> None:
        """_summary_

        Args:
            target_positions (list): _description_
            target_orientations (list): _description_
            orientation_modes (list): _description_
            interval (int, optional): _description_. Defaults to 1.
            save_as_gif (bool, optional): _description_. Defaults to False.
            file_name (str, optional): _description_. Defaults to "robot_animation.gif".
            ax (matplotlib.axes._subplots.Axes3DSubplot, optional): Axes to use for plotting. Defaults to None.
            fig (matplotlib.figure.Figure, optional): Figure to use for plotting. Defaults to None.
        """
        if ax is None and fig is None:
            fig, ax = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(12, 8))

        # Initialize a line for the trailing plot
        line, = ax.plot([], [], [], color='red', marker='o')

        def update(frame: int) -> None:
            """Update the animation frame.

            Args:
                frame (int): Current frame.
            """
            ax.clear()

            trailing_x = [tup[0] for tup in target_positions[frame-15:frame]]
            trailing_y = [tup[1] for tup in target_positions[frame-15:frame]]
            trailing_z = [tup[2] for tup in target_positions[frame-15:frame]]
            ax.scatter(trailing_x, trailing_y, trailing_z, color="red", s=10, marker='o', alpha=0.1)

            target_position = target_positions[frame]
            target_orientation = target_orientations[frame]
            orientation_mode = orientation_modes[frame]
            
            ik_solution = list(self.calculate_ik([target_position], [target_orientation], [orientation_mode], batch_size=1))[0]
            
            self.my_chain.plot(ik_solution[0], ax, target=np.array(target_position, dtype=np.float32))
            self.last_angles = ik_solution[0]

            # Add arrows at the origin for the axes
            ax.quiver(0, 0, 0, 1, 0, 0, color=(0.0 , 0.5 , 0.0), arrow_length_ratio=0.05)
            ax.quiver(0, 0, 0, 0, 1, 0, color=(0.36, 0.96, 0.96), arrow_length_ratio=0.05)
            ax.quiver(0, 0, 0, 0, 0, 1, color=(1.0 , 0.64, 0.0), arrow_length_ratio=0.05)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)

        anim = animation.FuncAnimation(fig, update, frames=len(target_positions), interval=interval, repeat=False)

        if save_as_gif:
            anim.save(file_name, writer='pillow')
        plt.show()

def rotate_x(matrix, angle_x):
    """_summary_

    Args:
        matrix (_type_): _description_
        angle_x (_type_): _description_

    Returns:
        _type_: _description_
    """
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x), -np.sin(angle_x)],
        [0, np.sin(angle_x), np.cos(angle_x)]
    ])
    rotated_matrix = np.dot(rotation_matrix_x, matrix)
    return rotated_matrix

def rotate_y(matrix, angle_y):
    """_summary_

    Args:
        matrix (_type_): _description_
        angle_y (_type_): _description_

    Returns:
        _type_: _description_
    """
    rotation_matrix_y = np.array([
        [np.cos(angle_y), 0, np.sin(angle_y)],
        [0, 1, 0],
        [-np.sin(angle_y), 0, np.cos(angle_y)]
    ])
    rotated_matrix = np.dot(rotation_matrix_y, matrix)
    return rotated_matrix

def rotate_z(matrix, angle_z):
    """_summary_

    Args:
        matrix (_type_): _description_
        angle_z (_type_): _description_

    Returns:
        _type_: _description_
    """
    rotation_matrix_z = np.array([
        [np.cos(angle_z), -np.sin(angle_z), 0],
        [np.sin(angle_z), np.cos(angle_z), 0],
        [0, 0, 1]
    ])
    rotated_matrix = np.dot(rotation_matrix_z, matrix)
    return rotated_matrix

def main():
    urdf_file_path = "app\\backend\\python code\\urdf_tes2.urdf"
    initial_position = np.array([0, 0.0, np.pi/2, 0, np.pi/2, -np.pi/2, 0, -np.pi/2, 0])
    robot = RobotArm(urdf_file_path, initial_position)
    num_positions = 1


    # # robot.animate_fk([[0, 0, 138.6653286, 17.4672121, 0, -14.51805191, -8.8323662, 0, 0]]) 
    robot.animate_fk([[0, 0.0, np.pi/2, 0, np.pi/2, -np.pi/2, 0, -np.pi/2, 0]]) 
    num_samples = 1


    # target_positions = [[-0.0729999, 0.45, (-0.3+0.0508)] for _ in range(num_samples)]
    # target_orientations = [rotate_x(rotate_y(np.eye(3),-np.pi/2),np.pi/2) for _ in range(num_samples)]
    # orientation_modes = ['all' for _ in range(num_samples)]

    # robot.animate_ik(target_positions, target_orientations, orientation_modes)
       
    # IK = robot.calculate_ik(target_positions, target_orientations, orientation_modes)

    # for ik in IK:
    #     print(ik)

    
        
    
    

if __name__ == "__main__":
    main()

    # Function for stress testing
def stress_test():
    urdf_file_path = "urdf_tes1.urdf"
    robot = RobotArm(urdf_file_path)

    # Adjust the number of samples as needed
    num_samples = 100

    # Generate random target positions and orientations for stress testing
    target_positions = [[random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)] for _ in range(num_samples)]
    target_orientations = [[random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi)] for _ in range(num_samples)]

    # Perform stress testing and print the results
    start_time = time.time()
    ik_generator = robot.calculate_ik(target_positions, target_orientations, precision=5, batch_size=10)
    for ik in ik_generator:
        print("The angles of each joint are:", ik)
    end_time = time.time()

    elapsed_time = end_time - start_time
    print(f"Stress test completed in {elapsed_time:.5f} seconds.")

    # Free the memory used by the generator
    ik_generator = None

def run_stress_test_with_batch_range(batch_size_range):
    urdf_file_path = "urdf_tes1.urdf"
    robot = RobotArm(urdf_file_path)

    # Adjust the number of samples as needed
    num_samples = 1000

    # Generate random target positions and orientations for stress testing
    target_positions = [[random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)] for _ in range(num_samples)]
    target_orientations = [[random.uniform(-2*np.pi, 2*np.pi), random.uniform(-2*np.pi, 2*np.pi), random.uniform(-2*np.pi, 2*np.pi)] for _ in range(num_samples)]

    # Store the times for each batch size
    elapsed_times = []
    average_time_per_samples = []

    # Iterate through the range of batch sizes and perform stress testing for each batch size
    for batch_size in batch_size_range:
        start_time = time.time()
        ik_generator = robot.calculate_ik(target_positions, target_orientations, precision=5, batch_size=batch_size)
        progress_percent = 0
        print(f"Stress test with batch size {batch_size}:", end=" ")

        for i, ik in enumerate(ik_generator, 1):
            if i * 10 / num_samples > progress_percent:
                print(".", end="", flush=True)
                progress_percent += 1

        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"\nTest completed in {elapsed_time:.5f} seconds.")

        # Append the elapsed time to the times list
        elapsed_times.append(elapsed_time)

        # Calculate and print the average time per sample for the batch
        average_time_per_sample = elapsed_time / num_samples
        print(f"Average time per sample: {average_time_per_sample:.5f} seconds")

        # Append the elapsed time to the times list
        average_time_per_samples.append(average_time_per_sample)

        # Free the memory used by the generator
        ik_generator = None

    fig, ax1 = plt.subplots(figsize=(10, 6))

    # Plot the times for each batch size
    ax1.plot(batch_size_range, elapsed_times, color='tab:blue', label='Elapsed Time')
    ax1.set_xlabel('Batch Size')
    ax1.set_ylabel('Elapsed Time (seconds)', color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')

    # Create a second y-axis for the scatter plot
    ax2 = ax1.twinx()
    ax2.scatter(batch_size_range, average_time_per_samples, color='tab:red', label='Average Time per Sample')
    ax2.set_ylabel('Average Time per Sample (seconds)', color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')

    plt.title('Stress Test Batch Size vs. Elapsed Time and Average Time per Sample')
    fig.legend(loc="upper right")
    plt.grid(True)
    plt.show()
