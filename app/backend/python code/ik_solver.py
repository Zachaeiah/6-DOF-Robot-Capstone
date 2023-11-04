# Import necessary libraries
import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import time

class RobotArm:
    def __init__(self, urdf_file_path):
        self.my_chain = ikpy.chain.Chain.from_urdf_file(urdf_file_path, active_links_mask=[False, True, True, False, True, True, False, True, True])
        self.last_angles = None  # Store the last calculated angles

    def calculate_ik(self, target_positions, target_orientations, precision=3, batch_size=1):
        for i in range(0, len(target_positions), batch_size):
            positions_batch = target_positions[i:i + batch_size]
            orientations_batch = target_orientations[i:i + batch_size]
            for target_position, target_orientation in zip(positions_batch, orientations_batch):
                target_orientation = np.array(target_orientation, dtype=np.float32).reshape(3,)
                # Use last angles if available
                if self.last_angles is not None:
                    ik = self.my_chain.inverse_kinematics(np.array(target_position, dtype=np.float32), target_orientation, orientation_mode="Y", initial_position=self.last_angles)
                else:
                    ik = self.my_chain.inverse_kinematics(np.array(target_position, dtype=np.float32), target_orientation, orientation_mode="Y")
                rounded_ik = (round(angle, precision) for angle in np.degrees(ik))
                self.last_angles = ik  # Store the current angles as last angles
                yield list(rounded_ik)

    def plot_robot(self, target_positions, target_orientations):
            fig, ax = plot_utils.init_3d_figure()
            fig.set_figheight(9)
            fig.set_figwidth(13)
            for i in range(len(target_positions)):
                target_position = target_positions[i]
                target_orientation = target_orientations[i]
                ik_solution = list(self.calculate_ik([target_position], [target_orientation], precision=2, batch_size=1))[0] # Extract the IK solution as a list
                self.my_chain.plot(np.radians(ik_solution), ax, target=np.array(target_position, dtype=np.float32)) # Convert angles to radians before plotting
            plt.xlim(-1, 1)
            plt.ylim(-1, 1)
            ax.set_zlim(-1, 1)
            plt.show()

    def animate_robot(self, target_positions, target_orientations, interval=1):
        fig, ax = plot_utils.init_3d_figure()
        fig.set_figheight(9)
        fig.set_figwidth(13)

        def update(frame):
            ax.clear()
            target_position = target_positions[frame]
            target_orientation = target_orientations[frame]
            ik_solution = list(self.calculate_ik([target_position], [target_orientation], precision=2, batch_size=1))[0]
            self.my_chain.plot(ik_solution, ax, target=np.array(target_position, dtype=np.float32))
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)

        ani = animation.FuncAnimation(fig, update, frames=len(target_positions), interval=interval, repeat=False)
        plt.show()



# Main function for execution
def main():
    urdf_file_path = "urdf_tes1.urdf"
    robot = RobotArm(urdf_file_path)

    # Example array of target positions
    target_positions = [[0.5, 0.5 , 0.5], [-0.5, 0.5 , 0.1]]
    # Example array of target orientations
    target_orientations = [[-np.pi/4, 0.0, 0] for _ in range(2)]

    # Calculate IK and print the results
    ik_generator = robot.calculate_ik(target_positions, target_orientations, precision=1, batch_size=4)
    for ik in ik_generator:
        print("The angles of each joint are:", ik)

    # Free the memory used by the generator
    ik_generator = None

    # Plot the robotic arm for the specified target positions
    robot.plot_robot(target_positions, target_orientations)


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

if __name__ == "__main__":
    main()

