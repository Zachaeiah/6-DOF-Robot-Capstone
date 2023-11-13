"""A script to manage the robotic arm and control its movements."""
import numpy as np
import tkinter as tk
from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *

LARGE_FONT = ("Verdana", 12)
DROP_OFF_ZONE = (-0.50, -0.100, 0.0)
PICK_UP_ZONE = (0.50, -0.100, 0.50)

AllMotorNames = ["Bace Motor", "Sholder Motor", "Elbow Motor", "Elbow revolut Motor", "Wirist Motor", "Wirist revolut Motor"]

ERRORmsg = [
    "Get the list of parts to fetch from the database",
    "Get the part info from the database and organize it in a dictionary",
    "Get the locations and orientations of all parts",
    "Get the paths and Velocity profile for each path"
]


def main1():
    try:
        """Initialize the motors and the parts database."""

        active_motors = range(3, 6, 1)
        motors = {f"Motor: {i}": StepperMotor(f"Motor: {AllMotorNames[i]}", i, 100, 10, 1, 70000) for i in range(0, 6, 1)}

        # Create a MotorManager instance
        motor_manager = motorManager(motors)

        # Create a PartsDatabase instance and create the Parts table
        parts_db = PartsDatabase()
        parts_db.create_parts_table()

        state = 0
        run = True  # Initialize run to True to enter the loop

        # State machine
        while run:
            """Run the state machine to perform various actions."""

            try:
                print(f'state: {state}')
                if state == 0:
                    """Retrieve the list of parts to fetch."""

                    # Array of part names to fetch
                    part_names_to_fetch = ['Part1', 'Part2', 'Part3']
                    state = 1  # Transition to the next state

                elif state == 1:
                    """Retrieve and store part information from the database."""

                    part_info_dict = {}  # Dictionary to store part information

                    # Retrieve and store information for specified parts
                    for part_name_to_find in part_names_to_fetch:
                        part = parts_db.get_part_by_name(part_name_to_find)
                        if part:
                            part_info_dict[part_name_to_find] = {
                                'PartName': part[1],
                                'NumberOfParts': part[2],
                                'LocationX': part[3],
                                'LocationY': part[4],
                                'LocationZ': part[5],
                                'Orientation': [part[6], part[7], part[8]],
                                'FullWeight': part[9],
                                'HalfWeight': part[10],
                                'EmptyWeight': part[11],
                                'InService': part[12]
                            }
                    state = 2  # Transition to the next state

                elif state == 2:
                    """Retrieve and store locations of all parts."""

                    locations = {}  # Dictionary to store locations of each part

                    for part_name, part_info in part_info_dict.items():
                        location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
                        locations[part_name] = location

                    state = 3  # Transition to the next state

                elif state == 3:
                    """Generate paths and velocity profiles for each part."""

                    planner = PathPlanner(20, 2)
                    planner.setVelocityPFP(1)

                    direction_vector = []
                    paths = []
                    for part_name, location in locations.items():
                        direction_vector.append((np.array(location) / np.linalg.norm(location), 0, 0))  # Calculate direction vector from the origin to the current location
                        paths.append(planner.generate_path(location, DROP_OFF_ZONE, linear=False))

                    # Plot the 3D paths
                    planner.plot_3d_path()

                    state = 4

                elif state == 4:
                    """Perform inverse kinematics for the generated paths and visualize the motion using RobotArm."""

                    # Initialize the RobotArm with the URDF file path
                    urdf_file_path = "app\\backend\\python code\\urdf_tes1.urdf"  # Replace with the actual file path
                    robot = RobotArm(urdf_file_path)

                    target_positions = []
                    target_orientations = []

                    for path in paths:
                        for point in path:
                            target_positions.append(point)
                            # Define a default orientation (you may need to adjust this based on your specific setup)
                            target_orientations.append([0, 0, np.pi/4])

                    print("animateing")
                    # Animate the robotic arm along the generated path
                    robot.animate_robot(target_positions, target_orientations, interval=1,save_as_gif=False)  # Adjust arguments as needed

                    state = 5

                elif state == 5:

                    run = False  # Exit the loop when done

            except Exception as e:
                print(f"An error occurred at state {state}\n{ERRORmsg[state]}\t:", e)

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    """Entry point of the script."""

    main1()


    
    
