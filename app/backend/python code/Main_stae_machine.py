"""A script to manage the robotic arm and control its movements."""
import numpy as np
import timeit
from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *

LARGE_FONT = ("Verdana", 12)
DROP_OFF_ZONE = (-0.51, -0.50, 0.0)
DROP_OFF_ORINT = [0, -1, 0]

PICK_UP_ZONE = (0.51, -0.50, 0.00)
GRAB_DISTANCE = 0.10
NORTH_WALL = (0.0, 1.0, 0.0)
EAST_WALL = (1.0, 0.0, 0.0)
WEST_WALL = (-1.0, 0.0, 0.0)
SOUNTH_WALL = (0.0, -1.0, 0.0)

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
                if state == 0:
                    """Retrieve the list of parts to fetch."""

                    # Array of part names to fetch
                    part_names_to_fetch = ['Part0', 'Part1']
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
                    start_time = timeit.default_timer()
                    """Retrieve and store locations of all parts."""

                    locations = {}  # Dictionary to store locations of each part
                    orientations = {}

                    for part_name, part_info in part_info_dict.items():
                        location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
                        orientation = (part_info['Orientation'][0], part_info['Orientation'][1], part_info['Orientation'][2])
                        locations[part_name] = location
                        orientations[part_name] = orientation


                    state = 3  # Transition to the next state

                    elapsed_time = timeit.default_timer() - start_time
                    print(f"State {state} execution time: {elapsed_time} seconds")

                elif state == 3:
                    start_time = timeit.default_timer()

                    """Generate paths and velocity profiles for each part."""

                    max_acc = 50
                    max_vel = 50
                    travle_paths = []
                    gripper_orientation = []
                    gripper_alinements = []

                    planner = PathPlanner(max_acc, max_vel)

                    drop_off_zone = np.array(DROP_OFF_ZONE)

                    for location, orientation in zip(locations.values(), orientations.values()):
                        location = np.array(location)
                        orientation = np.array(orientation)

                        T1 = planner.generate_path(drop_off_zone, location, linear=False)
                        T2 = planner.generate_path(location, location + GRAB_DISTANCE*orientation, linear=True)
                        T3 = planner.generate_path(location + GRAB_DISTANCE*orientation, location, linear=True)
                        T4 = planner.generate_path(location, drop_off_zone, linear=False)
                        travle_paths.extend(T1)
                        travle_paths.extend(T2)
                        travle_paths.extend(T3)
                        travle_paths.extend(T4)

                        

                        for _ in range (len(travle_paths)):
                            gripper_alinements.append([0, 0, 1])
                            gripper_orientation.append("Y")

                        # if (orientation == NORTH_WALL) or (orientation == SOUNTH_WALL):
                        #     gripper_orientation = np.append(gripper_orientation, np.full(len(current_path), "Y"))
                            

                        # elif (orientation == EAST_WALL) or (orientation == WEST_WALL):
                        #     gripper_orientation = np.append(gripper_orientation, np.full(len(current_path), "Y"))
                            

                    
                    # Plot the 3D paths
                    planner.plot_3d_path()

                    state = 4
                    elapsed_time = timeit.default_timer() - start_time
                    print(f"State {state} execution time: {elapsed_time} seconds")

                elif state == 4:
                    start_time = timeit.default_timer()
                    """Perform inverse kinematics for the generated paths and visualize the motion using RobotArm."""

                    # Initialize the RobotArm with the URDF file path
                    urdf_file_path = "E:\\Capstone\\app\\backend\\python code\\urdf_tes1.urdf"
                    robot = RobotArm(urdf_file_path)

            
                    print("animateing")
                    # Animate the robotic arm along the generated path
                    robot.animate_robot(travle_paths, gripper_alinements, gripper_orientation)

                    state = 5
                    elapsed_time = timeit.default_timer() - start_time
                    print(f"State {state} execution time: {elapsed_time} seconds")

                elif state == 5:

                    run = False  # Exit the loop when done

            except Exception as e:
                print(f"An error occurred at state {state}\n{ERRORmsg[state]}\t:", e)
                run = False

    except Exception as e:
        print(f"Error: {e}")
        run = False


if __name__ == "__main__":
    """Entry point of the script."""

    main1()


    
    

