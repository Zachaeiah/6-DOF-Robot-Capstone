"""A script to manage the robotic arm and control its movements."""
import numpy as np
import timeit
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *

def rotate_x(matrix, angle_x):
    """Rotate a 3x3 matrix around the X-axis."""
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x), -np.sin(angle_x)],
        [0, np.sin(angle_x), np.cos(angle_x)]
    ])
    rotated_matrix = np.dot(rotation_matrix_x, matrix)
    return rotated_matrix

def rotate_y(matrix, angle_y):
    """Rotate a 3x3 matrix around the Y-axis."""
    rotation_matrix_y = np.array([
        [np.cos(angle_y), 0, np.sin(angle_y)],
        [0, 1, 0],
        [-np.sin(angle_y), 0, np.cos(angle_y)]
    ])
    rotated_matrix = np.dot(rotation_matrix_y, matrix)
    return rotated_matrix

def rotate_z(matrix, angle_z):
    """Rotate a 3x3 matrix around the Z-axis."""
    rotation_matrix_z = np.array([
        [np.cos(angle_z), -np.sin(angle_z), 0],
        [np.sin(angle_z), np.cos(angle_z), 0],
        [0, 0, 1]
    ])
    rotated_matrix = np.dot(rotation_matrix_z, matrix)
    return rotated_matrix

def translate_point_along_z(point, orientation_matrix, translation_distance):
    final_coordinates = np.dot(orientation_matrix, [0, 0, translation_distance]) + point

    return final_coordinates

def XY_angle(vector1, vector2):
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


LARGE_FONT = ("Verdana", 12)
DROP_OFF_ZONE = (-0.3, -0.50, 0.0)
DROP_OFF_ORINT = rotate_x(np.eye(3), np.pi/2)
IDLE_POSITION = (0.0, -0.43209168, -1.21891881, 0.0, -1.92214302,  1.138704, 0.0, -1.57057404,  0.0)

PICK_UP_ZONE = (0.51, -0.50, 0.00)
GRAB_DISTANCE_Z = [0, 0, 0.1]
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
                    part_names_to_fetch = ['Part0','Part1', 'Part6', 'Part7']
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
                    orientations = {}  # Dictionary to store orientations of each part

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
                    travle_orientation = []
                    travle_alinements = []
                    

                    planner = PathPlanner(max_acc, max_vel)

                    drop_off_zone = np.array(DROP_OFF_ZONE)

                    for location, orientation in zip(locations.values(), orientations.values()):
                    
                        T1 = planner.generate_path(drop_off_zone, location, linear=False)

                        if (orientation == EAST_WALL):
                            if (XY_angle(drop_off_zone, location) >= 0):
                                delta_angles = np.linspace(0, np.pi/2, len(T1))
                            else:
                                delta_angles = np.linspace(0, -np.pi/2, len(T1))
                        
                        elif (orientation == NORTH_WALL):
                            if (XY_angle(drop_off_zone, location) >= 0):
                                delta_angles = np.linspace(0, np.pi, len(T1))
                            else:
                                delta_angles = np.linspace(0, -np.pi, len(T1))

                        elif (orientation == WEST_WALL):
                            if (XY_angle(drop_off_zone, location) >= 0):
                                delta_angles = np.linspace(0, np.pi/2, len(T1))
                            else:
                                delta_angles = np.linspace(0, -np.pi/2, len(T1))

                        elif (orientation == SOUNTH_WALL):
                            if (XY_angle(drop_off_zone, location) <= np.pi):
                                delta_angles = np.linspace(0, 0, len(T1))
                            else:
                                delta_angles = np.linspace(0, 0, len(T1))
                        else:
                            print("No angle")
                            delta_angles = np.linspace(0, 0, len(T1))

                        #print(np.degrees(delta_angles))

                        T1_orientation = [rotate_z(DROP_OFF_ORINT, rad) for rad in delta_angles]
                        
                        translated_point = np.dot(T1_orientation[-1], GRAB_DISTANCE_Z) + T1[-1]

                        T2 = planner.generate_path(T1[-1], translated_point, linear=True)
                        T2_orientation = [T1_orientation[-1] for _ in range(len(T2))]

                        T3 = planner.generate_path(T2[-1], location, linear=True)
                        T3_orientation = [T2_orientation[-1] for _ in range(len(T3))]
                    
                        T4 = planner.generate_path(location, drop_off_zone, linear=False)
                        T4_orientation = [rotate_z(T3_orientation[-1], -rad) for rad in delta_angles]

                        travle_paths.extend(T1)
                        travle_paths.extend(T2)
                        travle_paths.extend(T3)
                        travle_paths.extend(T4)

                        travle_orientation.extend(T1_orientation)
                        travle_orientation.extend(T2_orientation)
                        travle_orientation.extend(T3_orientation)
                        travle_orientation.extend(T4_orientation)


                    travle_alinements = ["all"for _ in range(len(travle_paths))]

                    # # Plot the 3D paths
                    # planner.plot_3d_path()

                    state = 4
                    elapsed_time = timeit.default_timer() - start_time
                    print(f"State {state} execution time: {elapsed_time} seconds")

                elif state == 4:
                    start_time = timeit.default_timer()
                    """Perform inverse kinematics for the generated paths and visualize the motion using RobotArm."""

                    # Initialize the RobotArm with the URDF file path
                    
                    urdf_file_path = "E:\\Capstone\\app\\backend\\python code\\urdf_tes2.urdf"
                    #urdf_file_path = "C:\\Users\\zachl\\Capstone2024\\app\\backend\\python code\\urdf_tes2.urdf"
                    robot = RobotArm(urdf_file_path, IDLE_POSITION)

            
                    print("animateing")

                    print(len(travle_paths), len(travle_orientation), len(travle_alinements))

                    robot.animate_ik(travle_paths, travle_orientation, travle_alinements, interval=250)

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


    
    

