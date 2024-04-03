"""A script to manage the robotic arm and control its movements."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from queue import LifoQueue
import ast
from scipy.spatial.transform import Rotation
from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *
from moshionPlanning import *
from Messager import *

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

def Hrotate_x(matrix, angle_x):
    """Rotate the given matrix around the x-axis by the specified angle.

    Args:
        matrix (np.array): The matrix to rotate.
        angle_x (float): The angle of rotation in radians.

    Returns:
        np.array: The rotated matrix.
    """
    rotation_matrix_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle_x), -np.sin(angle_x), 0],
        [0, np.sin(angle_x), np.cos(angle_x), 0],
        [0, 0, 0, 1]
    ])
    return np.dot(rotation_matrix_x, matrix)

def Hrotate_y(matrix, angle_y):
    """Rotate the given matrix around the y-axis by the specified angle.

    Args:
        matrix (np.array): The matrix to rotate.
        angle_y (float): The angle of rotation in radians.

    Returns:
        np.array: The rotated matrix.
    """
    rotation_matrix_y = np.array([
        [np.cos(angle_y), 0, np.sin(angle_y), 0],
        [0, 1, 0, 0],
        [-np.sin(angle_y), 0, np.cos(angle_y), 0],
        [0, 0, 0, 1]
    ])
    return np.dot(rotation_matrix_y, matrix)

def Hrotate_z(matrix, angle_z):
    """Rotate the given matrix around the z-axis by the specified angle.

    Args:
        matrix (np.array): The matrix to rotate.
        angle_z (float): The angle of rotation in radians.

    Returns:
        np.array: The rotated matrix.
    """
    rotation_matrix_z = np.array([
        [np.cos(angle_z), -np.sin(angle_z), 0, 0],
        [np.sin(angle_z), np.cos(angle_z), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return np.dot(rotation_matrix_z, matrix)

def Htranslation(matrix, translation_vector):
    """
    Create a 4x4 homogeneous translation matrix.

    Args:
        translation_vector (array-like): The translation vector, e.g., [x, y, z].

    Returns:
        numpy.ndarray: The 4x4 homogeneous translation matrix.
    """

    if len(translation_vector) != 3:
        raise ValueError("Translation vector must have three elements (x, y, z).")
    
    translation_matrix = np.eye(4)
    translation_matrix[:3, 3] = translation_vector
    return np.dot(translation_matrix, np.transpose(matrix))

def translate_point_along_z(point, orientation_matrix, translation_distance):
    """_summary_

    Args:
        point (_type_): _description_
        orientation_matrix (_type_): _description_
        translation_distance (_type_): _description_

    Returns:
        _type_: _description_
    """
    final_coordinates = np.dot(orientation_matrix, [0, 0, translation_distance]) + point

    return final_coordinates

def XY_angle(vector1, vector2):
    """_summary_

    Args:
        vector1 (_type_): _description_
        vector2 (_type_): _description_

    Returns:
        _type_: _description_
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
    
def handle_response(expected_r: str = None, expected_prefix: str = None, dataWrite: list = [], message_stack: LifoQueue = None):
    """Handle response based on expected response or prefix.

    Args:
        expected_r (str, optional): Expected response. Defaults to None.
        expected_prefix (str, optional): Expected prefix. Defaults to None.
        dataWrite (list, optional): List to write data when expected_prefix is found. Defaults to [].
        message_stack (LifoQueue, optional): Stack of messages. Defaults to None.
    """
    while True:
        response = message_stack.get()  # Get the latest response from the message stack
        if response is None:  # If response is None, continue to wait for the next response
            continue

        if expected_r is not None and expected_prefix is None:  # If expected_r is provided but expected_prefix is not
            if response == expected_r:  # Check if the response matches the expected response
                print(response)  # Print the response
                return  # Exit the function since the expected response is received
            else:
                print(f">>> {response}")

        elif expected_r is None and expected_prefix is not None:  # If expected_r is not provided but expected_prefix is
            if response.startswith(expected_prefix):  # Check if the response starts with the expected prefix
                dataWrite.extend(response.split())  # Split the response and add it to the dataWrite list
                return  # Exit the function since the response with the expected prefix is received
            else:
                print(f">>> {response}")

def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix into a quaternion.

    Args:
        rotation_matrix (np.array): A 3x3 rotation matrix.

    Returns:
        np.array: A 4-element array representing the quaternion.
    """
    R = np.array(rotation_matrix, dtype=np.float64)
    
    trace = np.trace(R)
    
    if trace > 0:
        S = 2.0 * np.sqrt(trace + 1.0)
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    
    quaternion = np.array([qw, qx, qy, qz])

    return quaternion

def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion into a 3x3 rotation matrix.

    Args:
        quaternion (list or np.array): A 4-element list or array representing the quaternion.

    Returns:
        np.array: A 3x3 rotation matrix.
    """
    q = np.array(quaternion, dtype=np.float64)
    q = q / np.linalg.norm(q)

    w, x, y, z = q
    rotation_matrix = np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
                                 [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
                                 [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]])

    return rotation_matrix

def quaternion_multiply_2q(q1: np.ndarray, q2: np.ndarray) -> np.ndarray :
    """
    Multiply two quaternions.

    Args:
        q1 (np.array): The first quaternion [w, x, y, z].
        q2 (np.array): The second quaternion [w, x, y, z].

    Returns:
        np.array: The resulting quaternion.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def rotate_quaternion(quat: np.ndarray, angle_x:float, angle_y:float, angle_z:float) -> np.ndarray :
    """
    Rotate a quaternion around the unit vectors.

    Args:
        quat (np.array): The original quaternion [w, x, y, z].
        angle_x (float): Angle to rotate around the x-axis (in radians).
        angle_y (float): Angle to rotate around the y-axis (in radians).
        angle_z (float): Angle to rotate around the z-axis (in radians).

    Returns:
        np.array: The rotated quaternion [w, x, y, z].
    """
    # Quaternion representing the rotation around x-axis
    qx = np.array([np.cos(angle_x / 2), np.sin(angle_x / 2), 0, 0])
    
    # Quaternion representing the rotation around y-axis
    qy = np.array([np.cos(angle_y / 2), 0, np.sin(angle_y / 2), 0])
    
    # Quaternion representing the rotation around z-axis
    qz = np.array([np.cos(angle_z / 2), 0, 0, np.sin(angle_z / 2)])
    
    # Combine the rotations by quaternion multiplication
    rotated_quat = quaternion_multiply_2q(quaternion_multiply_2q(quaternion_multiply_2q(quat, qx), qy), qz)

    return rotated_quat

def quaternion_slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    """
    Perform spherical linear interpolation (slerp) between two quaternions.

    Args:
        q1 (numpy.ndarray): The first quaternion as a 4-element numpy array [w, x, y, z].
        q2 (numpy.ndarray): The second quaternion as a 4-element numpy array [w, x, y, z].
        t (float): Interpolation parameter. It ranges from 0 to 1. 
                   For t=0, the resulting quaternion is equal to q1.
                   For t=1, the resulting quaternion is equal to q2.

    Returns:
        numpy.ndarray: The interpolated quaternion as a 4-element numpy array [w, x, y, z].
    """
    # Ensure quaternions are normalized
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    dot_product = np.dot(q1, q2)
    
    # Determine the sign
    if dot_product < 0.0:
        q1 = -q1
        dot_product = -dot_product

    # Clamp dot product to ensure stability
    dot_product = min(1.0, max(-1.0, dot_product))
    
    # Calculate the angle between the quaternions
    theta_0 = np.arccos(dot_product)
    sin_theta_0 = np.sin(theta_0)
    
    # Perform interpolation
    s0 = np.sin((1 - t) * theta_0) / sin_theta_0
    s1 = np.sin(t * theta_0) / sin_theta_0
    
    return (s0 * q1) + (s1 * q2)

LARGE_FONT = ("Verdana", 12)
DROP_OFF_ZONE = (-0.35, -0.70, 0.0)
DROP_OFF_ORIENTATION = rotate_y(rotate_x(np.eye(3), np.pi/2), np.pi/2)
IDLE_POSITION = (0.01, -0.615, 0.15)
IDLE_ORIENTATION  = rotate_y(rotate_x(np.eye(3), np.pi/2), np.pi/2)
IDLE_AGLE_POSITION = np.array([ 0.00000000e+00,  2.34698997e-03,  -8.50613920e-01,  0.00000000e+00, -2.28573529e+00,  2.34669544e-03,  0.00000000e+00, 1.56555290e+00,3.14158035e+00])
WORKING_HIGHT = 0.1
WORKING_POSITION = (*IDLE_POSITION[:-1], IDLE_POSITION[-1] + 0.1)


PICK_UP_ZONE = (0.35, -0.70, 0.0)
GRAB_DISTANCE_Z = (0, 0, 0.1)
NORTH_WALL = (0.0, 1.0, 0.0)
EAST_WALL = (1.0, 0.0, 0.0)
WEST_WALL = (-1.0, 0.0, 0.0)
SOUTH_WALL = (0.0, -1.0, 0.0)

NORTH_WALL_TRANFORM = Htranslation(np.eye(4), [0.7, 0.7, -0.3])
SOUTH_WALL_TRANFORM = Hrotate_x(Hrotate_z(Htranslation(np.eye(4), [0.7, -0.7, -0.3]), -np.pi), -np.pi/2)
EAST_WALL_TRANFORM = Hrotate_x(Hrotate_z(Htranslation(np.eye(4), [0.7, 0.7, -0.3]), -np.pi/2), -np.pi/2)
WEST_WALL_TRANFORM = Hrotate_x(Hrotate_z(Htranslation(np.eye(4), [-0.7, -0.7, -0.3]), np.pi/2), -np.pi/2)


AllMotorNames = ["Bace Motor", "Sholder Motor", "Elbow Motor", "Elbow revolut Motor", "Wirist Motor", "Wirist revolut Motor"]

ERRORmsg = [
    "Get the list of parts to fetch from the database",
    "Get the part info from the database and organize it in a dictionary",
    "Get the locations and orientations of all parts",
    "Get the paths and Velocity profile for each path"
]

def state0():
    """
    Initialize the state machine to fetch part names and determine whether to pick up or drop off.

    Returns:
        tuple: A tuple containing part names to fetch and a flag indicating pickup/dropoff.
    """
    part_names_to_fetch = [f'Box {i+1}' for i in range(18, 22)]
    pickip_dropoff = True
    return part_names_to_fetch, pickip_dropoff

def state1(part_names_to_fetch, parts_db):
    """
    Fetch information about parts from the parts database.

    Args:
        part_names_to_fetch (list): A list of part names to fetch information for.
        parts_db (PartsDatabase): An instance of the PartsDatabase class containing part information.

    Returns:
        dict: A dictionary containing information about the fetched parts.
            Keys: Part names
            Values: Dictionary containing part information
                - 'PartName': Name of the part
                - 'LocationX': X-coordinate of the part's location
                - 'LocationY': Y-coordinate of the part's location
                - 'LocationZ': Z-coordinate of the part's location (assumed to be 0)
                - 'Orientation': Orientation of the part
                - 'FullWeight': Weight of the part when full
                - 'HalfWeight': Weight of the part when half full
                - 'EmptyWeight': Weight of the part when empty
                - 'InService': Flag indicating if the part is in service
    """
    part_info_dict = {}
    for part_name_to_find in part_names_to_fetch:
        # Get part information from the parts database
        part = parts_db.get_part_by_name(part_name_to_find)
        if part:
            # Construct a dictionary with part information
            part_info_dict[part_name_to_find] = {
                'PartName': part.name,
                'LocationX': ast.literal_eval(part.position)[0],
                'LocationY': ast.literal_eval(part.position)[1],
                'LocationZ': 0,  # Assuming Z-coordinate is 0
                'Orientation': ast.literal_eval(part.orientation),
                'FullWeight': part.FullWeight,
                'HalfWeight': part.HalfWeight,
                'EmptyWeight': part.EmptyWeight,
                'InService': part.InService
            }
    return part_info_dict

def state2(part_info_dict):
    """
    Extract locations and orientations of parts from the part information dictionary.

    Args:
        part_info_dict (dict): A dictionary containing information about parts.

    Returns:
        tuple: A tuple containing dictionaries of part locations and orientations.
            - locations (dict): Dictionary containing part names as keys and their respective locations as values.
            - orientations (dict): Dictionary containing part names as keys and their respective orientations as values.
    """
    # Initialize dictionaries to store locations and orientations
    locations = {}
    orientations = {}

    # Iterate over each part in the part information dictionary
    for part_name, part_info in part_info_dict.items():
        # Extract location and orientation information from part info
        location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
        orientation = tuple(part_info['Orientation'])

        # Store location and orientation information in respective dictionaries
        locations[part_name] = location
        orientations[part_name] = orientation

    # Return dictionaries containing locations and orientations
    return locations, orientations

def state3(pickip_dropoff,locations, orientations, drop_off_zone, planner):
    """
    Generate travel paths for the robotic arm to pick up and drop off parts.

    Args:
        pickip_dropoff (bool): A flag indicating whether the robot is picking up or dropping off parts.
        locations (dict): Dictionary containing part names as keys and their respective locations as values.
        orientations (dict): Dictionary containing part names as keys and their respective orientations as values.
        drop_off_zone (tuple): The drop-off zone coordinates (x, y, z).
        planner (PathPlanner): An instance of the PathPlanner class for path generation.

    Returns:
        tuple: A tuple containing lists of travel paths, orientations, and alignment flags for the robotic arm.
            - travle_paths (list): List of travel paths for the robotic arm.
            - travle_orientation (list): List of orientations corresponding to the travel paths.
            - travle_alinements (list): List of alignment flags for the robotic arm.
    """
    travle_paths = []
    travle_orientation = []
    travle_alinements = []
    T0= []
    T1= []
    T2= []
    T3= []
    T4= []

    T0 = planner.generate_path(IDLE_POSITION, WORKING_POSITION, linear=True)
    T0_orientation = [IDLE_ORIENTATION for _ in range(len(T0))]

    travle_paths.extend(T0)
    travle_orientation.extend(T0_orientation)

    T0 = planner.generate_path(WORKING_POSITION, drop_off_zone, linear=True)
    T0_orientation = [IDLE_ORIENTATION for _ in range(len(T0))]

    travle_paths.extend(T0)
    travle_orientation.extend(T0_orientation)

    for index, (location, orientation) in enumerate(zip(locations.values(), orientations.values())):
        location = np.array(location)
        location = np.append(location, [0])
        
        if (orientation == EAST_WALL):
            location = [0.7, 0.7 - location[0], location[1]-0.3]

        elif (orientation == NORTH_WALL):
            location = [location[0]-0.7, 0.7, location[1]-0.3]
            
        elif (orientation == WEST_WALL):
            location = [-0.7, location[0]-0.7, location[1]-0.3]
            
        elif (orientation == SOUTH_WALL):
            location = [0.7-location[0], -0.7, location[1]-0.3]
        else:
            print("No angle")

        delta_angle = XY_angle(drop_off_zone, location)

        Linear = False
        # if (abs(delta_angle) <= (19/24)*np.pi):
        #     Linear = True
            
        T1 = planner.generate_path(drop_off_zone, location, linear=Linear)
        

        if (orientation == EAST_WALL):
            if (delta_angle >= 0):
                delta_angles = np.linspace(0, np.pi/2, len(T1))
            else:
                delta_angles = np.linspace(0, -np.pi/2, len(T1))
        
        elif (orientation == NORTH_WALL):
            if (delta_angle >= 0):
                delta_angles = np.linspace(0, np.pi, len(T1))
            else:
                delta_angles = np.linspace(0, -np.pi, len(T1))

        elif (orientation == WEST_WALL):
            if (delta_angle >= 0):
                delta_angles = np.linspace(0, np.pi/2, len(T1))
            else:
                delta_angles = np.linspace(0, -np.pi/2, len(T1))

        elif (orientation == SOUTH_WALL):
            if (delta_angle <= np.pi):
                delta_angles = np.linspace(0, 0, len(T1))
            else:
                delta_angles = np.linspace(0, 0, len(T1))
        else:
            print("No angle")
            delta_angles = np.linspace(0, 0, len(T1))




        T1_orientation = [rotate_z(T0_orientation[-1], rad) for rad in delta_angles]
        
        translated_point = np.dot(T1_orientation[-1], GRAB_DISTANCE_Z) + T1[-1]

        T2 = planner.generate_path(T1[-1], translated_point, linear=True)
        T2_orientation = [T1_orientation[-1] for _ in range(len(T2))]

        T3 = planner.generate_path(T2[-1], location, linear=True)
        T3_orientation = [T2_orientation[-1] for _ in range(len(T3))]


        T4 = planner.generate_path(location, drop_off_zone, linear=Linear)
        delta_angles = np.linspace(delta_angles[0], delta_angles[-1], len(T4))
        T4_orientation = [rotate_z(T3_orientation[-1], -rad) for rad in delta_angles]

        travle_paths.extend(T1)
        travle_paths.extend(T2)
        travle_paths.extend(T3)
        travle_paths.extend(T4)
        
        travle_orientation.extend(T1_orientation)
        travle_orientation.extend(T2_orientation)
        travle_orientation.extend(T3_orientation)
        travle_orientation.extend(T4_orientation)
        
        
    T5 = planner.generate_path(travle_paths[-1], WORKING_POSITION, linear=True)
    T6 = planner.generate_path(T5[-1], IDLE_POSITION, linear=True)
    T56_orientation = [IDLE_ORIENTATION for _ in range(len(T6)+len(T5))]

    travle_paths.extend(T5)
    travle_paths.extend(T6)
    
    travle_orientation.extend(T56_orientation)
    travle_alinements = ["all"] * len(travle_paths)

    T0 = np.array([])
    T1 = np.array([])
    T2= np.array([])
    T2= np.array([])
    T3= np.array([])
    T4= np.array([])
    T5= np.array([])
    T6= np.array([])
    T0_orientation= np.array([])
    T1_orientation= np.array([])
    T2_orientation= np.array([])
    T3_orientation= np.array([])
    T4_orientation= np.array([])
    T56_orientation= np.array([])
    planner.plot_3d_path()
    
    return travle_paths, travle_orientation, travle_alinements

def state4(travle_paths, travle_orientation, travle_alinements, urdf_file_path):
    """
    Execute the robotic arm's travel paths and orientations using inverse kinematics.

    Args:
        travle_paths (list): List of travel paths for the robotic arm.
        travle_orientation (list): List of orientations corresponding to the travel paths.
        travle_alinements (list): List of alignment flags for the robotic arm.
        urdf_file_path (str): The file path of the URDF file defining the robotic arm.
    """
    # Initialize the RobotArm with the URDF file path
    robot = RobotArm(urdf_file_path, IDLE_AGLE_POSITION)

    robot.animate_ik(travle_paths, travle_orientation, travle_alinements, interval=1)

def joggingXYZ(init_angles):
   
    def create_gui(controll):
        """
        Create the GUI for entering joint angles.

        Args:
            controll (list): Initial joint positions and orientations.
        """
        
        default_values = [str(value) for value in controll]
        for i, label in enumerate(['X Cor', 'Y Cor', 'Z cor', "X_Rot", "Y_Rot", "Z_Rot"]):
            tk.Label(root, text=label).grid(row=i, column=0)
            entry = tk.Entry(root)
            entry.insert(tk.END, default_values[i])  # Set default value
            entry.grid(row=i, column=1)
            angle_entries.append(entry)
    
    def Send_orintashion():
        nonlocal current_ORI_quater
        _COR_path = []
        New_ORI = []

        #get the previos anle poshion
        FK_pre = robot.calculate_fk([robot.last_angles], 1)
        for fk in FK_pre:
            Pre_COR = fk[0]
            Pre_ORI = rotation_matrix_to_quaternion(fk[1])

        # get the y x z rotashion angles in rad
        inputs = [float(entry.get()) for entry in angle_entries]

        # get the dirsired quaternion
        end_quaternion = rotate_quaternion([1, 0, 0, 0], inputs[3], inputs[4], inputs[5])

        for t in np.linspace(0, 1, 50):
            V_rot_theta = quaternion_slerp(Pre_ORI, end_quaternion, t)
            New_ORI.append(quaternion_to_rotation_matrix(V_rot_theta))

        COR_path = [Pre_COR for _ in range(0, len(New_ORI), 1)]
        New_ORI_MODE = ["all" for _ in range(0, len(New_ORI), 1)]
        animationRobot(COR_path, New_ORI, New_ORI_MODE)


        
    def Send_poshion():
        _COR_path = []

        #get the previos anle poshion
        FK_pre = robot.calculate_fk([robot.last_angles], 1)
        for fk in FK_pre:
            Pre_COR = fk[0]
            Pre_ORI = fk[1]

        inputs = [float(entry.get()) for entry in angle_entries]

        # get the nect point
        New_COR = inputs[:3]
        New_ORI = Pre_ORI

        # will return an array of points to move the robot to
        COR_path = COR_planner.generate_path(Pre_COR, New_COR, linear=True)

        New_ORI = [Pre_ORI for _ in range(0, len(COR_path), 1)]
        New_ORI_MODE = ["all" for _ in range(0, len(COR_path), 1)]

        animationRobot(COR_path, New_ORI, New_ORI_MODE)

    def animationRobot(COR_path, New_ORI, New_ORI_MODE):

        IK_SOLUSHIONS = list(robot.calculate_ik(COR_path, New_ORI, New_ORI_MODE, 1))
        TRANFORM_MASK = np.array([False, True, True, False, True, True, False, True, True])

        new_angles = []
        for SOLUSHION in IK_SOLUSHIONS:
            angles = np.rad2deg(SOLUSHION[0])
            new_angles.append(angles[TRANFORM_MASK])

        movement_results = controller.move_motors(new_angles)

        for move in movement_results:
            print(move)
            
        #robot.animate_ik(COR_path, New_ORI, New_ORI_MODE, ax = ax, fig = fig)

        

    # Create an instance of MoshionController
    controller = MoshionController()
    controller.Input_DEGREES()

    # Create the Tkinter window
    root = tk.Tk()
    root.title("Enter Angles")

    max_acc = 30
    max_vel = 30

    max_ang_acc = 1
    max_ang_vel = 1
    COR_planner = PathPlanner(max_acc, max_vel)

    urdf_file_path = "app\\backend\\python code\\urdf_tes2.urdf"
    robot = RobotArm(urdf_file_path, init_angles)
    FK = robot.calculate_fk([init_angles], 1)

    controll = []
    for fk in FK:
        current_COR = fk[0]
        current_ORI_quater = rotation_matrix_to_quaternion(fk[1])
        controll.extend(current_COR)
        controll.extend([0,0,0])

    angle_entries = [] 
    create_gui(controll)

    # Create a button to send the entered angles
    send_poshion_button = tk.Button(root, text="Send poshion", command=Send_poshion)
    send_poshion_button.grid(row=len(angle_entries), columnspan=3)

    # Create a button to send the entered angles
    send_orintashion_button = tk.Button(root, text="Send Orintashion", command=Send_orintashion)
    send_orintashion_button.grid(row=len(angle_entries)+1, columnspan=4)
    

    fig, ax = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(12, 8))
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(-1, 1)

    # Run the Tkinter event loop
    root.mainloop()

def joggingAngles():

    # Create an instance of MoshionController
    controller = MoshionController()

    MSG = Mesageer("COM12")
    MSG.connect()

    # Create the Tkinter window
    root = tk.Tk()
    root.title("Enter Angles")

    # Create entry fields for entering angles
    angle_entries = []
    default_values = ["0"] * 6  # Default values set to 0
    for i, label in enumerate(['Shoulder Rotate Motor', 'Shoulder Lift Motor', 'Elbow Motor', 'Forearm Rotate Motor', 'Wrist Motor', 'Wrist Rotate Motor']):
        tk.Label(root, text=label).grid(row=i, column=0)
        entry = tk.Entry(root)
        entry.insert(tk.END, default_values[i])  # Set default value
        entry.grid(row=i, column=1)
        angle_entries.append(entry)
    
    def send_angles():
        angles = tuple(float(entry.get()) for entry in angle_entries)
        
        # Move motors based on the defined angles and get the results
        movement_results = controller.move_motors([(0,0,0,0,0,0), angles])
        print(movement_results[0])

        expected_response = "MoshionState changed to: 1"
        MSG.send_message("R_MOVES 1")
        handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

        expected_response = "storing 1"
        MSG.send_message("R_MOSHION 1")
        handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

        expected_response = "MoshionState changed to: 2"
        MSG.send_message(movement_results[0])
        handle_response(expected_response, None, None, MSG.message_stack) # weight until i get the my response

        expected_response = "MoshionState changed to: 0"
        MSG.send_message("R_EXECUTE 1")
        handle_response(expected_response, None, None, MSG.message_stack) # weight until i get the my response
        
    # Create a button to send the entered angles
    send_button = tk.Button(root, text="Send Angles", command=send_angles)
    send_button.grid(row=len(angle_entries), columnspan=2)
    
    # Run the Tkinter event loop
    root.mainloop()


def main():
    try:
        """Initialize the motors and the parts database."""
        # Initialize motors and parts database here

        # Initialize state and run variables
        state = 0
        run = True

        box_w = 0.2
        box_h = 0.25
        grid_size = (1.40, 1.40)

        parts_db = PartsDatabase("parts_db", grid_size = grid_size, shelf_height=0.015, margin=0.015, offset_x=(box_w / 2), offset_y=(box_h / 2))
        parts_db.create_parts_table()
       
        #urdf_file_path = "E:\\Capstone\\app\\backend\\python code\\urdf_tes2.urdf"
        urdf_file_path = "C:\\Users\\zachl\\Capstone2024\\app\\backend\\python code\\urdf_tes2.urdf"
        #urdf_file_path = "//home//zachl//Capstone//app/backend//python code//urdf_tes2.urdf"
        

        max_acc = 50
        max_vel = 50
        planner = PathPlanner(max_acc, max_vel)

        Mode = False

        if Mode:

            while run:
                try:
                    if state == 0:
                        part_names_to_fetch, pickip_dropoff = state0()
                        state = 1

                    elif state == 1:
                        part_info_dict = state1(part_names_to_fetch, parts_db)
                        state = 2

                    elif state == 2:
                        locations, orientations = state2(part_info_dict)
                        state = 3

                    elif state == 3:
                        travle_paths, travle_orientation, travle_alinements = state3(pickip_dropoff, locations, orientations, DROP_OFF_ZONE, planner)
                        state = 4

                    elif state == 4:
                        state4(travle_paths, travle_orientation, travle_alinements, urdf_file_path)
                        state = 5

                    elif state == 5:
                        run = False
                except Exception as e:
                    print(f"An error occurred at state {state}\n{ERRORmsg[state]}:", e)
                    run = False
        else:
            pass

    except Exception as e:
        print(f"Error: {e}")
        run = False

if __name__ == "__main__":
    """Entry point of the script."""
    
    #joggingAngles()

    # Example usage
    # print(quaternion_to_euler((-0.0262, -0.9362, -0.0785, 0.3416)))
    # print(quaternion_to_euler((0.9902, 0.0302, -0.1365, 0.0001)))
    # print(quaternion_to_euler((0.7023, -0.06951, -0.01500, 0.0329)))
    # print(quaternion_to_euler((0.9866, 0.0035, -0.0667, 0.1486)))

    joggingXYZ([0, 0, 0, 0, 0, 0, 0, 0, 0])

    #joggingAngles()

    #init_angles = [0, 138.6653286, 17.4672121, -14.51805191, -8.8323662, 0]



    
    

