# Define a custom Tkinter application class
import traceback
import inspect
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import colorchooser
from matplotlib.figure import Figure
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from queue import LifoQueue
import ast
from scipy.spatial.transform import Rotation
import random
import threading
import queue
import time
from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *
from moshionPlanning import *
from Messager import *

LARGE_FONT = ("Verdana", 12)

def error_handling_wrapper(func):
    """A decorator that wraps a function with error handling.

    Args:
        func (function): The function to be wrapped.

    Returns:
        function: The wrapped function with error handling.
    """
    def wrapper(self, *args, **kwargs):
        """Wrapper function that adds error handling to the original function.

        Returns:
            Any: The result of the original function.

        Raises:
            tk.TclError: If a TclError occurs (typically for Tkinter-related issues).
            matplotlib.MatplotlibError: If a MatplotlibError occurs.
            Exception: If any other unexpected error occurs.
        """
        try:
            # Call the original function
            return func(self, *args, **kwargs)
        
        except tk.TclError as tcl_error:
            # Handle TclError, which might occur for Tkinter-related issues
            error_msg = f"TclError in {func.__name__} at line {inspect.currentframe().f_lineno}: {str(tcl_error)}\n"
            error_msg += traceback.format_exc()
            print(error_msg)
            self.popup_error(error_msg)

        except Exception as e:
            # Handle other unexpected errors 
            error_msg = f"Error in {func.__name__} at line {inspect.currentframe().f_lineno}: {str(e)}\n"
            error_msg += traceback.format_exc()
            print(error_msg)
            self.popup_error(error_msg)

    # Copy the original function's metadata to the wrapper
    wrapper.__doc__ = func.__doc__
    wrapper.__name__ = func.__name__
    wrapper.__module__ = func.__module__

    return wrapper

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

def Htranslation(input_matrix, tx, ty, tz):
    """
    Translate a given matrix using a homogeneous translation matrix.

    Parameters:
    input_matrix (numpy.ndarray): Input matrix to be translated.
    tx (float): Translation in the x-axis.
    ty (float): Translation in the y-axis.
    tz (float): Translation in the z-axis.

    Returns:
    numpy.ndarray: Translated matrix.
    """
    # Create the homogeneous translation matrix
    translation_mat = np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])
    
    # Perform matrix multiplication to translate the input matrix
    translated_matrix_homogeneous = np.dot(translation_mat, input_matrix)
    
    return translated_matrix_homogeneous

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
    """Calculate the angle between two vectors in the XY plane.

    Args:
        vector1 (array_like): First vector in the XY plane.
        vector2 (array_like): Second vector in the XY plane.

    Returns:
        float: Angle between the two vectors in radians.
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

    # Check the sign of the cross product to determine orientation
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
                continue

        elif expected_r is None and expected_prefix is not None:  # If expected_r is not provided but expected_prefix is
            if response.startswith(expected_prefix):  # Check if the response starts with the expected prefix
                dataWrite.extend(response.split())  # Split the response and add it to the dataWrite list
                return  # Exit the function since the response with the expected prefix is received
            else:
                print(f">>> {response}")
                continue

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

def quaternion_to_euler(quaternion):
    """Convert quaternion to Euler angles.

    Args:
        quaternion (np.array): Quaternion in the form [w, x, y, z].

    Returns:
        np.array: Euler angles in the form [roll, pitch, yaw].
    """
    w, x, y, z = quaternion
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # Use +/-90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])

def rotation_matrix_to_euler_angles(R: np.ndarray):
    """
    Convert a 3x3 rotation matrix to Euler angles.

    Parameters:
    R : numpy.ndarray
        3x3 rotation matrix.

    Returns:
    numpy.ndarray
        Euler angles in radians as [roll, pitch, yaw].
        
    Notes:
        - This function assumes the rotation matrix represents a proper rotation (orthogonal matrix).
        - Gimbal lock may occur when the pitch angle approaches ±90 degrees.
          In such cases, the resulting roll and yaw angles may become ambiguous.
          Consider handling gimbal lock cases accordingly in your application.
    """
    # Input validation
    if R.shape != (3, 3):
        raise ValueError("Input matrix must be a 3x3 array.")

    # Calculate sin of pitch angle
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    
    # Check for gimbal lock
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    return np.array([roll, pitch, yaw])

LARGE_FONT = ("Verdana", 12)
DROP_OFF_ZONE = (0, -0.73, 0.03)
DROP_OFF_ORIENTATION = rotate_quaternion([1, 0, 0, 0], np.pi/2, 0, np.pi)
IDLE_POSITION = (-0.08847695,  0.47161316, -0.22542325)
IDLE_ORIENTATION  = [ 0.00211846, -0.70549629, -0.02529774,  0.7082588 ]
IDLE_AGLE_POSITION = np.array([0, -(1/96)*np.pi, (3601/7200)*np.pi, 0, -(39/80)*np.pi, -np.pi/2, 0, -(481/960)*np.pi, 0])
WORKING_HIGHT = 0.22197503
WORKING_POSITION = (*IDLE_POSITION[:-1], IDLE_POSITION[-1] + WORKING_HIGHT)
JOGGING_START = (-0.58780819, 0, 0.17683316)
JOGGING_START_ORIENTATION = IDLE_ORIENTATION
WEIGHTING_POSITION = (-0.2, -0.73, 0.115)
WEIGHTING_ORIENTATION = DROP_OFF_ORIENTATION

RROTASHION_BUFFER_DIS = 0.1
insershion_distance =  0.2 + RROTASHION_BUFFER_DIS
lifting_distance = 0.015
Hight_drop_off_box = 0.1
Gripper_offset = [0, -0.2, 0.006]

NORTH_WALL = (0.0, 1.0, 0.0)
EAST_WALL = (1.0, 0.0, 0.0)
WEST_WALL = (-1.0, 0.0, 0.0)
SOUTH_WALL = (0.0, -1.0, 0.0)

NORTH_WALL_TRANFORM = Htranslation(Hrotate_x(np.eye(4), np.pi/2), -0.7, -0.7, -0.3)
# SOUTH_WALL_TRANFORM = Hrotate_x(Hrotate_z(Htranslation(np.eye(4), [0.7, -0.7, -0.3]), -np.pi), -np.pi/2)
# EAST_WALL_TRANFORM = Hrotate_x(Hrotate_z(Htranslation(np.eye(4), [0.7, 0.7, -0.3]), -np.pi/2), -np.pi/2)
# WEST_WALL_TRANFORM = Hrotate_x(Hrotate_z(Htranslation(np.eye(4), [-0.7, -0.7, -0.3]), np.pi/2), -np.pi/2)


AllMotorNames = ["Bace Motor", "Sholder Motor", "Elbow Motor", "Elbow revolut Motor", "Wirist Motor", "Wirist revolut Motor"]

ERRORmsg = [
    "Get the list of parts to fetch from the database",
    "Get the part info from the database and organize it in a dictionary",
    "Get the locations and orientations of all parts",
    "Get the paths and Velocity profile for each path"
]

class RobotCart(tk.Tk):
    """Main application class representing a robot control interface.

    Args:
        tk (module): The Tkinter module.
    """
    @error_handling_wrapper
    def __init__(self, *args, **kwargs):
        """Initialize the RobotCart application.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        tk.Tk.__init__(self, *args, **kwargs)

        # Initialize attributes
        self.logged_in = False
        self.admin_page = 0

        # Create a container frame to hold the pages
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)

        # Configure the container's grid layout
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        # Dictionary to store different pages
        self.frames = {}

        # Create and add pages to the application
        for F in (MainUserPage, SecurityPage, MoshionPlanningPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        # Show the working page by default
        self.show_frame(MainUserPage)

        # Create a PartsDatabase instance and create the Parts table
        try:
            parts_db = PartsDatabase()
            parts_db.create_parts_table()
        except Exception as e:
            error_msg = f"Error connecting to the database: {str(e)}"
            self.popup_error(error_msg)

        # Create and configure the menu bar
        menuBar = tk.Menu(container)
        settingMenu = tk.Menu(menuBar, tearoff=0)
        settingMenu.add_command(label='Motor Set Up', command=lambda: self.admin_control(0))
        settingMenu.add_command(label='Database Config', command=lambda: self.admin_control(1))
        settingMenu.add_separator()
        settingMenu.add_command(label='Primary Color', command=self.primary_color)
        settingMenu.add_command(label='Secondary Color', command=self.secondary_color)
        settingMenu.add_command(label='Highlight Color', command=self.highlight_color)
        settingMenu.add_command(label='Low Weight', command=self.low_weight_color)
        menuBar.add_cascade(label="Menu", menu=settingMenu)

        try:
            tk.Tk.config(self, menu=menuBar)
        except Exception as e:
            self.popup_error(f"Error configuring menu: {str(e)}")

        # Configure the style for Treeview widgets
        style = ttk.Style()
        style.theme_use('default')

        # Specify your tag configurations here
        style.configure("OddRow.TTreeview", background='white')
        style.configure("EvenRow.TTreeview", background='lightblue')
        style.configure("LowWeight.TTreeview", background='lightcoral')
        # Add more tag configurations as needed

        try:
            tk.Tk.config(self, menu=menuBar)
        except Exception as e:
            self.popup_error(f"Error configuring menu: {str(e)}")

    def show_frame(self, cont: tk.Frame, cart_data=None) -> None:
        """Show the specified frame.

        Args:
            cont (tk.Frame): The frame to be shown.
            cart_data (Any, optional): Additional data for the frame. Defaults to None.
        """
        frame = self.frames.get(cont)

        if frame:
            if hasattr(frame, 'set_cart_data'):
                frame.set_cart_data(cart_data)

            frame.tkraise()
        else:
            self.popup_error(f"Error: Frame {cont} not found.")

    def admin_control(self, page_num) -> None:
        """Control the admin page.

        Args:
            page_num (int): The admin page number.
        """
        self.admin_page = page_num

        if not self.logged_in:
            self.show_frame(SecurityPage)
        # else:
        #     if self.admin_page == 0:
        #         self.show_frame(MotorSetUpPage)
        #     elif self.admin_page == 1:
        #         self.show_frame(DataBacePannle)

    def popup_error(self, msg: str) -> None:
        """Show an error popup with the specified message.

        Args:
            msg (str): The error message.
        """
        error_popup = tk.Toplevel(self)
        error_popup.title("Error")

        # Create a frame to hold the error message and button
        frame = ttk.Frame(error_popup)
        frame.pack(padx=20, pady=20)

        # Error message label
        label = ttk.Label(frame, text=msg, font=("Helvetica", 12), wraplength=300)
        label.grid(row=0, column=0, padx=10, pady=10)

        # Okay button
        okay_button = ttk.Button(frame, text="Okay", command=error_popup.destroy)
        okay_button.grid(row=1, column=0, pady=10)

        # Center the error popup on the screen
        error_popup.geometry(f"+{self.winfo_screenwidth() // 2 - 150}+{self.winfo_screenheight() // 2 - 100}")

        # Make the error popup a transient window, preventing interaction with the main window until closed
        error_popup.transient(self)
        error_popup.grab_set()

        # Wait for the error popup to be closed before returning
        self.wait_window(error_popup)

    @error_handling_wrapper
    def primary_color(self) -> None:
        """Choose and set the primary color for the interface."""
        primary_color = colorchooser.askcolor()[1]
        if primary_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                current_frame.parts_treeview.tag_configure('OddRow.TTreeview', background=primary_color)
            else:
                self.popup_error("Error: MainUserPage frame not found.")

    @error_handling_wrapper
    def secondary_color(self) -> None:
        """Choose and set the secondary color for the interface."""
        secondary_color = colorchooser.askcolor()[1]
        if secondary_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                current_frame.parts_treeview.tag_configure('EvenRow.TTreeview', background=secondary_color)
            else:
                self.popup_error("Error: MainUserPage frame not found.")
        
    @error_handling_wrapper
    def low_weight_color(self) -> None:
        """Choose and set the color for low-weight items in the interface."""
        low_weight_color = colorchooser.askcolor()[1]
        if low_weight_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                current_frame.parts_treeview.tag_configure('LowWeight.TTreeview', background=low_weight_color)
            else:
                self.popup_error("Error: MainUserPage frame not found.")

    @error_handling_wrapper
    def highlight_color(self) -> None:
        """Choose and set the highlight color for the interface."""
        highlight_color = colorchooser.askcolor()[1]
        if highlight_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                style = ttk.Style()
                style.map("OddRow.TTreeview", background=[('selected', highlight_color)])
                style.map("EvenRow.TTreeview", background=[('selected', highlight_color)])
            else:
                self.popup_error("Error: MainUserPage frame not found.")

class PageBase(tk.Frame):
    """Base class for pages in the RobotCart application.

    Args:
        tk (module): The Tkinter module.
    """
    def __init__(self, parent: tk.Widget, controller: RobotCart, title: str):
        """Initialize the PageBase.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
            title (str): The title of the page.
        """
        super().__init__(parent)

        # Title label
        label = tk.Label(self, text=title, font=LARGE_FONT)
        label.grid(row=0, column=0, pady=10, padx=10)

        # Content frame
        self.content_frame = tk.Frame(self)
        self.content_frame.grid(row=1, column=0, sticky="nsew")

        # Configure row and column weights to allow expansion
        # self.grid_rowconfigure(1, weight=1)
        # self.grid_columnconfigure(0, weight=1)

    def popup_error(self, msg: str) -> None:
        """Display an error popup with the specified message.

        Args:
            msg (str): The error message.
        """
        error_popup = tk.Toplevel(self)
        error_popup.title("Error")

        # Create a frame to hold the error message and button
        frame = ttk.Frame(error_popup)
        frame.pack(padx=20, pady=20)

        # Error message label
        label = ttk.Label(frame, text=msg, font=("Helvetica", 12), wraplength=300)
        label.grid(row=0, column=0, padx=10, pady=10)

        # Okay button
        okay_button = ttk.Button(frame, text="Okay", command=error_popup.destroy)
        okay_button.grid(row=1, column=0, pady=10)

        # Center the error popup on the screen
        error_popup.geometry(f"+{self.winfo_screenwidth() // 2 - 150}+{self.winfo_screenheight() // 2 - 100}")

        # Make the error popup a transient window, preventing interaction with the main window until closed
        error_popup.transient(self)
        error_popup.grab_set()

        # Wait for the error popup to be closed before returning
        self.wait_window(error_popup)

class SecurityPage(PageBase):
    """Page for handling user authentication.

    Args:
        PageBase (class): The base class for pages.

    Raises:
        ValueError: Raised when username or password is empty.
    """
    @error_handling_wrapper
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the SecurityPage.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
        """
        super().__init__(parent, controller, "Security")

        # Username entry
        self.username_label = tk.Label(self.content_frame, text="Username")
        self.username_label.grid(row=0, column=0, padx=10, pady=10)
        self.username_var = tk.StringVar()
        self.username_entry = tk.Entry(self.content_frame, textvariable=self.username_var)
        self.username_entry.grid(row=0, column=1, padx=10, pady=10)

        # Password entry
        self.password_label = tk.Label(self.content_frame, text="Password")
        self.password_label.grid(row=1, column=0, padx=10, pady=10)
        self.password_var = tk.StringVar()
        self.password_entry = tk.Entry(self.content_frame, textvariable=self.password_var, show="*")
        self.password_entry.grid(row=1, column=1, padx=10, pady=10)

        # Login button
        self.login_button = tk.Button(self.content_frame, text="Login", command=lambda: self.check_credentials(controller))
        self.login_button.grid(row=2, column=0, columnspan=2, pady=10)

    # @error_handling_wrapper
    # def check_credentials(self, controller):
    #     """Check the entered credentials and perform login.

    #     Args:
    #         controller (RobotCart): The main application controller.

    #     Raises:
    #         ValueError: Raised when username or password is empty.
    #     """
    #     # Input validation: Check if username and password are not empty
    #     entered_username = self.username_var.get().strip()
    #     entered_password = self.password_var.get().strip()

    #     if not entered_username or not entered_password:
    #         raise ValueError("Username and password cannot be empty")

    #     # For simplicity, using hardcoded username and password
    #     correct_username = "admin"
    #     correct_password = "admin123"

    #     if entered_username == correct_username and entered_password == correct_password:
    #         # If the credentials are correct, set the login status to True
    #         controller.logged_in = True

    #         if controller.admin_page == 0:
    #             controller.show_frame(MotorSetUpPage)

    #         elif controller.admin_page == 1:
    #             controller.show_frame(DataBacePannle)
        # else:
        #     # If the credentials are incorrect, show a message
        #     messagebox.showerror("Login Failed", "Incorrect username or password")

class MainUserPage(PageBase):
    """Page for selecting and managing parts.

    Args:
        PageBase (class): The base class for pages.
    """
    
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the MainUserPage.

        Args:
            parent (type): The parent widget.
            controller (type): The main application controller.
        """
        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, "Part Selection Page")

        box_w = 0.2
        box_h = 0.25
        grid_size = (1.40, 1.40)

        # Create a PartsDatabase instance for handling parts data
        self.db_path = 'C:\\Users\\zachl\\Capstone2024\\parts_db'
        self.parts_db = PartsDatabase(db_name="parts_db", grid_size = grid_size, shelf_height=0.015, margin=0.015, offset_x=(box_w / 2), offset_y=(box_h / 2))
        self.parts_db.create_parts_table()

        # List to store selected parts in the cart
        self.cart: list[str] = []

        # Create a frame to hold the main treeview for available parts
        self.parts_treeview_frame = tk.Frame(self.content_frame)
        self.parts_treeview_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create a frame to hold the cart treeview
        self.cart_tree_frame = tk.Frame(self.content_frame)
        self.cart_tree_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Configure the main treeview for available parts
        self.configure_parts_db_treeview()

        # Create buttons for interaction with available parts
        self.create_buttons(controller)

        # Create a second treeview for the selected parts (cart)
        self.configure_cart_treeview()

    def configure_parts_db_treeview(self) -> None:
        """Configure the Treeview widget for displaying available parts.

        This method sets up the style, structure, and data loading for the Treeview.

        """
        # Set up the style for the Treeview
        style = ttk.Style()
        style.theme_use('default')
        style.configure("treeview", background="#D3D3D3", foreground="black", rowheight=25, fieldbackground="#D3D3D3")
        style.map("treeview", background=[('selected', "#347083")])

        # Create a frame to hold the Treeview
        tree_frame = tk.Frame(self.content_frame)
        tree_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create a vertical scrollbar for the Treeview
        tree_scroll = tk.Scrollbar(tree_frame)
        tree_scroll.pack(side='right', fill='y')

        # Create the Treeview widget with vertical scrollbar
        self.parts_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended", height=20)
        self.parts_treeview.pack()

        # Configure the scrollbar to control the Treeview's vertical movement
        tree_scroll.config(command=self.parts_treeview.yview)

        # Define columns and headings for the Treeview
        columns = ["Part Name", "Part ID", "In Service", "Current Weight"]
        headings = ["Name", "ID", "InService", "CurrentWeight"]

        # Set the columns for the Treeview
        self.parts_treeview['columns'] = columns
        self.parts_treeview.column("#0", width=0, stretch=0)

        # Configure column widths and anchor points
        for col in columns:
            if col in ("Part Name", "Part ID"):
                self.parts_treeview.column(col, anchor="w", width=140)
            else:
                self.parts_treeview.column(col, anchor="center", width=140)

        # Set column headings and anchor points
        self.parts_treeview.heading("#0", text='', anchor="w")
        for col, heading in zip(columns, headings):
            self.parts_treeview.heading(col, text=heading, anchor="center")

        # Load data into the Treeview
        self.load_data()

    def load_data(self) -> None:
        """Load data into the Treeview widget.

        This method retrieves data from the Parts database, configures tags for styling,
        sorts the data alphabetically, and inserts it into the Treeview.

        """
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select all records from the Parts table and order by the PartName column
            self.parts_db.cursor.execute("SELECT * FROM Parts ORDER BY Name")  

            # Fetch all the data from the executed query
            data = self.parts_db.cursor.fetchall()

            # Configure tags for alternate row colors and low weight indication
            self.parts_treeview.tag_configure('oddrow', background='white')
            self.parts_treeview.tag_configure('evenrow', background='lightblue')
            self.parts_treeview.tag_configure('lowweight', background='lightcoral') 

            # Clear existing content in the parts_treeview
            self.parts_treeview.delete(*self.parts_treeview.get_children())

            # Iterate through the retrieved data and insert it into the parts_treeview
            for idx, record in enumerate(data):
                # Determine the tag for alternate row coloring
                tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
                # Determine the service status for display
                service = "In Service" if record[8] == 1 else " "

                if record[7] < record[6]:  
                    tag = 'lowweight'

                # Insert the data into the parts_treeview with specified tags
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[0], service, record[7]), tags=(tag,))

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

    def create_buttons(self, controller: RobotCart) -> None:
        """Create buttons for user interaction.

        This method creates buttons for refreshing data, adding parts to the cart,
        placing an order, and searching data.

        """
        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.content_frame)
        button_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky="nsew")

        # Create a button to refresh the data
        refresh_button = tk.Button(button_frame, text="Refresh Data", command=self.load_data, bg='#4CAF50')
        refresh_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a button to search the data
        search_button = tk.Button(button_frame, text="Search Data", command=self.search_data, bg='#2196F3')
        search_button.grid(row=0, column=1, padx=10, pady=10)

        # Create a button to add selected parts to the cart
        add_to_cart_button = tk.Button(button_frame, text="Add to Cart", command=self.select_part, bg='#FFEB3B')
        add_to_cart_button.grid(row=0, column=2, padx=10, pady=10)

        # Create a button to place an order with selected parts
        place_order_button = tk.Button(button_frame, text="Place Order", command=lambda: self.place_order(controller), bg='#FF5722')
        place_order_button.grid(row=0, column=3, padx=10, pady=10)

    def select_part(self) -> None:
        """Select a part and add it to the cart."""
        
        # Method to add a selected part to the cart
        selected = self.parts_treeview.focus()

        # Check if a part is selected before attempting to retrieve its values
        if selected:
            # Retrieve part_name and in_service status from the selected part
            part_name = self.parts_treeview.item(selected, "values")[0]
            in_service = self.parts_treeview.item(selected, "values")[2]

            if in_service == "In Service":
                # If the part is "In Service," display a message to the user
                error_msg = "This part is currently in service and cannot be added to the cart."
                self.popup_error(error_msg)
            else:
                # If the part is not "In Service," add it to the cart
                if part_name not in self.cart:
                    self.cart.append(part_name)
                    # Update the cart treeview
                    self.update_cart_tree()

        else:
            # If no part is selected, show an error message or handle it as needed
            error_msg = "No part selected. Please select a part before adding it to the cart."
            self.popup_error(error_msg)

    def update_cart_tree(self)-> None:
        """Update the cart treeview with the current content of the cart.

        This method clears the existing content in the cart treeview and inserts the current
        parts in the cart for display.

        """

        # Clear existing content in the cart treeview
        self.cart_tree.delete(*self.cart_tree.get_children())

        # Iterate through the parts in the cart and insert them into the cart treeview
        for idx, part in enumerate(self.cart):
            # Determine the tag for alternate row coloring
            tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
            # Insert the part into the cart treeview with specified tag
            self.cart_tree.insert(parent='', index='end', iid=idx, text='', values=(part,), tags=(tag,))

    def place_order(self, controller: RobotCart) -> None:
        """Place an order for the selected parts in the cart.

        Args:
            controller (RobotCart): The main application controller.
        """
    
        # Check if the cart is empty before proceeding with the order
        if not self.cart:
            error_msg = "Cannot place an order with an empty cart. Please add parts to the cart first."
            self.popup_error(error_msg)
            return

        # Assuming you have logic here to process the order using the selected parts in self.cart

        # Show the Motion Planning Page with cart data
        controller.show_frame(MoshionPlanningPage, cart_data=self.cart)

        # Clear the cart after placing the order
        self.cart = []

        # Update the cart treeview
        self.update_cart_tree()

    def configure_cart_treeview(self) -> None:
        """Configure the Treeview widget for displaying selected parts in the cart.

        This method sets up the style, structure, and appearance of the cart Treeview.

        """
        
        # Configure treeview style for the cart
        style = ttk.Style()
        style.theme_use('default')
        style.configure("cart.Treeview", background="#D3D3D3", foreground="black", rowheight=25, fieldbackground="#D3D3D3")
        style.map("cart.Treeview", background=[('selected', "#347083")])

        # Configure cart tree scroll
        cart_tree_scroll = tk.Scrollbar(self.cart_tree_frame)
        cart_tree_scroll.pack(side='right', fill='y')

        # Create cart treeview widget
        self.cart_tree = ttk.Treeview(self.cart_tree_frame, yscrollcommand=cart_tree_scroll.set, selectmode="extended", style="cart.Treeview")
        self.cart_tree.pack(fill='both', expand=True)

        # Configure scrollbar to control cart treeview's vertical movement
        cart_tree_scroll.config(command=self.cart_tree.yview)

        # Set columns for the cart treeview
        self.cart_tree['columns'] = ("#0",)

        # Configure column widths and anchor points (optional if you decide to keep it)
        self.cart_tree.column("#0", anchor="w", width=140)

        # Set column headings and anchor points (optional if you decide to keep it)
        self.cart_tree.heading("#0", text="Selected Part", anchor="center")  # Change "#0" to the actual column ID if necessary

    def search_data(self) -> None:
        """Open a search window to look up parts."""
        
        # Create a new Toplevel window for the search functionality
        search_window = tk.Toplevel(self)
        search_window.title("Lookup Parts")
        search_window.geometry("400x200")

        # Create a reference to the search window to use in the closing event
        self.search_window = search_window

        # Create a labeled frame for the part name search
        search_frame = tk.LabelFrame(search_window, text="Part Name")
        search_frame.pack(padx=10, pady=10)

        # Entry widget for entering the part name and bind KeyRelease event for auto-fill suggestions
        search_entry = tk.Entry(search_frame)
        search_entry.pack(padx=20, pady=20)
        search_entry.bind('<KeyRelease>', lambda event, entry=search_entry: self.auto_fill_suggestions(entry))

        # Bind the closing event of the search window to the restore_treeview method
        search_window.protocol("WM_DELETE_WINDOW", self.restore_treeview)

    def auto_fill_suggestions(self, entry: tk.Widget) -> None:
    
        """Auto-fill suggestions based on the current content of the entry."""
        # Get the current content of the entry
        current_text = entry.get()

        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())

        # Load suggestions based on the current_text
        self.load_suggestions(current_text)

    def load_suggestions(self, prefix: str) -> None:
        """Load suggestions based on the provided part name or part ID prefix.

        Args:
            prefix (str): The prefix used for searching part names or part IDs.
        """
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select records from the Parts table with a matching part name or part ID
            query = "SELECT * FROM Parts WHERE Name LIKE ? OR ID LIKE ?"
            self.parts_db.cursor.execute(query, ('%' + prefix + '%', '%' + prefix + '%'))

            # Fetch all the data from the executed query
            data = self.parts_db.cursor.fetchall()

            # Configure tags for alternate row colors and low weight indication
            self.parts_treeview.tag_configure('oddrow', background='white')
            self.parts_treeview.tag_configure('evenrow', background='lightblue')
            self.parts_treeview.tag_configure('lowweight', background='lightcoral')  # Add a tag for low weight

            # Clear existing content in the parts_treeview
            self.parts_treeview.delete(*self.parts_treeview.get_children())

            # Iterate through the retrieved data and insert it into the parts_treeview
            for idx, record in enumerate(data):
                # Determine the tag for alternate row coloring
                tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
                # Determine the service status for display
                service = "In Service" if record[8] == 1 else " "

                # Check if CurrentWeight is below 5 and set a different tag for low weight
                if record[7] < record[6]:  
                    tag = 'lowweight'

                # Insert the data into the parts_treeview with specified tags
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[0], service, record[7]), tags=(tag,))

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

    def search_and_update_treeview(self, part_name: str) -> None:
        """Search for parts and update the parts_treeview based on the search."""
        self.serch_db_parts(part_name)

    def restore_treeview(self) -> None:
        """Restore the parts_treeview to show all parts."""
        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())
        
        # Load all parts into the parts_treeview
        self.load_data()

        # Destroy the search window
        if hasattr(self, 'search_window') and self.search_window:
            self.search_window.destroy()

    def serch_db_parts(self, partName: str) -> None:
        
        """Search for parts in the database with the given name and update the parts_treeview.

        Args:
            partName (str): The name of the part to search for.
        """
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select records from the Parts table with a matching part name
            query = "SELECT * FROM parts_db WHERE PartName LIKE ?"
            self.parts_db.cursor.execute(query, ('%' + partName + '%',))

            # Fetch all the data from the executed query
            data = self.parts_db.cursor.fetchall()

            # Clear existing content in the parts_treeview
            self.parts_treeview.delete(*self.parts_treeview.get_children())

            # Configure tags for alternate row colors and low weight indication
            self.parts_treeview.tag_configure('oddrow', background='white')
            self.parts_treeview.tag_configure('evenrow', background='lightblue')
            self.parts_treeview.tag_configure('lowweight', background='lightcoral')  # Add a tag for low weight

            # Iterate through the retrieved data and insert it into the parts_treeview
            for idx, record in enumerate(data):
                # Determine the tag for alternate row coloring
                tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
                # Determine the service status for display
                service = "In Service" if record[13] == 1 else " "

                # Check if CurrentWeight is below 5 and set a different tag for low weight
                if record[12] < record[10]:
                    tag = 'lowweight'

                # Insert the data into the parts_treeview with specified tags
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[2], service, record[12]), tags=(tag,))

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

class MoshionPlanningPage(PageBase):
    """Page for Motion Planning and Verification.

    Args:
        PageBase (type): The base class for pages.
    """
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the MoshionPlanningPage.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
        """
        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, "Motion Verification")

        box_w = 0.2
        box_h = 0.25
        grid_size = (1.40, 1.40)

        max_acc = 10000
        max_vel = 10000
        self.planner = PathPlanner(max_acc, max_vel)

        self.PathShow = False

        # Set the default size for figures in the canvas
        self.figsizes = (8, 6)

        # Additional attribute to store cart data
        self.cart_data = {}

        # Dictionary to store part information
        self.part_info_dict = {}

        # Dictionary to store locations of each part
        self.locations = {}  

        # Dictionary to store orientations of each part
        self.orientations = {}  

        # List to store part names to fetch
        self.part_names_to_fetch = []

        # list to stor the path of the robot
        self.travle_paths = []

        # list of the orientations alone the
        self.travle_orientation = []

        # what axix to fix alone the path
        self.travle_alinements = []

        # Create a PartsDatabase instance for handling parts data
        self.parts_db = PartsDatabase()

        # Create a pth panner for moshion planning
        self.planner = PathPlanner(max_acc, max_vel)

        #urdf_file_path = "E:\\Capstone\\app\\backend\\python code\\urdf_tes2.urdf"
        urdf_file_path = "C:\\Users\\zachl\\Capstone2024\\app\\backend\\python code\\urdf_tes2.urdf"
        #urdf_file_path = "//home//zachl//Capstone//app/backend//python code//urdf_tes2.urdf"
        self.robot = RobotArm(urdf_file_path, IDLE_AGLE_POSITION)

        # Create a PartsDatabase instance for handling parts data
        self.db_path = 'C:\\Users\\zachl\\Capstone2024\\parts_db'
        self.parts_db = PartsDatabase(db_name="parts_db", grid_size = grid_size, shelf_height=0.015, margin=0.015, offset_x=(box_w / 2), offset_y=(box_h / 2))
        self.parts_db.create_parts_table()

        # Create buttons for interaction with available parts
        self.create_order_buttons(controller)

        self.configure_order_treeview()

        self.parts_return_entry_boxes()

        self.create_return_buttons(controller)

        self.parts_treeview.bind("<ButtonRelease-1>", lambda event: self.select_part())

    def create_order_buttons(self, controller: RobotCart) -> None:
        """Create buttons for motion planning.

        Args:
            controller (RobotCart): The main application controller.
        """

        # Create a frame to hold the buttons
        button_frame = tk.LabelFrame(self.content_frame, text="Sim controll")  # Use 'self.content_frame' instead of 'self'
        button_frame.grid(row=1, column=0, pady=10, sticky="nsew")  

        # Create a button to start the simulation
        start_sim_button = tk.Button(button_frame, text="Start Sim", command=self.get_select_part)
        start_sim_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a button to confirm the path
        confirm_path_button = tk.Button(button_frame, text="Confirm Path", command = self.animate_robot)
        confirm_path_button.grid(row=1, column=0, padx=10, pady=10)

        # Create a button to confirm the path
        confirm_path_button = tk.Button(button_frame, text="Confirm Sim", command = self.load_return_treeview)
        confirm_path_button.grid(row=2, column=0, padx=10, pady=10)

        # Button to return to Part Selection
        return_button = tk.Button(button_frame, text="Reaturn to Part Selection", command=lambda: controller.show_frame(MainUserPage))
        return_button.grid(row=3, column=0, padx=10, pady=10, sticky="nw")

    def parts_return_entry_boxes(self) -> None:
        """Create entry boxes for paty data editing."""
        # Create a frame to hold the entry boxes
        self.entry_frame = tk.LabelFrame(self.content_frame, text="Part edit return data")
        self.entry_frame.grid(row=2, column=1,  pady=10, sticky="nsew")

        # Define the labels and corresponding entry variables

        self.labels = ["Part ID", "Part Name", "CurrentWeight"]
        
        self.entry_vars = [tk.StringVar() for _ in range(len(self.labels))]

        # Loop to create labels and entry boxes
        row, col = (0,0)
        for i, label in enumerate(self.labels):
            
            if ((i % 7) == 0):
                row += 2
                col = 0
            else:
                col += 1

            tk.Label(self.entry_frame, text=label).grid(row = row, column=col, padx=10, pady=10)
            tk.Entry(self.entry_frame, textvariable=self.entry_vars[i]).grid(row= row + 1, column=col, padx=10, pady=10)

        # Assign entry variables to class attributes for later access
        self.PartID, self.PartName, self.CurrentWeight = self.entry_vars

    def create_return_buttons(self, controller):
        """_summary_
        """
        # Calculate the number of rows required for entry boxes
        num_rows = 3

        # Button to return to Part Selection
        return_button = tk.Button(self.entry_frame, text="Return Sim", command=self.return_updaed_part)
        return_button.grid(row=1, column=0, padx=10, pady=10, sticky="nesw")

        # Button to return to Part Selection
        return_button = tk.Button(self.entry_frame, text="Confirm return Path", command=self.animate_robot)
        return_button.grid(row=1, column=1, padx=10, pady=10, sticky="nesw")

        # Button to return to Part Selection
        return_button = tk.Button(self.entry_frame, text="Reaturn to Part Selection", command=lambda: self.return_to_part_select(controller))
        return_button.grid(row=1, column=2, padx=10, pady=10, sticky="nesw")

    def return_to_part_select(self, controller):
        
        self.planner.clear_saved_paths()

        # Clear existing content in the cart treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())

        controller.show_frame(MainUserPage)

    def configure_order_treeview(self) -> None:
        """Configure the Treeview widget for displaying available parts.

        This method sets up the style, structure, and data loading for the Treeview.

        """
        # Set up the style for the Treeview
        style = ttk.Style()
        style.theme_use('default')
        style.configure("treeview", background="#D3D3D3", foreground="black", rowheight=25, fieldbackground="#D3D3D3")
        style.map("treeview", background=[('selected', "#347083")])

        # Create a frame to hold the Treeview
        tree_frame = tk.Frame(self.content_frame)
        tree_frame.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        # Create a vertical scrollbar for the Treeview
        tree_scroll = tk.Scrollbar(tree_frame)
        tree_scroll.pack(side='right', fill='y')

        # Create the Treeview widget with vertical scrollbar
        self.parts_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended", height=20)
        self.parts_treeview.pack()

        # Configure the scrollbar to control the Treeview's vertical movement
        tree_scroll.config(command=self.parts_treeview.yview)

        # Define columns and headings for the Treeview
        columns = ["Part Name", "Part ID", "Current Weight"]
        headings = ["Name", "ID", "CurrentWeight"]

        # Set the columns for the Treeview
        self.parts_treeview['columns'] = columns
        self.parts_treeview.column("#0", width=0, stretch=0)

        # Configure column widths and anchor points
        for col in columns:
            if col in ("Part Name", "Part ID"):
                self.parts_treeview.column(col, anchor="w", width=140)
            else:
                self.parts_treeview.column(col, anchor="center", width=140)

        # Set column headings and anchor points
        self.parts_treeview.heading("#0", text='', anchor="w")
        for col, heading in zip(columns, headings):
            self.parts_treeview.heading(col, text=heading, anchor="center")

    def load_return_treeview(self):
        # Initialize idx
        idx = 0

        # Configure tags for alternate row colors and low weight indication
        self.parts_treeview.tag_configure('oddrow', background='white')
        self.parts_treeview.tag_configure('evenrow', background='lightblue')
        self.parts_treeview.tag_configure('lowweight', background='lightcoral') 

        # Iterate through the retrieved data and insert it into the parts_treeview
        for part_name, part_info in self.part_info_dict.items():
            # Determine the tag for alternate row coloring
            tag = 'evenrow' if idx % 2 == 0 else 'oddrow'

            if part_info['EmptyWeight'] < part_info['HalfWeight']:  
                tag = 'lowweight'

            # Insert the data into the parts_treeview with specified tags
            # Assuming parts_treeview is your treeview widget
            self.parts_treeview.insert('', 'end', text=part_name, values=(
                part_info['PartName'],
                part_info['ID'],
                part_info['CurrentWeight']
            ), tags=(tag,))

            # Increment idx
            idx += 1

    def select_part(self):
        # Method to add a selected part to the cart
        selected = self.parts_treeview.focus()

        # Check if a part is selected before attempting to retrieve its values
        if selected:
            # Retrieve part_name and in_service status from the selected part
            part_name = self.parts_treeview.item(selected, "values")[0]
            Part_ID = self.parts_treeview.item(selected, "values")[1]
            Part_weight = self.parts_treeview.item(selected, "values")[2]

            self.PartID.set(Part_ID)
            self.PartName.set(part_name)
            self.CurrentWeight.set(Part_weight)

    def return_updaed_part(self):
        entry_data = []
        for var in self.entry_vars:
            entry_data.append(var.get())

        print(entry_data)
        if self.parts_db.update_part_by_name(entry_data[1], CurrentWeight = entry_data[2]):
            print("Part updated successfully!")
        else:
            print("Failed to update part.")

        self.generate_return_robot_path()

    def generate_return_robot_path(self):
        """Generate paths and velocity profiles for each part."""

        depbug_stage = 14
        self.travle_paths = []
        self.travle_orientation = []
        travle_alinements = []
        travle_stop = []
        over_wrap = False
        self.planner.clear_saved_paths()

        # leav the idle position and 
        if depbug_stage > 0:
            T0 = self.planner.generate_path(IDLE_POSITION, WORKING_POSITION, linear=True)
            T0_orientation = [IDLE_ORIENTATION for _ in range(len(T0))]

            self.travle_paths.append(T0)
            self.travle_orientation.append(T0_orientation)

        # go to the drop off zone
        if depbug_stage > 1:
            T0 = self.planner.generate_path(WORKING_POSITION, JOGGING_START, linear=False)
            T0_orientation = [T0_orientation[-1] for _ in range(len(T0))]

            self.travle_paths.append(T0)
            self.travle_orientation.append(T0_orientation)

        for index, (location, orientation) in enumerate(zip(self.locations.values(), self.orientations.values())):
            over_wrap = False
            location = np.array(location) + Gripper_offset + [0, -RROTASHION_BUFFER_DIS, 0]

            if depbug_stage > 2:
                Pickup_point = np.dot(quaternion_to_rotation_matrix(DROP_OFF_ORIENTATION), (0, 0, -insershion_distance)) + DROP_OFF_ZONE
                T1 = self.planner.generate_path(JOGGING_START, Pickup_point, linear=False)
                T1_orientation = [quaternion_slerp(T0_orientation[-1], DROP_OFF_ORIENTATION, t) for t in np.linspace(0, 1, len(T1), endpoint=True)]
                self.travle_paths.append(T1)
                self.travle_orientation.append(T1_orientation)

            if depbug_stage > 3:
                T2 = self.planner.generate_path(T1[-1], DROP_OFF_ZONE, linear=True)
                T2_orientation = [T1_orientation[-1] for _ in range(len(T2))]
                self.travle_paths.append(T2)
                self.travle_orientation.append(T2_orientation)

            if depbug_stage > 4:
                Pickup_point = np.dot(quaternion_to_rotation_matrix(DROP_OFF_ORIENTATION), (0, -Hight_drop_off_box, 0)) + DROP_OFF_ZONE
                T3 = self.planner.generate_path(T2[-1], Pickup_point, linear=True)
                T3_orientation = [T2_orientation[-1] for _ in range(len(T3))]
                self.travle_paths.append(T3)
                self.travle_orientation.append(T3_orientation)

            if depbug_stage > 5:
                T4 = self.planner.generate_path(T3[-1], WEIGHTING_POSITION, linear=True)
                T4_orientation = [WEIGHTING_ORIENTATION for _ in range(len(T4))]
                self.travle_paths.append(T4)
                self.travle_orientation.append(T4_orientation)
                travle_stop.append(5)


            if depbug_stage > 6:
                Pickup_point = np.dot(quaternion_to_rotation_matrix(DROP_OFF_ORIENTATION), (0, -Hight_drop_off_box, 0)) + WEIGHTING_POSITION
                T5 = self.planner.generate_path(T4[-1], Pickup_point, linear=True)
                T5_orientation = [WEIGHTING_ORIENTATION for _ in range(len(T5))]
                self.travle_paths.append(T5)
                self.travle_orientation.append(T5_orientation)


            if depbug_stage > 7:
                T6 = self.planner.generate_path(T5[-1], location, linear=False)

                delta_angles = XY_angle(T5[-1], location)
                if (-np.pi/2 > delta_angles >= -np.pi):
                    delta_angles =  np.linspace(0, np.pi, len(T6), endpoint=True)
                elif (0 > delta_angles >= -np.pi/2):
                    delta_angles =  np.linspace(0, np.pi/2, len(T6), endpoint=True)
                elif (0 < delta_angles <= np.pi/2):
                    delta_angles =  np.linspace(0, -np.pi/2, len(T6), endpoint=True)
                elif (np.pi/2 < delta_angles <= np.pi):
                    over_wrap = True
                    delta_angles =  np.linspace(0, -np.pi, len(T6), endpoint=True)
                
                T6_orientation = [rotate_quaternion(T5_orientation[-1], 0, rad, 0) for rad in delta_angles]
                self.travle_paths.append(T6)
                self.travle_orientation.append(T6_orientation)

            if depbug_stage > 8:
                insershion_point = np.dot(quaternion_to_rotation_matrix(T6_orientation[-1]), (0, 0, insershion_distance)) + location
                T7 = self.planner.generate_path(location, insershion_point, linear=True)
                T7_orientation =  [T6_orientation[-1] for _ in range(len(T7))]
                self.travle_paths.append(T7)
                self.travle_orientation.append(T7_orientation)

            if depbug_stage > 9:
                lifing_point = np.dot(quaternion_to_rotation_matrix(T7_orientation[-1]), (0, lifting_distance, 0)) + T7[-1]
                T8 = self.planner.generate_path(T7[-1], lifing_point, linear=True)
                T8_orientation = [T7_orientation[-1] for _ in range(len(T8))]
                self.travle_paths.append(T8)
                self.travle_orientation.append(T8_orientation)

            if depbug_stage > 10:
                retracked_point = np.dot(quaternion_to_rotation_matrix(T8_orientation[-1]), (0, 0, -insershion_distance)) + T8[-1]
                T9 = self.planner.generate_path(T8[-1], retracked_point, linear=True)
                T9_orientation = [T8_orientation[-1] for _ in range(len(T9))]
                self.travle_paths.append(T9)
                self.travle_orientation.append(T9_orientation)

            if over_wrap:
                TOW_MID_point = np.dot(quaternion_to_rotation_matrix(DROP_OFF_ORIENTATION), (0, -Hight_drop_off_box, 0)) + WEIGHTING_POSITION
                TOW = self.planner.generate_path(T9[-1], TOW_MID_point, linear=False)
                TOW_orientation = [rotate_quaternion(T9_orientation[-1], 0, -rad, 0) for rad in delta_angles]
                self.travle_paths.append(TOW)
                self.travle_orientation.append(TOW_orientation)

                if depbug_stage > 11:
                    T10= self.planner.generate_path(TOW[-1], JOGGING_START, linear=False)
                    T10_orientation = [quaternion_slerp(TOW_orientation[-1], IDLE_ORIENTATION, t) for t in np.linspace(0, 1, len(T10), endpoint=True)]
                    self.travle_paths.append(T10)
                    self.travle_orientation.append(T10_orientation)
      
            else:

                if depbug_stage > 11:
                    T10= self.planner.generate_path(T9[-1], JOGGING_START, linear=False)
                    T10_orientation = [quaternion_slerp(T9_orientation[-1], IDLE_ORIENTATION, t) for t in np.linspace(0, 1, len(T10), endpoint=True)]
                    self.travle_paths.append(T10)
                    self.travle_orientation.append(T10_orientation)

        if depbug_stage > 12:
            T11 =  self.planner.generate_path(JOGGING_START, WORKING_POSITION, linear=False)
            T11_orientation = [T10_orientation[-1] for _ in range(len(T11))]
            self.travle_paths.append(T11)
            self.travle_orientation.append(T11_orientation)

        if depbug_stage > 13:
            T12 =  self.planner.generate_path(WORKING_POSITION, IDLE_POSITION, linear=True)
            T12_orientation = [T11_orientation[-1] for _ in range(len(T12))]
            self.travle_paths.append(T12)
            self.travle_orientation.append(T12_orientation)

        # Plot the 3D paths
        self.show_path()

    def set_cart_data(self, cart_data):
        """Set the cart data for motion planning.

        Args:
            cart_data (list): A list of part names to be fetched for the motion planning.
        """
        # Setter method to update cart data
        self.part_names_to_fetch = cart_data

    def get_select_part(self):
        """Retrieve and store information for specified parts.

        Fetch information for each part in the cart from the PartsDatabase and store it in the part_info_dict.
        """
        
        # Retrieve and store information for specified parts
        for part_name_to_find in self.part_names_to_fetch:
            part = self.parts_db.get_part_by_name(part_name_to_find)
            if part:
                # Construct a dictionary with part information
                self.part_info_dict[part_name_to_find] = {
                    'ID': part.ID,
                    'PartName': part.name,
                    'LocationX': ast.literal_eval(part.position)[0],
                    'LocationY': ast.literal_eval(part.position)[1],
                    'LocationZ': ast.literal_eval(part.position)[2],
                    'Orientation': ast.literal_eval(part.orientation),
                    'FullWeight': part.FullWeight,
                    'HalfWeight': part.HalfWeight,
                    'EmptyWeight': part.EmptyWeight,
                    'CurrentWeight': part.CurrentWeight,
                    'InService': part.InService
                }
        self.get_part_locations()

    def get_part_locations(self):
        """Retrieve part locations from part_info_dict.

        Iterate through part_info_dict and extract the location information for each part, storing it in the locations dictionary.
        """

        for part_name, part_info in self.part_info_dict.items():
            location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
            orientation = tuple(part_info['Orientation'])
            self.locations[part_name] = location
            self.orientations[part_name] = orientation

        # Proceed to generate the robot path using the collected part locations
        self.generate_robot_path()

    def generate_robot_path(self):
        """Generate paths and velocity profiles for each part."""

        depbug_stage = 14
        self.travle_paths = []
        self.travle_orientation = []
        travle_alinements = []
        travle_stop = []
        over_wrap = False
        self.planner.clear_saved_paths()

        # leav the idle position and 
        if depbug_stage > 0:
            T0 = self.planner.generate_path(IDLE_POSITION, WORKING_POSITION, linear=True)
            T0_orientation = [IDLE_ORIENTATION for _ in range(len(T0))]

            self.travle_paths.append(T0)
            self.travle_orientation.append(T0_orientation)

        # go to the drop off zone
        if depbug_stage > 1:
            T0 = self.planner.generate_path(WORKING_POSITION, JOGGING_START, linear=False)
            T0_orientation = [T0_orientation[-1] for _ in range(len(T0))]

            self.travle_paths.append(T0)
            self.travle_orientation.append(T0_orientation)

        for index, (location, orientation) in enumerate(zip(self.locations.values(), self.orientations.values())):
            over_wrap = False
            location = np.array(location) + Gripper_offset + [0, -RROTASHION_BUFFER_DIS, 0]

            if depbug_stage > 2:
                T1 = self.planner.generate_path(JOGGING_START, location, linear=False)

                delta_angles = XY_angle(JOGGING_START, location)
                if (-np.pi/2 > delta_angles >= -np.pi):
                    delta_angles =  np.linspace(0, np.pi/2, len(T1))
                    over_wrap = True
                elif (0 > delta_angles >= -np.pi/2):
                    delta_angles =  np.linspace(0, np.pi/2, len(T1))
                elif (0 < delta_angles <= np.pi/2):
                    delta_angles =  np.linspace(0, -np.pi/2, len(T1))
                elif (np.pi/2 < delta_angles <= np.pi):
                    delta_angles =  np.linspace(0, -np.pi, len(T1))

                T1_orientation = [rotate_quaternion(T0_orientation[-1], rad, 0, 0) for rad in delta_angles]
                insershion_point = np.dot(quaternion_to_rotation_matrix(T1_orientation[-1]), (0, 0, insershion_distance)) + location
                self.travle_paths.append(T1)
                self.travle_orientation.append(T1_orientation)

            if depbug_stage > 3:
                T2 = self.planner.generate_path(T1[-1], insershion_point, linear=True)
                T2_orientation = [T1_orientation[-1] for _ in range(len(T2))]
                self.travle_paths.append(T2)
                self.travle_orientation.append(T2_orientation)

            if depbug_stage > 4:
                lifing_point = np.dot(quaternion_to_rotation_matrix(T2_orientation[-1]), (-lifting_distance, 0, 0)) + T2[-1]
                T3 = self.planner.generate_path(insershion_point, lifing_point, linear=True)
                T3_orientation = [T2_orientation[-1] for _ in range(len(T3))]
                self.travle_paths.append(T3)
                self.travle_orientation.append(T3_orientation)

            if depbug_stage > 5:
                retracked_point = np.dot(quaternion_to_rotation_matrix(T3_orientation[-1]), (0, 0, -insershion_distance)) + T3[-1]
                T4 = self.planner.generate_path(lifing_point, retracked_point, linear=True)
                T4_orientation = [T3_orientation[-1] for _ in range(len(T4))]
                self.travle_paths.append(T4)
                self.travle_orientation.append(T4_orientation)

            if over_wrap:
                TOW = self.planner.generate_path(T4[-1], JOGGING_START, linear=False)
                TOW_orientation = [rotate_quaternion(T4_orientation[-1], 0, -rad, 0) for rad in delta_angles]
                self.travle_paths.append(TOW)
                self.travle_orientation.append(TOW_orientation)

                Drop_off_hight = np.dot(quaternion_to_rotation_matrix(T4_orientation[-1]), (-Hight_drop_off_box, 0, 0)) + DROP_OFF_ZONE
                T5 = self.planner.generate_path(TOW[-1], Drop_off_hight, linear=False)
                delta_angles = np.linspace(0, -np.pi/2, len(T5), endpoint=True)
                T5_orientation = [rotate_quaternion(TOW_orientation[-1], 0, rad, 0) for rad in delta_angles]
                self.travle_paths.append(T5)
                self.travle_orientation.append(T5_orientation)
            else:
                if depbug_stage > 6:
                    Drop_off_hight = np.dot(quaternion_to_rotation_matrix(T4_orientation[-1]), (-Hight_drop_off_box, 0, 0)) + DROP_OFF_ZONE
                    
                    T5 = self.planner.generate_path(retracked_point, Drop_off_hight, linear=False)
                    delta_angles = np.linspace(0, -np.pi, len(T5))
                    T5_orientation = [rotate_quaternion(T4_orientation[-1], rad, 0, 0) for rad in delta_angles]
                    self.travle_paths.append(T5)
                    self.travle_orientation.append(T5_orientation)

            if depbug_stage > 7:
                T6 = self.planner.generate_path(T5[-1], DROP_OFF_ZONE, linear=True)
                T6_orientation = [T5_orientation[-1] for _ in range(len(T6))]
                self.travle_paths.append(T6)
                self.travle_orientation.append(T6_orientation)

            if depbug_stage > 8:
                retracked_point = np.dot(quaternion_to_rotation_matrix(T6_orientation[-1]), (0, 0, -insershion_distance)) + T6[-1]
                T7 = self.planner.generate_path(T6[-1], retracked_point, linear=True)
                T7_orientation = [T6_orientation[-1] for _ in range(len(T7))]
                self.travle_paths.append(T7)
                self.travle_orientation.append(T7_orientation)

            if depbug_stage > 9:
                T8 = self.planner.generate_path(T7[-1], JOGGING_START, linear=False)
                T8_orientation = [quaternion_slerp(T7_orientation[-1], JOGGING_START_ORIENTATION, t) for t in np.linspace(0, 1, len(T8), endpoint=True)]
                self.travle_paths.append(T8)
                self.travle_orientation.append(T8_orientation)

        if depbug_stage > 10:
            T9 = self.planner.generate_path(JOGGING_START, WORKING_POSITION, linear=False)
            T9_orientation = [T8_orientation[-1] for _ in range(len(T9))]
            self.travle_paths.append(T9)
            self.travle_orientation.append(T9_orientation)

        if depbug_stage > 11:
            T10 = self.planner.generate_path(WORKING_POSITION, IDLE_POSITION, linear=True)
            T10_orientation = [T9_orientation[-1] for _ in range(len(T10))]
            self.travle_paths.append(T10)
            self.travle_orientation.append(T10_orientation)

        # Plot the 3D paths
        self.show_path()

    def show_path(self):
        """Display the generated paths."""
        self.PathShow = True
        self.planner.plot_3d_path()
        self.planner.clear_saved_paths()

    def animate_robot(self):
        positions_ani = []
        orientations_ani = []
        travle_alinements = []
        if self.PathShow:

            for i in range(len(self.travle_orientation)):
                for j in range(len(self.travle_orientation[i])):
                    ani = quaternion_to_rotation_matrix(self.travle_orientation[i][j])
                    orientations_ani.append(ani)

            for i in range(len(self.travle_paths)):
                positions_ani.extend(self.travle_paths[i]) 

            travle_alinements = ["all"] * len(positions_ani)

            self.robot.animate_ik(positions_ani, orientations_ani, travle_alinements)
        else:
            print("Must start sim")
   
    def returnSim():
        pass

# class ReturnPartSimPage(PageBase):
#     """Page for Motion Planning and Verification.

#     Args:
#         PageBase (type): The base class for pages.
#     """
#     @error_handling_wrapper
#     def __init__(self, parent: tk.Widget, controller: RobotCart):
#         """Initialize the MoshionPlanningPage.

#         Args:
#             parent (tk.Widget): The parent widget.
#             controller (RobotCart): The main application controller.
#         """
#         # Call the constructor of the base class (PageBase)
#         super().__init__(parent, controller, "Return Part")

#         box_w = 0.2
#         box_h = 0.25
#         grid_size = (1.40, 1.40)

#         # Additional attribute to store cart data
#         self.PartReturns = {}

#         # Create a PartsDatabase instance for handling parts data
#         self.db_path = 'C:\\Users\\zachl\\Capstone2024\\parts_db'
#         self.parts_db = PartsDatabase(db_name="parts_db", grid_size = grid_size, shelf_height=0.015, margin=0.015, offset_x=(box_w / 2), offset_y=(box_h / 2))
#         self.parts_db.create_parts_table()

#         # Create a frame to hold the main treeview for available parts
#         self.parts_treeview_frame = tk.Frame(self.content_frame)
#         self.parts_treeview_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

#         self.configure_order_treeview()

#         # Bind a click event to select a part
#         # self.parts_treeview.bind("<ButtonRelease-1>", lambda event: self.select_part())

#         # Create buttons for interaction with available parts
#         self.create_buttons(controller)

#         # Create Entry boxes for parts data
#         self.parts_entry_boxes()

#     def create_buttons(self, controller: RobotCart) -> None:
#         """Create buttons for motion planning.

#         Args:


if __name__ == "__main__":
    """Entry point of the script."""

    app = RobotCart()
    app.mainloop()