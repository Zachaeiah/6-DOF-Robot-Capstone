import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time


import sys
sys.path.append('E:\\Capstone\\app\\backend\\python code')  # Add the directory to sys.path

import numpy as np
from Messager import *
from MotorManager import *
from Motors import *
from Robot import *
from intrerpolation import *
from VelocityPFP import *
from DP_parts import *

AllMotorNames = ["BACE", 
              "SHOULDER", 
              "ELBOW",
              "ELBOWREVOLUT",
              "WRIST",
              "WRISTREVOLUT"]


  
def main():
    # Initialize the Robot instance with the serial port
    robot = Robot("COM3")  # Replace "COM3" with your actual serial port

    try:
        # Attempt to establish a connection with the Arduino
        robot.connect()
        print("Connection with Arduino established.")
        
        # Create a messager instance for communication
        robot_messager = messager(robot)

        # Create and configure StepperMotor instances
        active_motors = range(3, 6, 1)  
        motors = [StepperMotor(AllMotorNames[i], 3000, 100, 7e4) for i in active_motors]

        # Create a MotorManager instance
        motor_manager = motorManager(motors)
        print(motor_manager)

        # Create a PartsDatabase instance and create the Parts table
        parts_db = PartsDatabase()
        parts_db.create_parts_table()

        # Your code here - you can perform actions with the robot

    except RobotConnectionTimeout as e:
        print(f"Error: {e}")
    finally:
        # Close the serial connection when done
        robot.close_connection()

if __name__ == "__main__":
    main()

