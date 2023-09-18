import numpy as np
from Messager import *
from MotorManager import *
from Motors import *
from Robot import *
from intrerpolation import *
from VelocityPFP import *
from DP_parts import *
import tkinter as tk

LARGE_FONT = ("Verdana", 12)
DROP_OFF_ZONE = (-50, -100, 50)
DROP_OFF_ZONE_ORIENTATION = (0, -1, 0)
PICK_UP_ZONE = (50, -100, 50)
PULLBACKDISTANCE = 20

CORCE_RESOLUTION = 10
SOFT_RESOLUTION = 20
FINE_RESOLUTION = 30

AllMotorNames = ["BACE", 
              "SHOULDER", 
              "ELBOW",
              "ELBOWREVOLUT",
              "WRIST",
              "WRISTREVOLUT"]


  
def main1():
    # Initialize the Robot instance with the serial port
    robot = Robot("COM3")  # Replace "COM3" with your actual serial port

    try:
        # Attempt to establish a connection with the Arduino
        # robot.connect()
        # print("Connection with Arduino established.\n")
        
        # Create a messager instance for communication
        robot_messager = messager(robot)

        # Create and configure StepperMotor instances
        active_motors = range(3, 6, 1)  
        motors = [StepperMotor(AllMotorNames[i], 3000, 100, 7e4) for i in active_motors]

        # Create a MotorManager instance
        motor_manager = motorManager(motors)
        robot.include_motor_anager(motor_manager)

        # Create a PartsDatabase instance and create the Parts table
        parts_db = PartsDatabase()
        parts_db.create_parts_table()

        state = 0
        run = True  # Initialize run to True to enter the loop

        # State machine
        while run:
            match state:
                case 0:
                    # This part of the code is where we need to get the list of parts to fetch

                    # Array of part names to fetch
                    part_names_to_fetch = ['Part1', 'Part2', 'Part3']
                    #part_names_to_fetch = ['Part1']
                    state = 1  # Transition to the next state
                case 1:
                    # Get the part info from the database and organize it in a dictionary

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
                                'Orientation': (part[6], part[7], part[8]),
                                'FullWeight': part[9],
                                'HalfWeight': part[10],
                                'EmptyWeight': part[11]
                            }
                    state = 2  # Transition to the next state
                case 2:
                    # Get the locations of all parts

                    locations = {}  # Dictionary to store locations of each part
                    orientations = {}  # Dictionary to store orientations of each part


                    for part_name, part_info in part_info_dict.items():
                        location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
                        orientation = part_info['Orientation']
                        locations[part_name] = location
                        orientations[part_name] = orientation


                      
                    state = 3  # Transition to the next state
                case 3:
                    # Create an instance of the PathPlanner class
                    planner = PathPlanner()

                    # Set the resolution for both paths
                    resolution = SOFT_RESOLUTION     

                    for (part_name, location), (part_name, orientation) in zip(locations.items(), orientations.items()):

                        # Normalize the orientation (make it a unit orientation)
                        normalized_orientation = orientation / np.linalg.norm(orientation)

                        # Calculate the new point by adding the scaled vector to the original point
                        backPoint = location - PULLBACKDISTANCE * normalized_orientation

                        # Normalize the vector (make it a unit vector)
                        normalized_DROP_OFF_ZONE = DROP_OFF_ZONE_ORIENTATION / np.linalg.norm(DROP_OFF_ZONE_ORIENTATION)

                        # Calculate the new point by adding the scaled vector to the original point
                        pushPoint = DROP_OFF_ZONE + PULLBACKDISTANCE * normalized_DROP_OFF_ZONE

                        planner.generate_path(backPoint, location, resolution, linear=True)
                        planner.generate_path(location, backPoint, resolution, linear=True)
                        planner.generate_path(backPoint, DROP_OFF_ZONE, resolution, linear=False)
                        planner.generate_path(DROP_OFF_ZONE, pushPoint, resolution, linear=True)
                        planner.generate_path(pushPoint, DROP_OFF_ZONE, resolution, linear=True)
                        
                    # Plot the 3D paths
                    planner.plot_3d_path()

                    run = False  # Exit the loop when done

    except RobotConnectionTimeout as e:
        print(f"Error: {e}")
    finally:
        # Close the serial connection when done
        robot.close_connection()

if __name__ == "__main__":
   main1()
    

