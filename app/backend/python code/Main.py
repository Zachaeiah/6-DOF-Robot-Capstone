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
PICK_UP_ZONE = (50, -100, 50)

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
        motors = [StepperMotor(AllMotorNames[i], 3000, 100, 7e4, 27) for i in active_motors]

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
                                'Orientation': [part[6], part[7], part[8]],
                                'FullWeight': part[9],
                                'HalfWeight': part[10],
                                'EmptyWeight': part[11]
                            }
                    state = 2  # Transition to the next state
                case 2:
                    # Get the locations of all parts

                    locations = {}  # Dictionary to store locations of each part

                    for part_name, part_info in part_info_dict.items():
                        location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
                        locations[part_name] = location

                      
                    state = 3  # Transition to the next state
                case 3:
                    # Create an instance of the PathPlanner class
                    planner = PathPlanner()

                    # Set the resolution for both paths
                    resolution = 100

                    for part_name, location in locations.items():
                        planner.generate_path(location, DROP_OFF_ZONE, resolution, linear=False)

                    # Plot the 3D paths
                    planner.plot_3d_path()
                    print(planner.saved_paths[0])

                    state = 4  # Transition to the next state
                case 4:
                    ## inverce kinamatics
                    run = False  # Exit the loop when done

    except RobotConnectionTimeout as e:
        print(f"Error: {e}")
    finally:
        # Close the serial connection when done
        robot.close_connection()

if __name__ == "__main__":
   main1()


# # Define a custom Tkinter application class
# class RobotCart(tk.Tk):
#     def __init__(self, *args, **kwargs):
#         """
#         Initialize the main application.

#         Args:
#             *args: Additional positional arguments.
#             **kwargs: Additional keyword arguments.
#         """
#         # Initialize the Tkinter application
#         tk.Tk.__init__(self, *args, **kwargs)
        
#         # Create a container frame to hold the pages
#         container = tk.Frame(self)
#         container.pack(side="top", fill="both", expand=True)

#         # Configure the container's grid layout
#         container.grid_rowconfigure(0, weight=1)
#         container.grid_columnconfigure(0, weight=1)

#         # Dictionary to store different pages
#         self.frames = {}

#         # Create and add pages to the application
#         for F in (MainUserPage, InventoryPage):
#             frame = F(container, self)
#             self.frames[F] = frame
#             frame.grid(row=0, column=0, sticky="nsew")

#         # Show the MainUserPage by default
#         self.show_frame(MainUserPage)

#         # Create a PartsDatabase instance and create the Parts table
#         parts_db = PartsDatabase()
#         parts_db.create_parts_table()

#     def show_frame(self, cont):
#         """
#         Show the specified frame.

#         Args:
#             cont: The frame to be displayed.
#         """
#         frame = self.frames[cont]
#         frame.tkraise()

# # Base class for pages
# class PageBase(tk.Frame):
#     def __init__(self, parent, controller, title):
#         """
#         Initialize a page.

#         Args:
#             parent (tk.Frame): The parent frame.
#             controller (tk.Tk): The main application controller.
#             title (str): The title to be displayed on the page.
#         """
#         tk.Frame.__init__(self, parent)
        
#         # Title label
#         label = tk.Label(self, text=title, font=LARGE_FONT)
#         label.pack(pady=10, padx=10)

# # MainUserPage class
# class MainUserPage(PageBase):
#     def __init__(self, parent, controller):
#         def Connect_to_robot():
#             # Initialize the Robot instance with the serial port
#             robot = Robot("COM3")  # Replace "COM3" with your actual serial port

#             try:
#                 # Attempt to establish a connection with the Arduino
#                 robot.connect()
#                 print("Connection with Arduino established.")

#                 # Create a messager instance for communication
#                 robot_messager = messager(robot)

#             except RobotConnectionTimeout as e:
#                 print(f"Error: {e}")
#             finally:
#                 # Close the serial connection when done
#                 robot.close_connection()

#         super().__init__(parent, controller, "MainUserPage")

#         # Button to navigate to Page One
#         RconnectBTN = tk.Button(self, text="Connect to Robot", command=lambda: Connect_to_robot())
#         RconnectBTN.pack()

# # Page One class
# class InventoryPage(PageBase):
#     def __init__(self, parent, controller):

#         """
#         Initialize Page One.

#         Args:
#             parent (tk.Frame): The parent frame.
#             controller (tk.Tk): The main application controller.
#         """
#         super().__init__(parent, controller, "InventoryPage")

    
    

