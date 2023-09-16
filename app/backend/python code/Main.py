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

AllMotorNames = ["BACE", 
              "SHOULDER", 
              "ELBOW",
              "ELBOWREVOLUT",
              "WRIST",
              "WRISTREVOLUT"]


# Define a custom Tkinter application class
class RobotCart(tk.Tk):
    def __init__(self, *args, **kwargs):
        """
        Initialize the main application.

        Args:
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.
        """
        # Initialize the Tkinter application
        tk.Tk.__init__(self, *args, **kwargs)
        
        # Create a container frame to hold the pages
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)

        # Configure the container's grid layout
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        # Dictionary to store different pages
        self.frames = {}

        # Create and add pages to the application
        for F in (MainUserPage, InventoryPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        # Show the MainUserPage by default
        self.show_frame(MainUserPage)

        # Create a PartsDatabase instance and create the Parts table
        parts_db = PartsDatabase()
        parts_db.create_parts_table()

    def show_frame(self, cont):
        """
        Show the specified frame.

        Args:
            cont: The frame to be displayed.
        """
        frame = self.frames[cont]
        frame.tkraise()

# Base class for pages
class PageBase(tk.Frame):
    def __init__(self, parent, controller, title):
        """
        Initialize a page.

        Args:
            parent (tk.Frame): The parent frame.
            controller (tk.Tk): The main application controller.
            title (str): The title to be displayed on the page.
        """
        tk.Frame.__init__(self, parent)
        
        # Title label
        label = tk.Label(self, text=title, font=LARGE_FONT)
        label.pack(pady=10, padx=10)

# MainUserPage class
class MainUserPage(PageBase):
    def __init__(self, parent, controller):
        def Connect_to_robot():
            # Initialize the Robot instance with the serial port
            robot = Robot("COM3")  # Replace "COM3" with your actual serial port

            try:
                # Attempt to establish a connection with the Arduino
                robot.connect()
                print("Connection with Arduino established.")

                # Create a messager instance for communication
                robot_messager = messager(robot)

            except RobotConnectionTimeout as e:
                print(f"Error: {e}")
            finally:
                # Close the serial connection when done
                robot.close_connection()

        super().__init__(parent, controller, "MainUserPage")

        # Button to navigate to Page One
        RconnectBTN = tk.Button(self, text="Connect to Robot", command=lambda: Connect_to_robot())
        RconnectBTN.pack()

# Page One class
class InventoryPage(PageBase):
    def __init__(self, parent, controller):

        """
        Initialize Page One.

        Args:
            parent (tk.Frame): The parent frame.
            controller (tk.Tk): The main application controller.
        """
        super().__init__(parent, controller, "InventoryPage")


if __name__ == "__main__":
    # Create and run the main application
    app = RobotCart()
    app.mainloop()




    
    

