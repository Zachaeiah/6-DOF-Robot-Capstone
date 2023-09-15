import numpy as np
from Messager import *
from MotorManager import *
from Motors import *
from Robot import *
from VelocityPFP import *

activeMotors = range(3, 6, 1)


  
def main():
    # Initialize the Robot instance with the serial port
    robot = Robot("COM3")  # Replace "COM3" with your actual serial port

    try:
        # Attempt to establish a connection with the Arduino
        robot.connect()

        # Continue with the rest of your code once the connection is established
        print("Connection with Arduino established.")
        
        # Your code here - you can perform actions with the robot

    except RobotConnectionTimeout as e:
        print(f"Error: {e}")
    finally:
        # Close the serial connection when done
        robot.close_connection()

    Rmessager = messager(robot)


if __name__ == "__main__":
    main()




    
    

