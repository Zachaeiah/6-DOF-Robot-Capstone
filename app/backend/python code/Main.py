import numpy as np
from Messager import *
from MotorManager import *
from Motors import *
from Robot import *
from intrerpolation import *
from VelocityPFP import *

activeMotors = range(3, 6, 1)
Motors = []
AllMotorNames = ["BACE", 
              "SHOULDER", 
              "ELBOW",
              "ELBOWREVOLUT",
              "WRIST",
              "WRISTREVOLUT"]


  
def main():
    # Initialize the Robot instance with the serial port
   

    # try:
    #     # Attempt to establish a connection with the Arduino
    #     robot.connect()

    #     # Continue with the rest of your code once the connection is established
    #     print("Connection with Arduino established.")
        
    #     # Your code here - you can perform actions with the robot

    # except RobotConnectionTimeout as e:
    #     print(f"Error: {e}")
    # finally:
    #     # Close the serial connection when done
    #     robot.close_connection()
    robot = Robot("COM3")  # Replace "COM3" with your actual serial port
    robotMessager = messager(robot)
    
    for i in activeMotors:
        Motors.append(StepperMotor(f"{AllMotorNames[i]}", 3000, 100, 7e4))

    MotorManager = motorManager(Motors)

    print(MotorManager)


if __name__ == "__main__":
    main()




    
    

