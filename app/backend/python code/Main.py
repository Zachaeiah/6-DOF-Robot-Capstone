import numpy as np
import Messager
import Motors
import Robot
import VelocityPFP


  
def main():
    robot = Robot.robot('CON3')
    messager = Messager.messager(robot)
    motor = Motors.motor('test Motor', 100, 10, 70000)

    angles = [90.0,90.0,90.0,90.0,90.9,90.9]
    speeds = [10.0,10.0,10.0,10.0,10.0,10.0]

    print(messager.ROTATE_JOINT(angles, speeds))
    print(motor)


if __name__ == "__main__":
    main()




    
    

