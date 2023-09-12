import numpy as np
import Messager
import Motors
import Robot
import VelocityPFP

activeMotors = range(3, 6, 1)


  
def main():
    robot = Robot.robot('CON3')
    messager = Messager.messager(robot)

    for motor in activeMotors:
        print(motor)




    # motor = Motors.motor('test Motor', 100, 10, 70000)
    # PFP = VelocityPFP.Sigmoid(1, 0)
    # motor.set_velocity_profile(PFP)
    # speed = motor.velocity_profile.velocity(0)
    # print(speed)



if __name__ == "__main__":
    main()




    
    

