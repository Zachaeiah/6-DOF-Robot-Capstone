from MotorManager import *
from Motors import *

manager = motorManager({})

# Read motor configurations from the JSON file
manager.read_motor_config("motors_config.json")


angles = [(0, 0, 0, 0, 0, 0), (10, 20, 30, 40, 50, 60)]

for point in range(1 , len(angles), 1):
    Times = []  
    new_times = []
    # Iterate over motors and insert them into the Treeview
    for index, (motor_name, motor) in enumerate(manager.motors.items()):
        # Calculate the number of steps per degree for the motor
        steps_per_degree = motor.steps_per_revolution / 360

        # Calculate the time for one step (in seconds)
        time_per_step = (1 / motor.max_speed) * (1 / motor.steps_per_revolution) * 60

        # Calculate the total time for the motor movement
        motor_time = abs(angles[point-1][index] - angles[point][index]) * time_per_step * steps_per_degree


        Times.append(motor_time)



    # Iterate over motors and insert them into the Treeview
    for index, (motor_name, motor) in enumerate(manager.motors.items()):
        # Calculate the number of steps per degree for the motor
        steps_per_degree = motor.steps_per_revolution / 360

        # Calculate the time for one step (in seconds)
        time_per_step =  max(Times)/(abs(angles[point-1][index] - angles[point][index]) * steps_per_degree)

        # Calculate the total time for the motor movement, they should all be the same
        motor_time = abs(angles[point-1][index] - angles[point][index]) * time_per_step * steps_per_degree

        new_times.append(motor_time)

    print(Times)
    print(new_times)

