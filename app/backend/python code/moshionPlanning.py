from MotorManager import *
from Motors import *

manager = motorManager({})

# Read motor configurations from the JSON file
manager.read_motor_config("motors_config.json")


start_degrees = [(0, 0, 0, 0, 0, 0), (10, 20, 30, 40, 50, 60)]
end_degrees = [(10, 20, 30, 40, 50, 60), (60, 50, 40, 20, 20, 10)]

for point in range(0 , len(end_degrees)-1, 1):
    Times = []  
    new_times = []
    # Iterate over motors and insert them into the Treeview
    for index, (motor_name, motor) in enumerate(manager.motors.items()):
        # Calculate the number of steps per degree for the motor
        steps_per_degree = motor.steps_per_revolution / 360

        # Calculate the time for one step (in minutes)
        time_per_step = (1 / motor.max_speed) * (1 / motor.steps_per_revolution) * 60

        # Calculate the total time for the motor movement
        motor_time = abs(end_degrees[point][index] - start_degrees[point][index]) * time_per_step * steps_per_degree


        Times.append(motor_time)



    # Iterate over motors and insert them into the Treeview
    for index, (motor_name, motor) in enumerate(manager.motors.items()):
        # Calculate the number of steps per degree for the motor
        steps_per_degree = motor.steps_per_revolution / 360

        # Calculate the time for one step (in minutes)
        time_per_step =  max(Times)/(abs(end_degrees[point][index] - start_degrees[point][index]) * steps_per_degree)

        # Calculate the total time for the motor movement
        motor_time = abs(end_degrees[point][index] - start_degrees[point][index]) * time_per_step * steps_per_degree

        new_times.append(motor_time)

    print(Times)
    print(new_times)




