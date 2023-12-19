from MotorManager import *
from Motors import *

def format_time_with_prefix(time_micros):
    prefixes = ['n', 'u', 'm']  # nano, micro, milli
    index = 0
    
    if time_micros >= 1:
        return f"{time_micros:.5f} s"  # Return seconds directly

    while time_micros < 1 and index < len(prefixes) - 1:
        time_micros *= 1000
        index += 1

    return f"{time_micros:.5f} {prefixes[index]}s"


def main():
    # Create a MotorManager instance
    manager = motorManager({})

    # Read motor configurations from the JSON file
    manager.read_motor_config("motors_config.json")

    # Define a list of angles for each motor
    angles = [(0, 0, 0, 0, 0, 0), (10, 90, 90, 10, 10, 10)]

    for point in range(1, len(angles)):
        Times = []
        new_times = []

        for index, (motor_name, motor) in enumerate(manager.motors.items()):
             # Calculate the number of steps per degree for the motor
            steps_per_degree = motor.steps_per_revolution / 360

            # Calculate the time for one step (in seconds)
            seconds_per_step = (60 / (360 * motor.max_speed *  motor.steps_per_revolution))

            # Calculate the total time for the motor movement in seconds
            number_of_steps = abs(angles[point][index] - angles[point-1][index]) * steps_per_degree

            motor_time_seconds = (60*abs(angles[point][index] - angles[point-1][index]))/(360)

            print(f"motor: {index} steps_per_degree: {steps_per_degree:.5f} seconds_per_step: {seconds_per_step} number_of_steps: {number_of_steps:.5f} motor_time_seconds: {motor_time_seconds:.5f}")

            Times.append(motor_time_seconds)

        print("\n")
    
        for index, (motor_name, motor) in enumerate(manager.motors.items()):
            # Calculate the number of steps per degree for the motor
            steps_per_degree = motor.steps_per_revolution / 360

            # Calculate the time for one step (in seconds)
            seconds_per_step =  max(Times)/(abs(angles[point-1][index] - angles[point][index]) * steps_per_degree)

            # Calculate the total time for the motor movement, they should all be the same
            motor_time = abs(angles[point-1][index] - angles[point][index]) * seconds_per_step * steps_per_degree

            new_times.append(motor_time)

            print(f"motor: {index} steps_per_degree: {steps_per_degree:.5f} seconds_per_step: {seconds_per_step} number_of_steps: {number_of_steps:.5f} motor_time_seconds: {motor_time:.5f}")



if __name__ == "__main__":
    main()
