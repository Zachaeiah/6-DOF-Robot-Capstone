from MotorManager import *
import numpy as np

class MoshionController:
    
    def __init__(self):
        """
        Initialize the MoshionController.

        This method creates a MotorManager instance and reads motor configurations
        from a JSON file.
        """
        # Create a MotorManager instance
        self.manager = motorManager({})
        # Read motor configurations from the JSON file
        self.manager.read_motor_config("motors_config.json")
        self.in_degrees = True
        self.SLOWEREST_FREQUENSY = 100
    
    def Input_DEGREES(self):
        """_summary_
        """
        self.in_degrees = True

    def Input_RAD(self):
        """_summary_
        """
        self.in_degrees = False

    def move_motors(self, angles):
        """
        Move motors based on the provided angles.

        Args:
            angles (list of tuples): A list of tuples where each tuple represents
                                    the angles for each motor.

        Returns:
            str: A string containing all six frequencies and the maximum time.
        """
        # Initialize lists and variables
        frequencies_list = []  # List to store frequencies of all motors
        slow_motor_max_time_list = np.array([0,0,0,0,0,0])  # Array to store max time for slow motors
        slow_motor_delta_theta_list = [0,0,0,0,0,0]  # List to store delta theta for slow motors
        max_time_list = []  # List to store max time for all movements
        slow_motor_angles = []  # List to store angles for slow motors

        # Loop through each movement point
        for point in range(1, len(angles), 1):
            times  = []  # List to store times for each motor movement
            motor_frequencys = np.array([0,0,0,0,0,0])  # Array to store frequencies for each motor

            # Calculate motor times for the current movement
            for index, (motor_name, motor) in enumerate(list(self.manager.motors.items())):
                delta_theta = (angles[point][index] - angles[point-1][index])  # Calculate angle change
                max_motor_frequency = int(motor.max_speed * motor.steps_per_revolution)/60  # Calculate max frequency

                # Check if angle change is within threshold or motor is at slowest frequency
                if abs(delta_theta) <= (360/motor.steps_per_revolution):
                    delta_theta = 0
                    motor_time = 0
                elif ( 0 < abs(max_motor_frequency) <= self.SLOWEREST_FREQUENSY):
                    delta_theta = 0
                    motor_time = 0
                else:
                    if self.in_degrees:
                        motor_time = delta_theta*(60.0/(360.0*motor.max_speed))  # Calculate motor time in degrees
                    else:
                        motor_time = delta_theta*(60.0/(2*np.pi*motor.max_speed))  # Calculate motor time in radians
                times.append(motor_time)

            # Calculate motor frequencies and new times for the current movement
            for index, (motor_name, motor) in enumerate(list(self.manager.motors.items())):
                delta_theta = (angles[point][index] - angles[point-1][index])  # Calculate angle change

                # Check if angle change is within threshold
                if abs(delta_theta) <= (360/motor.steps_per_revolution):
                    delta_theta = 0
                    Motor_time = 0

                if delta_theta == 0:
                    new_motor_speed = 0
                    motor_time = 0
                    Motor_time = 0
                else:
                    Motor_time = abs(max(times, key=abs))  # Get maximum time
                    if self.in_degrees:
                        new_motor_speed = (delta_theta*60)/(360*Motor_time)  # Calculate new motor speed in degrees
                    else:
                        new_motor_speed = (delta_theta*60)/(2*np.pi*Motor_time)  # Calculate new motor speed in radians

                new_motor_frequency = int(new_motor_speed * motor.steps_per_revolution)/60  # Calculate new frequency

                # Check if new frequency is within threshold
                if ( 0 < abs(new_motor_frequency) <= self.SLOWEREST_FREQUENSY):
                    slow_motor_angles.append((index, delta_theta))  # Add to slow motor angles
                    new_motor_frequency = 0  # Set frequency to zero

                motor_frequencys[index] = new_motor_frequency  # Store new frequency

            frequencies_list.append([str(frequency) for frequency in motor_frequencys])  # Store frequencies list

            if (abs(max(times, key=abs)) * 1e6) <= 1:
                max_time_list.append(1)  # Append max time as 1 microsecond if very small
            else:
                max_time_list.append(int(abs(max(times, key=abs)) * 1e6))  # Append max time in microseconds

            # Calculate slow motor max time and delta theta
            for SM in slow_motor_angles:
                slow_move = np.array([0,0,0,0,0,0])  # Array to store slow motor movement
                slow_motor = self.manager.get_motor_by_index(SM[0])  # Get slow motor

                if self.in_degrees:
                    slow_motor_time = SM[1]*(60.0/(360.0*slow_motor.max_speed))  # Calculate slow motor time in degrees
                else:
                    slow_motor_time = SM[1]*(60.0/(2*np.pi*slow_motor.max_speed))  # Calculate slow motor time in radians
                
                slow_motor_frequency = int((slow_motor.max_speed*slow_motor.steps_per_revolution)/60)  # Calculate slow motor frequency
                
                if 1/abs(slow_motor_time) >= slow_motor_frequency:
                    continue  # Skip if slow motor time is greater than or equal to frequency

                slow_move[SM[0]] = slow_motor_frequency  # Store slow motor frequency
                slow_motor_max_time_list[SM[0]] += abs(slow_motor_time) * 1e6  # Add slow motor time to max time list
                slow_motor_delta_theta_list[SM[0]] += SM[1]  # Add delta theta to delta theta list

        # Calculate final frequencies and max times for slow motors
        for index, time in enumerate(slow_motor_max_time_list):
            if time == 0:
                continue

            motor_frequencys = np.array([0,0,0,0,0,0])  # Array to store final frequencies

            motor = self.manager.get_motor_by_index(index)  # Get motor
            if slow_motor_delta_theta_list[index] < 0:
                motor_frequency = -int((motor.max_speed*motor.steps_per_revolution)/60)  # Calculate negative frequency
            else:
                motor_frequency = int((motor.max_speed*motor.steps_per_revolution)/60)  # Calculate positive frequency

            motor_frequencys[index] = motor_frequency  # Store frequency

            frequencies_list.append([str(frequency) for frequency in motor_frequencys])  # Store frequencies list
            max_time_list.append(time)  # Append max time

        # Create return string
        return_str = []
        for move in range(0, len(frequencies_list), 1):
            move_cmd =  ", ".join(reversed(frequencies_list[move])) + ", " + str(max_time_list[move]) +"\n"  # Create move command
            return_str.append(move_cmd)  # Append move command to return string

        return return_str  # Return move commands
                
if __name__ == "__main__":
    # Create an instance of MoshionController
    controller = MoshionController()
    controller.Input_DEGREES()

    # Define the angles for motor movements
    angles = [(    0,   0,   0,   0,      0,   0), 
              (   45,   45,   0,   0,      0,   0)]
    movement_results = controller.move_motors(angles)

    for move in movement_results:
        print(move)


    
    
