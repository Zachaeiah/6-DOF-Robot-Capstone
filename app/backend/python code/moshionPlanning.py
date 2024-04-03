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
        frequencies_list = []  # List to store frequencies of all motors
        slow_motor_max_time_list = np.array([0,0,0,0,0,0])
        slow_motor_delta_theta_list = [0,0,0,0,0,0]
        max_time_list = []
        slow_motor_angles = []
        for point in range(1, len(angles), 1):
            times  = []
            motor_frequencys = np.array([0,0,0,0,0,0])

            # Calculate motor times for the current movement
            for index, (motor_name, motor) in enumerate(list(self.manager.motors.items())):
                delta_theta = (angles[point][index] - angles[point-1][index])
                max_motor_frequency = int(motor.max_speed * motor.steps_per_revolution)/60
                if abs(delta_theta) <= (360/motor.steps_per_revolution):
                    delta_theta = 0
                    motor_time = 0
                elif ( 0 < abs(max_motor_frequency) <= self.SLOWEREST_FREQUENSY):
                    delta_theta = 0
                    motor_time = 0
                else:
                    if self.in_degrees:
                        motor_time = delta_theta*(60.0/(360.0*motor.max_speed))
                    else:
                        motor_time = delta_theta*(60.0/(2*np.pi*motor.max_speed))
                times.append(motor_time)

            # Calculate motor frequencies and new times for the current movement
            for index, (motor_name, motor) in enumerate(list(self.manager.motors.items())):

                delta_theta = (angles[point][index] - angles[point-1][index])
                if abs(delta_theta) <= (360/motor.steps_per_revolution):
                    delta_theta = 0

                if delta_theta == 0:
                    new_motor_speed = 0
                    motor_time = 0
                else:
                    Motor_time = abs(max(times, key=abs))
                    if self.in_degrees:
                        new_motor_speed = (delta_theta*60)/(360*Motor_time)
                    else:
                        new_motor_speed = (delta_theta*60)/(2*np.pi*Motor_time)

                new_motor_frequency = int(new_motor_speed * motor.steps_per_revolution)/60

                if ( 0 < abs(new_motor_frequency) <= self.SLOWEREST_FREQUENSY):
                    slow_motor_angles.append((index, delta_theta))
                    new_motor_frequency = 0
                
        
                motor_frequencys[index] = new_motor_frequency

            frequencies_list.append([str(frequency) for frequency in motor_frequencys])
            max_time_list.append(int(Motor_time * 1e6))

            
            for SM in slow_motor_angles:
                slow_move = np.array([0,0,0,0,0,0])
                slow_motor = self.manager.get_motor_by_index(SM[0])

                if self.in_degrees:
                    slow_motor_time = SM[1]*(60.0/(360.0*slow_motor.max_speed))
                else:
                    slow_motor_time = SM[1]*(60.0/(2*np.pi*slow_motor.max_speed))
                
                slow_motor_frequency = int((slow_motor.max_speed*slow_motor.steps_per_revolution)/60)
                
                if 1/abs(slow_motor_time) >= slow_motor_frequency:
                    continue
                
                slow_move[SM[0]] = slow_motor_frequency

                #slow_motor_frequencies_list.append([str(frequency) for frequency in slow_move])
                slow_motor_max_time_list[SM[0]] += abs(slow_motor_time) * 1e6
                slow_motor_delta_theta_list[SM[0]] += SM[1]

        
        for index, time in enumerate(slow_motor_max_time_list):
            if time == 0:
                continue

            motor_frequencys = np.array([0,0,0,0,0,0])

            motor = self.manager.get_motor_by_index(index)
            if slow_motor_delta_theta_list[index] < 0:
                motor_frequency = -int((motor.max_speed*motor.steps_per_revolution)/60)
            else:
                motor_frequency = int((motor.max_speed*motor.steps_per_revolution)/60)

            motor_frequencys[index] = motor_frequency

            frequencies_list.append([str(frequency) for frequency in motor_frequencys])
            max_time_list.append(time)

        return_str = []
        for move in range(0, len(frequencies_list), 1):
            move_cmd =  ", ".join(reversed(frequencies_list[move])) + ", " + str(max_time_list[move]) +"\n"
            return_str.append(move_cmd)

        return return_str 
                
if __name__ == "__main__":
    # Create an instance of MoshionController
    controller = MoshionController()

    # Define the angles for motor movements
    angles = [( 0.1,-0.1, 0.0, 0.0,    0.0,   50), 
              (   0,   0,   0,   0,      0,    0),
              (-0.1, 0.1, 0.0, 0.0,    0.0,   50),
              (-0.2,-0.2,   0,   0,      0,    0),
              (-0.3, 0.3, 0.0, 0.0,    0.0,   50)]
    controller.Input_DEGREES()

    movement_results = controller.move_motors(angles)

    for move in movement_results:
        print(move)


    
    
