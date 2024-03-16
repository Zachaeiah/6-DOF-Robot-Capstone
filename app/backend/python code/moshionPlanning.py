from MotorManager import *

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

    def move_motors(self, angles):
        """
        Move motors based on the provided angles.

        Args:
            angles (list of tuples): A list of tuples where each tuple represents
                                     the angles for each motor.

        Returns:
            str: A string containing all six frequencies and the maximum time.
        """
        frequencies = []  # List to store frequencies of all motors
        times = []        # List to store times of all motors
        for point in range(1, len(angles)):
            Times = []

            # Calculate motor times for the current movement
            for index, (motor_name, motor) in enumerate(list(self.manager.motors.items())):
                motor_time = (angles[point][index] - angles[point-1][index])*(60.0/(360.0*motor.max_speed))
                Times.append(motor_time)

            # Calculate motor frequencies and new times for the current movement
            for index, (motor_name, motor) in enumerate(list(self.manager.motors.items())):
                delta_theta = (angles[point][index] - angles[point-1][index])

                if delta_theta == 0:
                    new_motor_speed = 0
                    motor_time = 0
                else:
                    new_motor_speed = (delta_theta*60)/(360*abs((max(Times, key=abs))))

                motor_frequensy = (new_motor_speed * motor.steps_per_revolution)/60

                    

                # Append frequencies and times to respective lists
                frequencies.append(f"{int(motor_frequensy)}")
                

        # Convert times to string and find maximum time
        max_time = abs(max(Times, key=abs))
        max_time_str = f"{int(max_time*1e6)}"

        # Concatenate frequencies and max time into a single string
        result_str = ", ".join(reversed(frequencies)) + ", " + max_time_str +"\n"

        return result_str

if __name__ == "__main__":
    # Create an instance of MoshionController
    controller = MoshionController()

    # Define the angles for motor movements
    angles = [(0,0,0,0,0,0), (0,0,0,0,-20,20)]

    # Move motors based on the defined angles and get the results
    movement_results = controller.move_motors(angles)

    # Print the concatenated string containing frequencies and max time
    print("Motor Frequencies and Max Time:")
    print(movement_results)
