from Motors import *
import json

class motorManager:
    def __init__(self, motors: dict):
        """
        Initializes a motor manager with a dictionary of motors.

        Args:
            motors (dict): Dictionary of all the motors.

        Raises:
            TypeError: If the 'motors' argument is not a dictionary.
        """
        if not isinstance(motors, dict):
            raise TypeError("'motors' argument must be a dictionary.")
        self.motors = motors

    def activate(self, motor_name: str, state: bool) -> bool:
        """
        Activates or deactivates a motor.

        Args:
            motor_name (str): Name of the motor to activate or deactivate.
            state (bool): True if the motor is to be activated, False if the motor is to be deactivated.

        Returns:
            bool: False if the motor doesn't exist, True otherwise.
        """
        try:
            if motor_name in self.motors:
                self.motors[motor_name].activate(state)
                return True
            else:
                print(f"Error: Motor with name '{motor_name}' not found.")
                return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def remove_motor(self, motor_name: str) -> bool:
        """
        Removes a motor from the manager.

        Args:
            motor_name (str): Name of the motor to remove.

        Returns:
            bool: True if the motor was removed, False if the motor doesn't exist.
        """
        try:
            if motor_name in self.motors:
                del self.motors[motor_name]
                return True
            else:
                print(f"Error: Motor with name '{motor_name}' does not exist.")
                return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def get_motor_by_index(self, index: int):
        """
        Returns the motor with the specified index.

        Args:
            index (int): Index of the motor.

        Returns:
            motor: The motor with the specified index, or None if not found.
        """
        try:
            for motor in self.motors.values():
                if motor.index == index:
                    return motor
            print(f"Error: Motor with index '{index}' not found.")
            return None
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return None

    def get_motor_by_name(self, name: str):
        """
        Returns the motor with the specified name.

        Args:
            name (str): Name of the motor.

        Returns:
            motor: The motor with the specified name, or None if not found.
        """
        try:
            return self.motors[name]
        except KeyError:
            print(f"Error: Motor with name '{name}' not found.")
            return None
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return None

    def write_motor_config(self, file_path: str) -> bool:
        """
        Writes the motor configuration to a JSON file.

        Args:
            file_path (str): Path to the JSON file.

        Returns:
            bool: True if writing is successful, False otherwise.
        """
        try:
            with open(file_path, 'w') as json_file:
                json.dump(self.motors, json_file, indent=4, default=lambda x: x.__dict__)
            return True
        except FileNotFoundError as e:
            print(f"Error: File not found. {e}")
            return False
        except PermissionError as e:
            print(f"Error: Permission denied. {e}")
            return False
        except TypeError as e:
            print(f"Error: Type mismatch. {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def read_motor_config(self, file_path: str) -> bool:
        """
        Reads the motor configuration from a JSON file.

        Args:
            file_path (str): Path to the JSON file.

        Returns:
            bool: True if reading is successful, False otherwise.
        """
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                for motor_name, motor_data in data.items():
                    if "type" in motor_data and motor_data["type"] == "Stepper Motor":
                        new_motor = StepperMotor(
                            motor_data.get("name"),
                            motor_data.get("index"),
                            motor_data.get("max_speed"),
                            motor_data.get("max_acceleration"),
                            motor_data.get("max_torqu"),
                            motor_data.get("steps_per_revolution"),
                        )
                        if "is_activate" in motor_data and motor_data["is_activate"]:
                            new_motor.activate(True)
                        self.motors[motor_name] = new_motor
                    else:
                        raise ValueError("Motor type missing or not recognized.")
            return True
        except FileNotFoundError as e:
            print(f"File not found error: {e}")
            return False
        except json.JSONDecodeError as e:
            print(f"JSON decoding error: {e}")
            return False
        except Exception as e:
            print(f"An error occurred: {e}")
            return False

    def __str__(self):
        """
        Returns a string containing information about all motors in the manager.

        Returns:
            str: String containing motor information.
        """
        result = ""
        for motor_name, motor in self.motors.items():
            result += f"Motor Name: {motor.name}: {motor.is_activate}\n"

        return result

def main():
    test_motors = {f"Motor: {i}": StepperMotor(f"Motor: {i}", i, 100, 10, 1, 70_000) for i in range(0, 6, 1)}
    manager = motorManager(test_motors)

    # Activate a motor
    print(manager.activate("Motor: 2", True))  # Should return True

    # Print the motors' information
    print(manager)

    # Remove a motor
    print(manager.remove_motor("Motor: 3"))  # Should return True

    # Write motor configurations to a JSON file
    file_path = "motors_config.json"
    manager.write_motor_config(file_path)

    # Clear the motors data
    manager.motors = {}

    # Read motor configurations from the JSON file
    manager.read_motor_config(file_path)

    # Print the motors' information
    print(manager)

if __name__ == '__main__':
    main()
