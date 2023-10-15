class motor:
    def __init__(self, name: str, index: int, max_speed: float, max_acceleration: float, max_torqu: float):
        """
        Initializes a motor with the provided parameters.

        Args:
            name (str): Name of the motor.
            index (int): Index of the motor.
            max_speed (float): Maximum allowable speed.
            max_acceleration (float): Maximum acceleration.
            max_torqu (float): Maximum torque.

        Raises:
            ValueError: If any of the input parameters are invalid.
        """
        if not isinstance(name, str) or not name:
            raise ValueError("Invalid name. Name must be a non-empty string.")
        if not isinstance(index, int) or index < 0:
            raise ValueError("Invalid index. Index must be a non-negative integer.")
        if not isinstance(max_speed, (int, float)) or max_speed <= 0:
            raise ValueError("Invalid max_speed. Max_speed must be a positive float.")
        if not isinstance(max_acceleration, (int, float)) or max_acceleration <= 0:
            raise ValueError("Invalid max_acceleration. Max_acceleration must be a positive float.")
        if not isinstance(max_torqu, (int, float)) or max_torqu <= 0:
            raise ValueError("Invalid max_torque. Max_torque must be a positive float.")

        self.name = name
        self.index = index
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_torqu = max_torqu
        self.is_activate = False
        self.type = None

    def set_max_speed(self, new_speed: float):
        """
        Set the maximum speed of the motor.

        Args:
            new_speed (float): New maximum speed.

        Raises:
                ValueError: If the new speed is not a positive float.
        """
        if not isinstance(new_speed, (int, float)) or new_speed <= 0:
            raise ValueError("Invalid max_speed. Max_speed must be a positive float.")
        self.max_speed = new_speed

    def set_max_acceleration(self, new_acceleration: float):
        """
        Set the maximum acceleration of the motor.

        Args:
            new_acceleration (float): New maximum acceleration.

        Raises:
            ValueError: If the new acceleration is not a positive float.
        """
        if not isinstance(new_acceleration, (int, float)) or new_acceleration <= 0:
            raise ValueError("Invalid max_acceleration. Max_acceleration must be a positive float.")
        self.max_acceleration = new_acceleration

    def activate(self, state: bool = True):
        """
        Activate or deactivate the motor.

        Args:
            state (bool, optional): True to activate, False to deactivate. Defaults to True.

        Raises:
            ValueError: If the state is not a boolean value.
        """
        if not isinstance(state, bool):
            raise ValueError("Invalid state. State must be a boolean value.")
        self.is_activate = state

    def __str__(self) -> str:
        """
        Return a string representation of the motor's information.

        Returns:
            str: A formatted string containing motor information.
        """
        return (
            f"Motor Information:\n"
            f"  Name: {self.name}\n"
            f"  Index: {self.index}\n"
            f"  Max Speed: {self.max_speed} RPM\n"
            f"  Max Acceleration: {self.max_acceleration} RPM/s\n"
            f"  Max Torqu: {self.max_torqu}\n"
            f"  Activation State: {'Active' if self.is_activate else 'Inactive'}\n"
        )
    
class StepperMotor(motor):
    def __init__(self, name: str, index: int, max_speed: float, max_acceleration: float, max_torque: float, steps_per_revolution: int):
        """
        Initialize a stepper motor with the provided parameters.

        Args:
            name (str): Name of the motor.
            index (int): Index of the motor.
            max_speed (float): Maximum allowable speed.
            max_acceleration (float): Maximum acceleration.
            max_torque (float): Maximum torque.
            steps_per_revolution (int): Steps per revolution.
        """
        if not isinstance(max_speed, (int, float)) or max_speed <= 0:
            raise ValueError("Invalid max_speed. Max_speed must be a positive float.")

        # Call the constructor of the base class (motor)
        super().__init__(name, index, max_speed, max_acceleration, max_torque)
        self.type = "Stepper Motor"
        self.steps_per_revolution = steps_per_revolution

    def __str__(self) -> str:
        """
        Return a string representation of the stepper motor's information.

        Returns:
            str: A formatted string containing stepper motor information.
        """
        motor_info = super().__str__()
        return f"{motor_info}  Steps per Revolution: {self.steps_per_revolution}"

def test_motor():
    try:
        # Test the motor class
        print("Testing Motor Class:")
        test_motor = motor("Test Motor", 1, 100.0, 10.0, 1.0)
        print(test_motor)

        # Test setting max speed
        test_motor.set_max_speed(150.0)
        print(f"Max Speed Updated: {test_motor.max_speed}")

        # Test setting max acceleration
        test_motor.set_max_acceleration(15.0)
        print(f"Max Acceleration Updated: {test_motor.max_acceleration}")

        # Test activating the motor
        test_motor.activate()
        print(f"Motor Activated: {test_motor.is_activate}")

        print("\nTesting StepperMotor Class:")
        # Test the StepperMotor class
        test_stepper_motor = StepperMotor("Test Stepper Motor", 2, 200.0, 20.0, 2.0, 80000)
        print(test_stepper_motor)

        # Test setting max speed for StepperMotor
        test_stepper_motor.set_max_speed(250.0)
        print(f"Max Speed Updated: {test_stepper_motor.max_speed}")

        # Test setting max acceleration for StepperMotor
        test_stepper_motor.set_max_acceleration(25.0)
        print(f"Max Acceleration Updated: {test_stepper_motor.max_acceleration}")

        # Test activating the StepperMotor
        test_stepper_motor.activate()
        print(f"Stepper Motor Activated: {test_stepper_motor.is_activate}")

    except ValueError as ve:
        print(f"ValueError: {ve}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    test_motor()
