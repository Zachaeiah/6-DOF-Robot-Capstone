class motor:
    """
    A class representing a motor.

    Args:
        name (str): The name or identifier of the motor.
        max_speed (float): The maximum speed of the motor in RPM (Revolutions Per Minute).
        max_acceleration (float): The maximum acceleration of the motor in RPM/s (Revolutions Per Minute per second).
    """

    def __init__(self, name: str, max_speed: float, max_acceleration: float, max_torqu: float):
        """
        Initialize a Motor instance with the specified parameters.

        Args:
            name (str): The name or identifier of the motor.
            max_speed (float): The maximum speed of the motor in RPM (Revolutions Per Minute).
            max_acceleration (float): The maximum acceleration of the motor in RPM/s (Revolutions Per Minute per second).
        """
        self.name = name
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.velocity_profile = None
        self.is_activate = False
        self.max_torqu = max_torqu
        self.parint = motor("", 0, 0, 0)

    def set_velocity_profile(self, profile):
        """
        Set the velocity profile for the motor.

        Args:
            profile: The velocity profile to set.
        """
        self.velocity_profile = profile
    
    def activate(self, state = True):
        self.is_activate = state

    def add_parint(self, parint):
        self.parint = parint

    def __str__(self) -> str:
        """
        Return a string representation of the motor's information.

        Returns:
            str: A formatted string containing motor information.
        """
        return (
            f"Motor Information:\n"
            f"  Name: {self.name}\n"
            f"  Max Speed: {self.max_speed} RPM\n"
            f"  Max Acceleration: {self.max_acceleration} RPM/s\n"
            f"  Velocity Profile: {self.velocity_profile}\n"
            f"  Activate state: {self.is_activate}\n"
            f"  Parint: {self.parint.name}"
        )

    def __repr__(self) -> str:
        """
        Return a string representation of the motor object that can be used to recreate it.

        Returns:
            str: A string representation of the motor.
        """
        return f"Motor({self.name}, {self.max_speed}, {self.max_acceleration}, {self.is_activate})"
    
class StepperMotor(motor):
    def __init__(self, name: str, max_speed: float, max_acceleration: float, steps_per_revolution: int, max_torqu: float):
        # Call the constructor of the base class (motor)
        super().__init__(name, max_speed, max_acceleration, max_torqu)
        self.steps_per_revolution = steps_per_revolution

    def __str__(self) -> str:
        # Call the __str__ method of the base class (motor)
        motor_info = super().__str__()
        return f"{motor_info}\n  Steps per Revolution: {self.steps_per_revolution}"

    def __repr__(self) -> str:
        # Call the __repr__ method of the base class (motor)
        motor_repr = super().__repr__()
        return f"{motor_repr}, Steps per Revolution: {self.steps_per_revolution}"

    
def main():
    test_motor = StepperMotor(f"Motor: 1", 100, 10, 7e4, 27)
    print(test_motor)

    print(__name__)


        
if __name__ == "__main__":
    main()