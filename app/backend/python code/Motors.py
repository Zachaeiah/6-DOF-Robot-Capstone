

class motor:
    """
    A class representing a motor.

    Args:
        name (str): The name or identifier of the motor.
        max_speed (float): The maximum speed of the motor in RPM (Revolutions Per Minute).
        max_acceleration (float): The maximum acceleration of the motor in RPM/s (Revolutions Per Minute per second).
        steps_per_revolution (int): The number of steps per revolution for the motor.
    """

    def __init__(self, name: str, max_speed: float, max_acceleration: float, steps_per_revolution: int):
        """
        Initialize a Motor instance with the specified parameters.

        Args:
            name (str): The name or identifier of the motor.
            max_speed (float): The maximum speed of the motor in RPM (Revolutions Per Minute).
            max_acceleration (float): The maximum acceleration of the motor in RPM/s (Revolutions Per Minute per second).
            steps_per_revolution (int): The number of steps per revolution for the motor.
        """
        self.name = name
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.steps_per_revolution = steps_per_revolution
        self.velocity_profile = None
        self.is_activate = False

    def set_velocity_profile(self, profile):
        """
        Set the velocity profile for the motor.

        Args:
            profile: The velocity profile to set.
        """
        self.velocity_profile = profile
    
    def activate(self, state = True):
        self.is_activate = True

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
            f"  Steps per Revolution: {self.steps_per_revolution}"
        )

    def __repr__(self) -> str:
        """
        Return a string representation of the motor object that can be used to recreate it.

        Returns:
            str: A string representation of the motor.
        """
        return f"Motor({self.name}, {self.max_speed}, {self.max_acceleration}, {self.steps_per_revolution}), {self.is_activate}"
    
def main():
    print(__name__)
        
if __name__ == "__main__":
    main()