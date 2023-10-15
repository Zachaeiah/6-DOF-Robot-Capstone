class motor:
    def __init__(self, name: str, index: int, max_speed: float, max_acceleration: float, max_torqu: float):
        """_summary_

        Args:
            name (str): Name of the motor
            index (int): index of the motor
            max_speed (float): top alowable speed 
            max_acceleration (float): maximum acceleration
            max_torqu (float): maximum torqu
        """
        
        self.name = name
        self.index = index
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.is_activate = False
        self.max_torqu = max_torqu

    def SetMaxSpeed(self, NewSpeed: float):
        self.max_speed = NewSpeed

    def setMaxAcceleration(self, NewAcceleration):
        self.max_acceleration = NewAcceleration

    def activate(self, state = True):
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
            f"  index: {self.index}\n"
            f"  Max Speed: {self.max_speed} RPM\n"
            f"  Max Acceleration: {self.max_acceleration} RPM/s\n"
            f"  Max torqu: {self.max_torqu}\n"
            f"  Activate state: {self.is_activate}\n"
        )
    
class StepperMotor(motor):
    def __init__(self, name: str, index: int, max_speed: float, max_acceleration: float, max_torqu: float, steps_per_revolution: int):
        # Call the constructor of the base class (motor)
        super().__init__(name, index, max_speed,  max_acceleration, max_torqu)
        self.steps_per_revolution = steps_per_revolution

    def __str__(self) -> str:
        # Call the __str__ method of the base class (motor)
        motor_info = super().__str__()
        return f"{motor_info}  Steps per Revolution: {self.steps_per_revolution}"

    
def main():
    test_motor = StepperMotor(f"Motor: 1", 1, 100, 10, 1, 70_000)
    print(test_motor)

if __name__ == "__main__":
    main()