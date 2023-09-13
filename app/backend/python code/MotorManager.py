import Motors

class MotorManager:
    def __init__(self, motors: Motors.motor):
        """
        Initialize the MotorManager.

        :param motors: A list of Motor objects.
        """
        self.motors = motors
        # Create a dictionary mapping motor names to their instances for quick lookup
        self.motor_name_to_instance = {motor.name: motor for motor in motors}

    def activate_motor(self, motor_name: str):
        """
        Activate a motor by its name.

        :param motor_name: The name of the motor to activate.
        """
        motor = self.motor_name_to_instance.get(motor_name)
        if motor:
            if not motor.is_activate:
                motor.activate(True)
                print(f"Motor '{motor_name}' is now active.")
            else:
                print(f"Motor '{motor_name}' is already active.")
        else:
            print(f"Motor '{motor_name}' not found.")

    def deactivate_motor(self, motor_name: str):
        """
        Deactivate a motor by its name.

        :param motor_name: The name of the motor to deactivate.
        """
        motor = self.motor_name_to_instance.get(motor_name)
        if motor:
            if motor.is_activate:
                motor.activate(False)
                print(f"Motor '{motor_name}' is now deactivated.")
            else:
                print(f"Motor '{motor_name}' is already deactivated.")
        else:
            print(f"Motor '{motor_name}' not found.")

    def __iter__(self):
        # Allow iteration over the MotorManager, which will iterate over its Motor instances
        return iter(self.motors)

def main():
    test_motors = []
    for i in range(0, 6, 1):
        test_motors.append(Motors.StepperMotor(f"Motor: {i}", 100, 10, 7e4))
        
    manager = MotorManager(test_motors)

    manager.activate_motor("Motor: 3")

    for motor in manager:
        print(motor.is_activate)

if __name__ == '__main__':
    main()