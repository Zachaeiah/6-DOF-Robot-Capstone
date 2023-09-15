from Motors import *

class motorManager:
    def __init__(self, motors: motor):
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

    def add_motor(self, motor: motor):
        """
        Add a new motor to the MotorManager.

        :param motor: The Motor object to add.
        """
        if motor.name not in self.motor_name_to_instance:
            self.motors.append(motor)
            self.motor_name_to_instance[motor.name] = motor
            print(f"Motor '{motor.name}' added.")
        else:
            print(f"Motor '{motor.name}' already exists.")

    def remove_motor(self, motor_name: str):
        """
        Remove a motor by its name.

        :param motor_name: The name of the motor to remove.
        """
        motor = self.motor_name_to_instance.get(motor_name)
        if motor:
            self.motors.remove(motor)
            del self.motor_name_to_instance[motor_name]
            print(f"Motor '{motor_name}' removed.")
        else:
            print(f"Motor '{motor_name}' not found.")

    def __repr__(self):
        """
        Return a string representation of the MotorManager that can be used to recreate it.
        """
        motor_list = [f"Motor(name='{motor.name}', is_activate={motor.is_activate})" for motor in self.motors]
        return f"motorManager(motors=[{', '.join(motor_list)}])"

    def __str__(self):
        """
        Return a human-readable string representation of the MotorManager.
        """
        motor_list = [f"Motor '{motor.name}' {'active' if motor.is_activate else 'inactive'}" for motor in self.motors]
        return '\n'.join(motor_list)

    def __iter__(self):
        # Allow iteration over the MotorManager, which will iterate over its Motor instances
        return iter(self.motors)


def main():
    test_motors = []
    for i in range(0, 6, 1):
        test_motors.append(StepperMotor(f"Motor: {i}", 100, 10, 7e4))
        
    manager = motorManager(test_motors)

    manager.activate_motor("Motor: 3")

    for motor in manager:
        print(motor.is_activate)

if __name__ == '__main__':
    main()