from Motors import *
from MotorManager import *


def test_MotorManager_adding_motors():
    """
    Test if MotorManager correctly adds motors with expected names.

    It creates a list of motors and initializes a MotorManager with those motors.
    Then, it checks if the names of the motors match the expected pattern.
    """
    test_motors = []
    for index in range(0, 6, 1):
        # Create StepperMotor objects with names like "Motor: 0", "Motor: 1", etc.
        test_motors.append(StepperMotor(f"Motor: {index}", 100, 10, 7e4))
    
    # Initialize a MotorManager with the test motors
    manager = motorManager(test_motors)
    
    for index, motor in enumerate(manager):
        # Check if the names of the motors match the expected pattern
        assert motor.name == f"Motor: {index}"


def test_MotorManager_activate():
    """
    Test if MotorManager correctly activates motors.

    It creates a list of motors and initializes a MotorManager with those motors.
    Then, it activates every even-indexed motor and checks if they are active.
    Finally, it checks if odd-indexed motors are inactive.
    """
    test_motors = []
    for index in range(0, 6, 1):
        # Create StepperMotor objects with names like "Motor: 0", "Motor: 1", etc.
        test_motors.append(StepperMotor(f"Motor: {index}", 100, 10, 7e4))
    
    # Initialize a MotorManager with the test motors
    manager = motorManager(test_motors)
    
    for index in range(0, 6, 1):
        if index % 2 == 0:
            # Activate every even-indexed motor
            manager.activate_motor(f"Motor: {index}")

    for index, motor in enumerate(manager):
        if index % 2 == 0:
            # Check if even-indexed motors are active
            assert motor.is_activate == True
        else:
            # Check if odd-indexed motors are inactive
            assert motor.is_activate == False

def test_MotorManager_deactivate():
    """
    Test if MotorManager correctly deactivates motors.

    It creates a list of motors, initializes a MotorManager with those motors, and activates all motors.
    Then, it deactivates every even-indexed motor and checks if they are inactive.
    Finally, it checks if odd-indexed motors are active.
    """
    test_motors = []
    for index in range(0, 6, 1):
        # Create StepperMotor objects with names like "Motor: 0", "Motor: 1", etc.
        test_motors.append(StepperMotor(f"Motor: {index}", 100, 10, 7e4))
    
    # Initialize a MotorManager with the test motors
    manager = motorManager(test_motors)
    
    for index in range(0, 6, 1):
        manager.activate_motor(f"Motor: {index}")

    for index in range(0, 6, 1):
        if index % 2 == 0:
            # Deactivate every even-indexed motor
            manager.deactivate_motor(f"Motor: {index}")

    for index, motor in enumerate(manager):
        if index % 2 == 0:
            # Check if even-indexed motors are inactive
            assert motor.is_activate == False
        else:
            # Check if odd-indexed motors are active
            assert motor.is_activate == True
