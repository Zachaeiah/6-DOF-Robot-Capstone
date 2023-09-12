import Robot

class messager:
    """
    A class responsible for creating and sending messages to control robot joints.

    Attributes:
        robot (Robot): The robot instance associated with the Messager.

    Methods:
        ROTATE_JOINT(angles: list[float], speeds: list[float]) -> str:
            Create a message to rotate robot joints at specified speeds.

    """

    def __init__(self, robot: Robot):
        """
        Initialize the Messager object.

        Args:
            robot (Robot): The robot instance to which messages will be sent.
        """
        if not robot:
            raise ValueError("Robot must be provided")
        self.robot = robot

    def ROTATE_JOINT(self, angles: list[float], speeds: list[float]) -> str:
        """
        Create a message to rotate robot joints at one speed for each motor.

        Args:
            angles (list[float]): A list of angles (float values) for each joint.
            speeds (list[float]): A list of motor speeds (float values).

        Returns:
            str: The formatted message string.

        Raises:
            ValueError: If either angles or speeds are not lists of floats.
        """
        if all(isinstance(angle, float) for angle in angles) and all(isinstance(speed, float) for speed in speeds):
            message = f'ROTATE_JOINT {" ".join(map(str, angles))} {" ".join(map(str, speeds))}\n'
            return message
        else:
            raise ValueError("Both angles and speeds must be lists of floats")
        
def main():
    print(__name__)
        
if __name__ == "__main__":
    main()