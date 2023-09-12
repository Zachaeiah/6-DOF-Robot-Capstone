import numpy as np
import serial
import time
import logging
import math



import serial
import time
import logging

class RobotConnectionTimeout(Exception):
    pass

class Robot:
    """
    A class representing a robot with a serial connection to an Arduino.

    Attributes:
        port (str): The serial port used for communication.
        baud_rate (int): The default baud rate for serial communication.
        timeout (int): The default maximum timeout for connection confirmation.
        communications_message (dict): A dictionary containing communication-related messages and status.
        serial (serial.Serial): The serial connection object.

    Methods:
        connect(baud_rate: int, timeout: int): Establish a serial connection with the Arduino.
        send_message(message: str): Send a message to the Arduino.
        close_connection(): Close the serial connection with the Arduino.
    """

    DEFAULT_BAUD_RATE = 9600
    DEFAULT_TIMEOUT = 10

    def __init__(self, serial_port: str):
        """
        Initialize a Robot instance.

        Args:
            serial_port (str): The serial port for communication.
        """
        self.port = serial_port
        self.baud_rate = self.DEFAULT_BAUD_RATE
        self.timeout = self.DEFAULT_TIMEOUT
        self.communications_message = {
            "isConnected": False,
            "connection_message_sent": "leftHand\n",
            "connection_message_received": "rightHand",
            "battery_status": "Unknown",
            "temperature": "N/A",
        }
        self.serial = None

    def connect(self, baud_rate: int = None, timeout: int = None):
        """
        Establish a serial connection with the Arduino.

        Args:
            baud_rate (int, optional): The baud rate for serial communication.
            timeout (int, optional): The maximum time to wait for a connection confirmation.
        """
        if baud_rate is None:
            baud_rate = self.baud_rate
        if timeout is None:
            timeout = self.timeout

        try:
            # Check if the specified serial port is available
            self.serial = serial.Serial(self.port, baud_rate, timeout=1)
            # Wait for a moment to let the Arduino initialize
            time.sleep(2)

            # Send the connection message to the Arduino
            self.send_message(self.communications_message["connection_message_sent"])

            start_time = time.time()

            while not self.communications_message["isConnected"]:
                # Read data from the Arduino
                data = self.serial.readline().decode('utf-8').strip()
                # Check if the Arduino has sent a confirmation
                if data == self.communications_message["connection_message_received"]:
                    print("Arduino has confirmed the connection.")
                    self.communications_message["isConnected"] = True
                # Check if the timeout has been reached
                if time.time() - start_time > timeout:
                    raise RobotConnectionTimeout("Timeout reached. No connection confirmation received.")

        except serial.SerialException as e:
            logger = logging.getLogger(__name__)
            logger.error(f"Serial port error: {e}")
            raise  # Re-raise the exception to indicate a failure

    def send_message(self, message: str):
        """
        Send a message from self.communications_message to the Arduino.

        Args:
            message (str): The message sent to the Arduino.
        """
        
        if self.serial and self.serial.is_open:
            if message is not None:
                self.serial.write(message.encode('utf-8'))
            else:
                print(f"Message not found and is empty")

    def close_connection(self):
        """
        Close the serial connection with the Arduino.
        """
        if self.serial and self.serial.is_open:
            self.serial.close()


class Messager:
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

        
class Motor:
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

    def set_velocity_profile(self, profile):
        """
        Set the velocity profile for the motor.

        Args:
            profile: The velocity profile to set.
        """
        self.velocity_profile = profile

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
        return f"Motor({self.name}, {self.max_speed}, {self.max_acceleration}, {self.steps_per_revolution})"

class Linear:
    def __init__(self, slope: float, offset: float):
        """
        Initialize a linear function.

        :param slope: The slope of the linear function.
        :param offset: The offset of the linear function.
        """
        self.slope = slope
        self.offset = offset

    def velocity(self, time: float) -> float:
        """
        Calculate the velocity at a given time using the linear function.

        :param time: The time at which to calculate the velocity.
        :return: The calculated velocity.
        """
        return self.slope * time + self.offset

class Sigmoid:
    def __init__(self, growth_rate: float, time_shift: float):
        """
        Initialize a sigmoid function.

        :param growth_rate: The growth rate of the sigmoid function.
        :param time_shift: The time shift of the sigmoid function.
        """
        self.growth_rate = growth_rate
        self.time_shift = time_shift

    def velocity(self, time: float) -> float:
        """
        Calculate the velocity at a given time using the sigmoid function.

        :param time: The time at which to calculate the velocity.
        :return: The calculated velocity.
        """
        try:
            return 1 / math.exp(-self.growth_rate * (time - self.time_shift))
        except OverflowError as e:
            raise ValueError(f"OverflowError: {e}. Growth rate too high for the given time and time shift")


        
def main():
    robot = Robot('CON3')
    messager = Messager(robot)

    angles = [90.0,90.0,90.0,90.0,90.9,90.9]
    speeds = [10.0,10.0,10.0,10.0,10.0,10.0]

    motor = Motor('test Motor', 100, 10, 70000)
    print(motor)

    
    #print(messager.ROTATE_JOINT(angles, speeds))


if __name__ == "__main__":
    main()




    
    

