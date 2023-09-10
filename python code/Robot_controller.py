import numpy as np
import serial
import time
import logging



class RobotConnectionTimeout(Exception):
    pass

class Robot:


    DEFAULT_BAUD_RATE = 9600
    DEFAULT_TIMEOUT = 10

    def __init__(self, serial_port):
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

    def connect(self, baud_rate=None, timeout=None):
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

    def send_message(self, message):
        """
        Send a message from self.communications_message to the Arduino.

        Args:
            message_key (str): The  message sent to the arduno.
        """
        
        if self.serial and self.serial.is_open:
            if message is not None:
                self.serial.write(message.encode('utf-8'))
            else:
                print(f"Message not found and is emty")

    def close_connection(self):
        """
        Close the serial connection with the Arduino.
        """
        if self.serial and self.serial.is_open:
            self.serial.close()

class Messager:

    def __init__(self, robot):
        """
        Initialize the Messager object.

        :param robot: The robot's name or identifier.
        """
        if not robot:
            raise ValueError("Robot must be provided")
        self.robot = robot

    def ROTATE_JOINT(self, angles):
        """
        Create a message to rotate robot joints.

        :param angles: A list of angles (float values) for each joint.
        :return: The formatted message string.
        """
        if all(isinstance(angle, float) for angle in angles):
            message = f'ROTATE_JOINT {" ".join(map(str, angles))}\n'
            return message
        else:
            raise ValueError("All angles must be floats")

    def MOTOR_SPEED(self, speeds):
        """
        Create a message to set motor speeds for the robot.

        :param speed: A list of motor speeds (float values).
        :return: The formatted message string.
        """
        if all(isinstance(speed, float) for speed in speeds):
            message = f'MOTOR_SPEED {" ".join(map(str, speeds))}'
            return message
        else:
            raise ValueError("All speeds must be floats")



    
    

