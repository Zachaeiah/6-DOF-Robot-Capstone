import serial
import time
import logging

class RobotConnectionTimeout(Exception):
    pass

class Robot:
    DEFAULT_BAUD_RATE = 9600
    DEFAULT_TIMEOUT = 10

    def __init__(self, serial_port: str):
        self.port = serial_port
        self.baud_rate = self.DEFAULT_BAUD_RATE
        self.timeout = self.DEFAULT_TIMEOUT
        self.is_connected = False
        self.serial = None

    def connect(self, baud_rate: int = None, timeout: int = None):
        if baud_rate is None:
            baud_rate = self.baud_rate
        if timeout is None:
            timeout = self.timeout

        try:
            # Reset the connection status
            self.is_connected = False

            self.serial = serial.Serial(self.port, baud_rate, timeout=1)
            time.sleep(2)

            self.send_message("leftHand\n")

            start_time = time.time()

            while not self.is_connected:
                data = self.serial.readline().decode('utf-8').strip()
                if data == "rightHand":
                    print("Arduino has confirmed the connection.")
                    self.is_connected = True
                if time.time() - start_time > timeout:
                    raise Exception("Timeout reached. No connection confirmation received.")

        except Exception as e:
            logger = logging.getLogger(__name__)
            logger.error(f"Serial port error: {e}")
            raise

    def send_message(self, message: str):
        if self.serial and self.serial.is_open:
            if message is not None:
                self.serial.write(message.encode('utf-8'))
            else:
                print(f"Message not found and is empty")

    def close_connection(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.is_connected = False

def main():
    robot = Robot("COM3")  

    print("connecting")
    try:
        robot.connect()

        if robot.is_connected:
            print("Connection with Arduino established.")
            # Your code here - you can perform actions with the robot
        else:
            print("Failed to establish a connection with Arduino.")

    except RobotConnectionTimeout as e:
        print(f"Error: {e}")
    finally:
        robot.close_connection()

    print("connecting agine")
    robot.connect()

if __name__ == "__main__":
    main()
