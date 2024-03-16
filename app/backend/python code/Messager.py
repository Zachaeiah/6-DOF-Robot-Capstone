import serial
import threading
import time
from queue import LifoQueue

class RobotConnectionTimeout(Exception):
    pass

class Mesageer:
    """Class for managing communication with a robot over a serial port."""

    DEFAULT_BAUD_RATE = 115200
    DEFAULT_TIMEOUT = 10

    def __init__(self, serial_port: str):
        """Initialize the Mesageer instance.

        Args:
            serial_port (str): Serial port name.
        """
        self.port = serial_port
        self.baud_rate = self.DEFAULT_BAUD_RATE
        self.timeout = self.DEFAULT_TIMEOUT
        self.is_connected = False
        self.serial = None
        self.read_thread = None
        self._stop_reading = threading.Event()
        self.message_stack = LifoQueue()

    def connect(self, baud_rate: int = None, timeout: int = None):
        """Establish a connection with the robot.

        Args:
            baud_rate (int, optional): Baud rate for serial communication. Defaults to None.
            timeout (int, optional): Timeout for the connection attempt. Defaults to None.

        Raises:
            Exception: Raised if the connection attempt times out.
        """
        if baud_rate is None:
            baud_rate = self.baud_rate
        if timeout is None:
            timeout = self.timeout

        try:
            # Open the serial port
            self.serial = serial.Serial(self.port, baud_rate, timeout=1)
            time.sleep(1)

            start_time = time.time()

            # Wait for confirmation from the robot
            while not self.is_connected:
                self.serial.write("leftHand\n".encode('utf-8'))
                data = self.serial.readline().decode('utf-8').strip()
                if data == "rightHand":
                    print("Teensy has confirmed the connection.")
                    self.is_connected = True

                if time.time() - start_time > timeout:
                    raise Exception("Timeout reached. No connection confirmation received.")

            # Initialize the message stack
            self.message_stack = LifoQueue()

            # Start a separate thread to continuously read and put data into the message stack
            self.read_thread = threading.Thread(target=self._read_and_put_data)
            self.read_thread.daemon = True
            self.read_thread.start()

        except Exception as e:
            print(f"Error connecting to the robot: {e}")
            # Initialize the message stack even if the connection attempt fails
            self.message_stack = LifoQueue()
            raise

    def send_message(self, message: str):
        """Send a message to the robot and wait for an expected response.

        Args:
            message (str): Message to send.
        """
        try:
            if self.serial and self.serial.is_open:
                if message is not None:
                    if not message.endswith("\n"):
                        message += "\n"
                    self.serial.write(message.encode('utf-8'))
                else:
                    print("Message not found and is empty")
            else:
                print("Serial port is not open")
        except Exception as e:
            print(f"An error occurred while sending message: {e}")

    def _read_and_put_data(self):
        """Read data from the serial port and put it into the message stack."""
        try:
            response_buffer = ""
            while not self._stop_reading.is_set():
                data = self.serial.read().decode('utf-8')
                if data:
                    response_buffer += data
                    if data.endswith("\n"):
                        self.message_stack.put(response_buffer.strip())
                            
                        response_buffer = ""
        except Exception as e:
            print(f"An error occurred while reading data: {e}")

    def get_latest_message(self) -> str:
        """Get the latest message from the message stack.

        Returns:
            str: Latest message from the stack.
        """
        if not self.message_stack.empty():
            return self.message_stack.get()
        else:
            return None

    def close_connection(self):
        """Close the serial connection."""
        if self.serial and self.serial.is_open:
            self._stop_reading.set()
            self.read_thread.join()
            self.serial.close()
            self.is_connected = False

    def __str__(self):
        return f"Robot connected to port {self.port}\n\nConnected: {self.is_connected}"

    def __repr__(self):
        return f"Robot('{self.port}')"

def handle_response(expected_r: str = None, expected_prefix: str = None, dataWrite: list = [], message_stack: LifoQueue = None):
    """Handle response based on expected response or prefix.

    Args:
        expected_r (str, optional): Expected response. Defaults to None.
        expected_prefix (str, optional): Expected prefix. Defaults to None.
        dataWrite (list, optional): List to write data when expected_prefix is found. Defaults to [].
        message_stack (LifoQueue, optional): Stack of messages. Defaults to None.
    """
    if expected_r is None and expected_prefix is None:
        raise print("must give an Expected response or Expected prefix")
    
    while True:
        response = message_stack.get()  # Get the latest response from the message stack
        if response is None:  # If response is None, continue to wait for the next response
            continue


        if expected_r is not None and expected_prefix is None:  # If expected_r is provided but expected_prefix is not
            if response == expected_r:  # Check if the response matches the expected response
                print(response)  # Print the response
                return  # Exit the function since the expected response is received
            else:
                print(f">>> {response}")

        elif expected_r is None and expected_prefix is not None:  # If expected_r is not provided but expected_prefix is
            if response.startswith(expected_prefix):  # Check if the response starts with the expected prefix
                dataWrite.extend(response.split())  # Split the response and add it to the dataWrite list
                return  # Exit the function since the response with the expected prefix is received
            else:
                print(f">>> {response}")

        


def main():
    MSG = Mesageer("COM12")

    MSG.connect()

    expected_response = "MoshionState changed to: 1"
    MSG.send_message("R_MOVES 1")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

    expected_response = "storing 1"
    MSG.send_message("R_MOSHION 1")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

    expected_response = "MoshionState changed to: 2"
    MSG.send_message("-29859, 0, 0, 0, 0, 0, 277777")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

    expected_response = "MoshionState changed to: 0"
    MSG.send_message("R_EXECUTE 2")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

    





    expected_response = "MoshionState changed to: 1"
    MSG.send_message("R_MOVES 1")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

    expected_response = "storing 1"
    MSG.send_message("R_MOSHION 1")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

    expected_response = "MoshionState changed to: 2"
    MSG.send_message("29859, 0, 0, 0, 0, 0, 277777")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

    expected_response = "MoshionState changed to: 0"
    MSG.send_message("R_EXECUTE 1")
    handle_response(expected_response, None, None,MSG.message_stack) # weight until i get the my response

 

    # MSG.close_connection()

if __name__ == "__main__":
    main()
