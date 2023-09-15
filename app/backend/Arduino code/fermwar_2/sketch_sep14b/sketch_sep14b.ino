const int baudRate = 115200; // Increase the baud rate if supported
const int maxBufferSize = 128; // Adjust this as needed

bool connected = false;
char inputBuffer[maxBufferSize];
int bufferIndex = 0;

enum State { IDLE, PROCESSING };
State currentState = IDLE;

void setup() {
  Serial.begin(baudRate);
}

void processIncomingCommand(const char* command) {
  // Process the command here
  Serial.print("Processing command: ");
  Serial.println(command);
  // Add your command processing logic
}

void loop() {
  switch (currentState) {
    case IDLE:
      // Wait until the Python code sends the connection message
      while (Serial.available() > 0) {
        String receivedMessage = Serial.readStringUntil('\n');

        // Check if the received message matches the expected connection message
        if (receivedMessage == "leftHand") {
          // Send a confirmation message back to Python
          Serial.println("rightHand");
          connected = true; // Set the connection flag to true
          currentState = PROCESSING; // Transition to PROCESSING state
        }
      }
      break;

    case PROCESSING:
      // If connected, process incoming commands
      while (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == '\n') {
          // Terminate the input string
          inputBuffer[bufferIndex] = '\0';
          bufferIndex = 0;

          // Process the command
          processIncomingCommand(inputBuffer);
        } else {
          if (bufferIndex < maxBufferSize - 1) { // Check if there is room in the input buffer
            inputBuffer[bufferIndex++] = incoming; // Add the incoming character and increment index
          } else {
            Serial.println("Input buffer full, command ignored.");
          }
        }
      }
      break;
  }
}
