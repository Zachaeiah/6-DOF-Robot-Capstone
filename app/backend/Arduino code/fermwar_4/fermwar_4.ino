
#include "Gloabls.h"
#include "CMD_PARSER.h"

// Constants for communication
const int baudRate = 115200;             // Serial communication baud rate
const int maxBufferSize = 128;           // Maximum buffer size for incoming data
const int COMMAND_INDEX_NOT_FOUND = -1;   // Index value indicating command not found
const char* seps = "\t,\n ;:";            // Delimiters for tokenizing the line string
const double ERROR_VALUE = -1;            // Error value
const int chipSelect = BUILTIN_SDCARD;
bool connected = false; // Define connected
int STATE = 0;          // Define STATE

// Structure to hold a motion path
typedef struct MOTION_PLAN {
  POINT_INTERP* motion;   // Pointer to motion plan
  int motion_len;         // Length of the motion plan
} MOTION_PLAN;

//----------------------------- Function Prototypes -------------------------------------------------------------------
void INITuC();                                  // Initialize microcontroller
void readSerialData(char*);                     // Read serial data


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Setup function called once on boot-up
// ARGUMENTS:   None
// RETURN VALUE: None
void setup() {
  Serial.begin(baudRate);   // Initialize serial communication
  Serial.println("setting up");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  STATE = IFIS;             // Set initial state to IFIS (ON BOOT UP)
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Loop function called repeatedly
// ARGUMENTS:   None
// RETURN VALUE: None
void loop() {

  char inputBuffer[maxBufferSize];          // Static array to hold input
  char* token;
  int commandIndex = -1;


  switch (STATE) {
    case IFIS:
      STATE = INIT;   // Transition to INIT state
      break;

    case INIT:
      INITuC();   // Initialize microcontroller
      break;

    case RESEVING_COMMAND:
      if (Serial.available() > 0) {
        readSerialData(inputBuffer);  // Get what's on the serial line
        STATE = PROCESSING_COMMAND;
      }
      break;

    case PROCESSING_COMMAND:
      makeStringUpperCase(inputBuffer);  // Make the input string uppercase
      commandIndex = getCommandIndex(inputBuffer);  // Get the command index
      token = strtok(inputBuffer, seps);   // Tokenize the input buffer
      token = strtok(NULL, "");            // Get the rest of the line as the next token
      processCommand(commandIndex, token); // Process the command
      break;

    case DATA_DUMP:
      // Add data dump logic here
      break;

    case EXECUTING_COMMAND:
      // Add executing command logic here
      break;

    case LOADING_PART:
      // Add loading part logic here
      break;

    case WEIGHING_DIPARCHER:
      // Add weighing dispatcher logic here
      break;

    case DEPARTING:
      // Add departing logic here
      break;

    case ARRIVING:
      // Add arriving logic here
      break;

    case DUMPING_ARRIVAL:
      // Add dumping arrival logic here
      break;

    case WEIGHING_ARRIVAL:
      // Add weighing arrival logic here
      break;

    case ESTOP:
      // Add emergency stop logic here
      break;

    case IDLE:
      // Add idle logic here
      break;

    case ERROR:
      // Add ERROR logic here
      break;

    default:
      dsprintString("unknown command!\n");
      break;
  }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Read serial data into inputBuffer
// ARGUMENTS:   inputBuffer - Buffer to store incoming serial data
// RETURN VALUE: None
void readSerialData(char* inputBuffer) {
  unsigned int bufferIndex = 0;
  char inChar;

  while (Serial.available() > 0) {
    inChar = Serial.read();
    if (inChar == '\n') {
      inputBuffer[bufferIndex] = '\0';  // Terminate the input string
    } else {
      // If the buffer is not full, add the incoming character
      if (bufferIndex < maxBufferSize - 2) {
        inputBuffer[bufferIndex++] = inChar;
      } else {
        dsprintString("Input buffer full, command ignored.");
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Initialize microcontroller
// ARGUMENTS:   None
// RETURN VALUE: None
void INITuC() {
  while (Serial.available() > 0) {
    String receivedMessage = Serial.readStringUntil('\n');

    // Check if the received message matches the expected connection message
    if (receivedMessage == "leftHand") {
      // Send a confirmation message back to Python
      dsprintString("rightHand");
      connected = true;          // Set the connection flag to true
      STATE = RESEVING_COMMAND;  // Transition to RECEIVING COMMAND state
    }
  }
}


