
#include "Gloabls.h"
#include "CMD_PARSER.h"
// #include "Pannle.h"

// Constants for communication
const int baudRate = 115200;            // Serial communication baud rate
const int chipSelect = BUILTIN_SDCARD;  // where to stor the log file
const char* seps = "\t,\n ;:";          // Declare seps as an external constant
int error_index = NO_ERROR;             // the indx of witch error to handle
bool ReadDataDump = false;              // has the memory bean allowcated for a moshion
int STATE = 0;                          // Current state of the system
int ErrorState = 0;                     // the state an error happend

// Structure to hold a motion path
typedef struct MOTION_PLAN {
  POINT_INTERP* motion;  // Pointer to motion plan
  int motion_len;        // Length of the motion plan
} MOTION_PLAN;


//----------------------------- Function Prototypes -------------------------------------------------------------------
void INITuC();  // Initialize microcontroller

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Setup function called once on boot-up
// ARGUMENTS:   None
// RETURN VALUE: None
void setup() {

  Serial.begin(baudRate);  // Initialize serial communication

  for (int pin = 0; pin < 6; pin++) {
    pinMode(SepperDirPins[pin], OUTPUT);
    pinMode(StepperStepsPins[pin], OUTPUT);
  }

  //  flush Serial input
  while (Serial.available()) Serial.read();

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  STATE = INIT;
  // Set initial state to IFIS (ON BOOT UP)

  Serial.println("setting up\n");  // send a mesage telling it uP its geting ready
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Loop function called repeatedly
// ARGUMENTS:   None
// RETURN VALUE: None
void loop() {

  char inputBuffer[maxBufferSize];  // Static array to hold input
  char* token;
  int commandIndex = COMMAND_INDEX_NOT_FOUND;
  switch (STATE) {

    case INIT:
      INITuC();
      // initPannle();
      break;

    case IDLE:
      if (Serial.available() > 0) {
        readSerialData(inputBuffer);  // Get what's on the serial line
        STATE = PROCESSING_COMMAND;
      }

      break;

    case PROCESSING_COMMAND:
      makeStringUpperCase(inputBuffer);       // Make the input string uppercase
      token = strtok(inputBuffer, seps);      // get the command string first
      commandIndex = getCommandIndex(token);  // Get the command index

      // Handle case when no command is provided
      if (commandIndex == COMMAND_INDEX_NOT_FOUND) {
        print_error(COMMAND_INDEX_NOT_FOUND, "PROCESSING_COMMAND STATE");
        ErrorState = STATE;
        STATE = ERROR;  // Transition to the ERROR state
        break;
      }

      // Get the rest of the line as the next token
      token = strtok(NULL, seps);

      // Handle case when no command arguments are provided
      if (token == NULL) {
        print_error(INVALID_COMMAND_ARGUMENTS, "PROCESSING_COMMAND STATE");
        ErrorState = STATE;
        STATE = ERROR;  // Transition to the ERROR state
        break;
      }

      // Process the command
      processCommand(commandIndex, token);
      STATE = IDLE;
      break;

    case ERROR:
      STATE = IDLE;
      ErrorState = 0;
      break;

    default:
      dsprintf("unknown STATE!\n");
      break;
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
      dsprintf("rightHand\n");
      STATE = IDLE;  // Transition to RECEIVING COMMAND state
    }
  }
  //  flush Serial input
  while (Serial.available()) Serial.read();
}
