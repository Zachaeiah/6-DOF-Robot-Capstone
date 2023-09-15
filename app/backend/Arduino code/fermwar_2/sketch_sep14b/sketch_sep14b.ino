//---------------------------- library prototypes ---------------------------------------------------------------

#include <Servo.h>
#include <Wire.h>
#include <math.h>     // math functions
#include <string.h>   // string functions
#include <ctype.h>    // character functions
#include <stdbool.h>  // bool definitions

// Constants for communication
const int baudRate = 115200;        // Serial communication baud rate
const int maxBufferSize = 128;      // Maximum buffer size for incoming data
const int COMMAND_INDEX_NOT_FOUND = -1;  // Index value indicating command not found




#define MAX_LINE_SIZE 1002  // size of array to store a line from a file. 
                            // NOTE: 2 elements must be reserved for trailing '\n' and '\0'

// Enumeration for state machine
enum State {
  IDLE,
  PROCESSING
};

enum MOTOR_NAMES { BACE,
                   SHOULDER,
                   ELBOW,
                   ELBOWREVOLUT,
                   WRIST,
                   WRISTREVOLUT };

enum COMMAND_INDEX  // list of all command indexes
{
  ROTATE_JOINT,    // Set all six angles
  MOTOR_SPEED,     // set the robot rotashion speed
  GRIPPER_CLOSE,   // close the gripper
  GRIPPER_OPEN,    // open the gripper
  GRIPPER_OFFSET,  // Set the Gripper offset
  END,             // turn the robot off
  HOME,            // move the robot to its home posishion
  NUM_COMMANDS     // The Number of commands
};


enum CURRENT_ANGLES { GET_CURRENT_ANGLES,
                      UPDATE_CURRENT_ANGLES };  // used to get/update current SCARA angles

const char* seps = "\t,\n ;:";  // Delimiters for tokenizing the line string
const double ERROR_VALUE = -1;  // Error value

// structure to map command keyword string to a command index
typedef struct COMMAND {
  const int index;
  const char* strCommand;
} COMMAND;

// robot angles
typedef struct JOINT_VELOCITY {
  double Velocity[6] = { 0 };
} JOINT_VELOCITY;

// robot angles
typedef struct JOINT_ANGLES {
  double ThetaDeg[6] = { 0 };
} JOINT_ANGLES;

// robot angles
typedef struct ROTATE_JOINT_COMMAND {
  JOINT_ANGLES ANGLES = {0};
  JOINT_VELOCITY VELOCITY = {0};
} ROTATE_JOINT_COMMAND;

// tooltip coordinates
typedef struct TOOL_ORIENTATION {
  double theta1Deg, theta2Deg;
  double x, y, z;  // the poshion of the grippers active area
} TOOL_ORIENTATION;
State currentState = IDLE;

//----------------------------- Globals -------------------------------------------------------------------------------
// Global array of command keyword strings to command index associations
const COMMAND m_Commands[NUM_COMMANDS] = {
  { ROTATE_JOINT, "ROTATE_JOINT" },
  { MOTOR_SPEED, "MOTOR_SPEED" },
  { GRIPPER_CLOSE, "GRIPPER_CLOSE" },
  { GRIPPER_OPEN, "GRIPPER_OPEN" },
  { GRIPPER_OFFSET, "GRIPPER_OFFSET" },
  { END, "END" },
  { HOME, "HOME" }
};

//----------------------------- Function Prototypes -------------------------------------------------------------------

// Function to process an incoming command
void processCommand(int commandIndex, char*);

// Function to make a string uppercase
void makeStringUpperCase(char*);

// Function to get the command index from a string
int getCommandIndex(char*);

// Function to get or update the current robot angles
void robotAngles(JOINT_ANGLES*, int);

// Function to compute the nearest integer from a double value
int nint(double);

// Functions for angle conversion
double degToRad(double);
double radToDeg(double);
double mapAngle(double);

bool connected = false;             // Flag indicating connection status
char inputBuffer[maxBufferSize];    // Buffer for incoming data
int bufferIndex = 0;                // Index for reading data from the buffer

void setup() {
  Serial.begin(baudRate); // Initialize serial communication
}

// Process an incoming command
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
          connected = true;           // Set the connection flag to true
          currentState = PROCESSING;  // Transition to PROCESSING state
        }
      }
      break;

    case PROCESSING:
      // If connected, process incoming commands
      while (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming == '\n') {
          inputBuffer[bufferIndex] = '\0';  // Terminate the input string

          // Process the command
          Serial.println("Processing command");
          makeStringUpperCase(inputBuffer);  // Make the input string uppercase
          int commandIndex = getCommandIndex(inputBuffer);
          char* token = strtok(inputBuffer, seps);
          token = strtok(NULL, "");  // Get the rest of the line as the next token
          processCommand(commandIndex, token);

          // Reset the command buffer for the next command
          memset(inputBuffer, '\0', sizeof(inputBuffer));
          bufferIndex = 0;
          Serial.println("Done processing command");
        } else {
          // If the buffer is not full, add the incoming character
          if (bufferIndex < maxBufferSize - 2) {
            inputBuffer[bufferIndex++] = incoming;
          } else {
            Serial.println("Input buffer full, command ignored.");
          }
        }
      }
      break;
  }
}

// Process a command based on its index
void processCommand(int commandIndex, char* strCommandLine) {
  bool bSuccess = true;

  switch (commandIndex) {
    case ROTATE_JOINT:
      Serial.println("ROTATE_JOINT command selected");
      bSuccess = setJointAngles(strCommandLine);
      break;

    default:
      Serial.println("Unknown command!\n");
  }

  if (bSuccess) {
    Serial.println("Command sent to robot!\n\n");
  }
}

//---------------------------------------------------------------------------------------------------------------------

// Set the robot joint angles  
bool setJointAngles(char* strCommandLine) {
  const int paramaterNnumber = 12;  // Number of parameters to deal with
  bool goodCom = true;             // Flag indicating if the command is valid
  ROTATE_JOINT_COMMAND  AnglesVelOcity = { 0, 0 };  // Holds the angles of the command
  char* token = NULL;              // Token for parsing

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      if (i < 6){
        AnglesVelOcity.ANGLES.ThetaDeg[i] = (double)strtod(token, &token);
      }
      else if (i >= 6){
        AnglesVelOcity.VELOCITY.Velocity[i] = (double)strtod(token, &token);
      }

      if (token[0] != '\0') {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix the command!");
        goodCom = false;
        break;
      }
    } else {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }
  return goodCom;
}

//---------------------------------------------------------------------------------------------------------------------

// Get the index of a command from a string
int getCommandIndex(char* strLine) {
  char strLine2[MAX_LINE_SIZE];
  strcpy(strLine2, strLine);

  // Find which command matches the known commands
  for (int n = 0; n < NUM_COMMANDS; n++) {
    if (strstr(strLine2, m_Commands[n].strCommand) != NULL) {
      return m_Commands[n].index;
    }
  }

  // If the command was not found
  return -1;
}

//---------------------------------------------------------------------------------------------------------------------

// Convert a string to all uppercase characters
void makeStringUpperCase(char* str) {
  if (str == NULL) return;

  for (size_t i = 0; i < strlen(str); i++) {
    str[i] = (char)toupper(str[i]);
  }
}

//---------------------------------------------------------------------------------------------------------------------

// Get or update current robot shoulder and elbow angles
void robotAngles(JOINT_ANGLES* pAngles, int getOrUpdate) {
  static JOINT_ANGLES currentAngles = {0,0,0,0,0,0};

  if (pAngles == NULL) {
    Serial.println("NULL JOINT_ANGLES pointer! (robotAngles)");
    return;
  }

  if (getOrUpdate == UPDATE_CURRENT_ANGLES) {
    currentAngles = *pAngles;
  } else if (getOrUpdate == GET_CURRENT_ANGLES) {
    *pAngles = currentAngles;
  } else {
    Serial.println("Unknown value for getOrUpdate (robotAngles)");
  }
}

//---------------------------------------------------------------------------------------------------------------------

// Map an angle in radians into a range understood by the robot
double mapAngle(double angRad) {
  angRad = fmod(angRad, 2.0 * PI);  // Put in range -2*PI <= ang <= +2*PI

  // Map into range -PI <= ang <= +PI
  if (angRad > PI) {
    angRad -= 2.0 * PI;
  } else if (angRad < -PI) {
    angRad += 2.0 * PI;
  }

  return angRad;
}

//---------------------------------------------------------------------------------------------------------------------

// Convert degrees to radians
double degToRad(double angDeg) {
  return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------

// Convert radians to degrees
double radToDeg(double angRad) {
  return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------------------------------------

// Compute the nearest integer to a double value
int nint(double d) {
  return (int)floor(d + 0.5);
}

//---------------------------------------------------------------------------------------------------------------------

// Function to map a float value from one range to another
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  float result = 0;
  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return result;
}

//---------------------------------------------------------------------------------------------------------------------

// Function to calculate the length of a string up to a maximum length
size_t strnlen(const char* s, size_t len) {
  size_t i = 0;
  for (; i < len && s[i] != '\0'; ++i)
    ;
  return i;
}
