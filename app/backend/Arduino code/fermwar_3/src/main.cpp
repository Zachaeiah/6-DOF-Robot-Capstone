#include <Arduino.h>
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
enum States {
  IFIS,               // ON BOOT UP 
  IDLE,               // IDLE STATE
  INIT,               // INITSHLE SETUP
  RESEVING_COMMAND,   // A COMMAND IT COMMING IN ON SERIAL
  COMMAND_PREP,       // PREPARING FOR THE DATAT SUM WILL PROVIDE THE LENGHTH OF THE COMMING MOSHION
  DATA_DUMP,          // THE DATA DUM OF THE MOSHION
  EXECUTING_COMMAND,  // EXSUDTING THE COMMAND THAT WAS SENT FROM THE uP
  LOADING_PART,       // loading the part from the part bin into the basket
  WEIGHING_DIPARCHER, // weight the full basket of parts
  DIPARTING,          // Send the full basket out
  ARIVING,            // A part has arrived at arrivles
  DUMPING_ARIVLE,     // empting the part out of the celled basket
  WEIGHING_ARIVLE,    // weighing the basket with the exses parts
  ESTOP,              // emergance stop state
};

enum commands  // list of all command indexes
{
  R_MOVES,       // The number of move to be in the comming datadum
  R_DELAY,       // the delay beatween each point
  R_MOSHION,     // a moshion interpolashion point
  R_POOR_ACT,    // starting unloading the part into the basket
  R_POOR_COM,    // the poor has be done
  R_HAND,        // uP is redy to tranmit
  L_HAND,        // uC is ready to resevs
  NUM_COMMANDS   // The Number of commands
};

enum CURRENT_ANGLES { GET_CURRENT_ANGLES,
                      UPDATE_CURRENT_ANGLES };  // used to get/update current SCARA angles

const char* seps = "\t,\n ;:";  // Delimiters for tokenizing the line string
const double ERROR_VALUE = -1;  // Error value

// structure to map command keyword string to a command index
typedef struct COMMAND {
  int index;
  const char* strCommand;
} COMMAND;

// structure to hold a moshion point
typedef struct JOINT_INTERP {
  int paramaterCnumber = 2;
  int STEP_CNT;
  int STEP_FR;
} JOINT_INTERP;

// structure to hold a moshion point
typedef struct POINT_INTERP {
  int paramaterCnumber = 2;
  JOINT_INTERP INTEPR[6] = {0};
 int INDEX;
} POINT_INTERP;

// structure to hold a moshion path
typedef struct MOSHION_PLAN {
  int paramaterCnumber = 2;
  POINT_INTERP* moshion;
  int moshion_len;
} MOSHION_PLAN;


//----------------------------- Globals -------------------------------------------------------------------------------
// Global array of command keyword strings to command index associations
const COMMAND m_Commands[NUM_COMMANDS] = {
  {R_MOVES, "R_MOVES" },
  {R_DELAY, "R_DELAY" },
  {R_MOSHION, "R_MOSHION" },
  {R_POOR_ACT, "R_POOR_ACT" },
  {R_POOR_COM, "R_POOR_COM" },
  {R_HAND, "R_HAND" },
  {L_HAND, "L_HAND" }
};

//----------------------------- Function Prototypes -------------------------------------------------------------------
void processCommand(int commandIndex, char*); // Function to process an incoming command
void makeStringUpperCase(char*); // Function to make a string uppercase
int getCommandIndex(char*); // Function to get the command index from a string
int nint(double); // Function to compute the nearest integer from a double value

void INITuC();
void readSerialData(char* buffer, size_t bufferSize);

bool connected = false;             // Flag indicating connection status
char inputBuffer[maxBufferSize];    // Buffer for incoming data
int bufferIndex = 0;                // Index for reading data from the buffer

//---------------------------------------------------------------------------------------------------------------------
int STATE = IFIS;
char receivedString[maxBufferSize]; // Static array to hold input

void setup() {
  // Initialization code here
}

void loop() {

  Serial.println("hellow world");

  // switch (STATE)
  // {
  // case IFIS:
  //   STATE = INIT;
  //   break;
  
  // case INIT:
  //   INITuC();
  //   break;
  
  // case RESEVING_COMMAND:
  //   Serial.println("waiting to resec command");
  //   if (Serial.available() > 0) {
      
  //     readSerialData(receivedString, sizeof(receivedString));
  //     Serial.print("Received: ");
  //     Serial.println(receivedString);
  //   }
  //   break;
  
  // case COMMAND_PREP:
  //   break;

  // case DATA_DUMP:
  //   break;

  // case EXECUTING_COMMAND:
  //   break;
  
  // case LOADING_PART:
  //   break;
  
  // case WEIGHING_DIPARCHER:
  //   break;

  // case DIPARTING:
  //   break;
  
  // case ARIVING:
  //   break;

  // case DUMPING_ARIVLE:
  //   break;

  // case WEIGHING_ARIVLE:
  //   break;

  // case ESTOP:
  //   break;

  // case IDLE:
  //   break;
  
  // default:
  //   break;
  // }
}
//---------------------------------------------------------------------------------------------------------------------

void INITuC(){
  Serial.begin(baudRate); // Initialize serial communication
  Serial.println("setting up");
  STATE = RESEVING_COMMAND;
}

void readSerialData(char* buffer, size_t bufferSize) {
  unsigned int index = 0; // Index to keep track of position in the buffer
  char inChar;

  while (Serial.available()) {
    inChar = Serial.read(); // Read a character
    if (inChar == '\n') { // If newline character is received
      buffer[index] = '\0'; // Null-terminate the string
      break;
    }
    if (index < bufferSize - 1) { // Check if there's enough space in the buffer
      buffer[index++] = inChar; // Append the character to the buffer
    } else {
      // Buffer overflow protection, reset index and discard current message
      index = 0;
    }
  }
}

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