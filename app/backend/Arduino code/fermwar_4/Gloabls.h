#ifndef GLOABKS_H  // Header guard to prevent multiple inclusion
#define GLOABKS_H


#include <math.h>     // math functions
#include <stdlib.h>
#include <string.h>   // string functions
#include <stdarg.h>   // Include for variable argument lists
#include <ctype.h>    // character functions
#include <stdbool.h>  // bool definitions
#include <SD.h>       // SD card library
#include <Arduino.h>  // Arduino core library

extern bool ReadDataDump;  // Declare connected as an external variable
extern int error_index;    // Index of the current error
extern int STATE;          // Current state of the state machine
extern int ErrorState;     // State at which an error occurred
extern File dataFile;      // File for error logging
extern const char* seps;   // String delimiter for tokenization
const int maxBufferSize = 1024;          // Maximum buffer size for incoming data
const char StepperPins[] = {0, 1, 2, 3, 4, 5};

// Error codes
#define NO_ERROR 0                      // No error
#define COMMAND_INDEX_NOT_FOUND -1      // Command index does not exist
#define INPUT_BUFFER_FULL -2            // Input buffer is full, command was too big
#define MISSING_COMMAND -3              // No command provided
#define INVALID_COMMAND_ARGUMENTS -4    // No command arguments provided
#define EXECUTEFUNCTION_FAILD -5        // Command execution failed
#define MEMORY_ALLOCATION_FAILD -6      // Memory allocation failed
#define MISSING_DATA -7                 // Missing data for a command argument
#define MOSHION_PLAN_OVERRIGHTING -8    // Attempting to override a motion plan
#define MOSHION_PLAN_NOT_ALLOCATIED -9  // The moshion plan has not bean setup to stor the points
#define ERROR_NUM_C 9                   // Total number of errors

// Structure to hold error information
typedef struct ERROR_COMMAND {
  int index;             // Index of the error
  const char* strError;  // Error message
} ERROR_COMMAND;

// Enumeration for state machine
enum States {
  IFIS,                // ON BOOT UP
  IDLE,                // IDLE STATE
  INIT,                // INITIALIZATION
  RESEVING_COMMAND,    // RECEIVING COMMAND FROM SERIAL
  PROCESSING_COMMAND,  // PROCESSING RECEIVED COMMAND
  DATA_DUMP,           // THE DATA DUMP OF THE MOTION
  EXECUTING_COMMAND,   // EXECUTING THE RECEIVED COMMAND
  LOADING_PART,        // LOADING THE PART FROM THE PART BIN INTO THE BASKET
  WEIGHING_DIPARCHER,  // WEIGHING THE FULL BASKET OF PARTS
  DEPARTING,           // SENDING THE FULL BASKET OUT
  ARRIVING,            // A PART HAS ARRIVED AT ARRIVALS
  DUMPING_ARRIVAL,     // EMPTYING THE PART OUT OF THE CELLED BASKET
  WEIGHING_ARRIVAL,    // WEIGHING THE BASKET WITH THE EXCESS PARTS
  ESTOP,               // EMERGENCY STOP STATE
  ERROR,               // When an error occurs
};

// Array of error messages
const ERROR_COMMAND SYSTEM_ERROR[] = {
  { NO_ERROR, "NO ERROR\n" },
  { COMMAND_INDEX_NOT_FOUND, "Command with index %d was not found in RECEIVABLE_COMMAND list\n" },
  { INPUT_BUFFER_FULL, "The input buffer is full, the command sent may be too big\n" },
  { MISSING_COMMAND, "No command provided\n" },
  { INVALID_COMMAND_ARGUMENTS, "No command arguments provided\n" },
  { EXECUTEFUNCTION_FAILD, "cmd command with index %d was unsuccessful in its execution\n" },
  { MEMORY_ALLOCATION_FAILD, "Memory allocation failed\n" },
  { MISSING_DATA, "The %d command argument was not provided\n" },
  { MOSHION_PLAN_OVERRIGHTING, "A motion has already been allocated and executed\n" }
};

//----------------------------- Function Prototypes -------------------------------------------------------------------
void print_error(int error_index, ...);  // Print error message
int dsprintf(const char* fmt, ...);      // Print formatted string
void readSerialData(char*);  // Read serial data

float mapf(float, float, float, float, float);  // Map a float value from one range to another

#endif  // End of header guard
