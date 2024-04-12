#ifndef GLOABKS_H  // Header guard to prevent multiple inclusion
#define GLOABKS_H


#include <math.h>  // math functions
#include <stdlib.h>
#include <string.h>   // string functions
#include <stdarg.h>   // Include for variable argument lists
#include <ctype.h>    // character functions
#include <stdbool.h>  // bool definitions
#include <SD.h>       // SD card library
#include <Arduino.h>  // Arduino core library

extern bool ReadDataDump;       // Declare connected as an external variable
extern int error_index;         // Index of the current error
extern int SYS_registor;        // Current state of the system state machine
extern int MOSHIOSTATE;         // the state of the moishion
extern int ErrorState;          // State at which an error occurred
extern File dataFile;           // File for error logging
extern const char* seps;        // String delimiter for tokenization
const int maxBufferSize = 1024;  // Maximum buffer size for incoming data
const char StepperStepsPins[] = {  0, 1, 2, 4,  5,  6};
const char SepperDirPins[] =    { 26, 7, 8, 9, 24, 25};

// Error codes
#define NO_ERROR 0                    // No error
#define COMMAND_INDEX_NOT_FOUND -1    // Command index does not exist
#define INPUT_BUFFER_FULL -2          // Input buffer is full, command was too big
#define MISSING_COMMAND -3            // No command provided
#define INVALID_COMMAND_ARGUMENTS -4  // No command arguments provided
#define EXECUTEFUNCTION_FAILD -5      // Command execution failed
#define MEMORY_ALLOCATION_FAILD -6    // Memory allocation failed
#define MISSING_DATA -7               // Missing data for a command argument
#define BAD_TIMMER_SETUP -8           // the timmer for trasishiong moshion did not get setup right
#define TIMER_INIT_FAILURE -9        // when the moshion timer fasils to intit


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
  { NO_ERROR, "Error in %s, NO ERROR\n" },
  { COMMAND_INDEX_NOT_FOUND, "Error in %s, Command is not found in RECEIVABLE_COMMAND list\n" },
  { INPUT_BUFFER_FULL, "Error in %s, The input buffer is full, the command sent may be too big\n" },
  { MISSING_COMMAND, "Error in %s, No command provided\n" },
  { INVALID_COMMAND_ARGUMENTS, "Error in %s, No command arguments provided\n" },
  { EXECUTEFUNCTION_FAILD, "Error in %s, cmd command with index %d was unsuccessful in its execution\n" },
  { MEMORY_ALLOCATION_FAILD, "Error in %s, Memory allocation failed\n" },
  { MISSING_DATA, "Error in %s, The %d command argument was not provided\n" },
  { BAD_TIMMER_SETUP,  "Error in %s, Motion Timme did not setup all hardware timers are being used"},
};

//----------------------------- Function Prototypes -------------------------------------------------------------------
void print_error(int error_index, ...);  // Print error message
int dsprintf(const char* fmt, ...);      // Print formatted string
void readSerialData(char*);              // Read serial data

float mapf(float, float, float, float, float);  // Map a float value from one range to another

#endif  // End of header guard
