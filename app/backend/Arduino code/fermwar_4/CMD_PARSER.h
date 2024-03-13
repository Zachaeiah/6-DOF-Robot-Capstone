#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include "Gloabls.h"
#include "EXECUTION.h"

#define MAX_LINE_SIZE 128  // Size of array to store a line from a file.
                            // NOTE: 2 elements must be reserved for trailing '\n' and '\0'

// Function pointer type definition for command execution
typedef bool (*CommandFunction)(char* strCommandLine);

// Structure to map command keyword string to a command index
typedef struct COMMAND {
  int index;            // Index of the command
  const char* strCommand;  // String representation of the command
  CommandFunction executeFunction; // function pointer to execute the command
} COMMAND;

// Enumerated type listing all command indexes
enum commands {
  R_MOVES,       // The number of moves to be in the coming data dump
  R_MOSHION,     // A motion interpolation point
  R_EXECUTE,     // exacute the stored moshion 
  R_GET_WIGHT,    // Starting unloading the part into the basket
  R_HAND,        // uP is ready to transmit
  L_HAND,        // uC is ready to receive
  RECELABLE_NUM_COMMANDS   // The number of commands
};

// Global array of command keyword strings to command index associations
const COMMAND RECIEVABLE_COMMAND[] = {
  { R_MOVES, "R_MOVES", allocateMoveData},
  { R_MOSHION, "R_MOSHION", storeMoshioin},
  { R_EXECUTE, "R_EXECUTE", executPlanedMove},
  { R_GET_WIGHT, "R_GET_WIGHT", getWight},
  { L_HAND, "L_HAND", ReaduC}
};

//----------------------------- Function Prototypes -------------------------------------------------------------------

// Function to convert a string to all uppercase characters
void makeStringUpperCase(char* str);

// Function to process an incoming command
void processCommand(int commandIndex, char* commandString);

// Function to get the command index from a string
int getCommandIndex(const char* strLine) ;

#endif
