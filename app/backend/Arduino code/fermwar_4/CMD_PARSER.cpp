
#include "Gloabls.h"
#include "CMD_PARSER.h"

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Get the index of a command from a string
// ARGUMENTS:   strLine - The input string containing the command
// RETURN VALUE: The index of the command if found, -1 otherwise
int getCommandIndex(const char* strLine) {
  // Find the number of commands
  const int numCommands = sizeof(RECIEVABLE_COMMAND) / sizeof(RECIEVABLE_COMMAND[0]);

  // Ensure strLine is not NULL
  if (strLine == nullptr)
    return COMMAND_INDEX_NOT_FOUND;

  // Convert strLine to uppercase
  char strLine2[MAX_LINE_SIZE];
  strcpy(strLine2, strLine);

  // Find which command matches the known commands
  for (int n = 0; n < numCommands; n++) {
    if (strcmp(RECIEVABLE_COMMAND[n].strCommand, strLine2) == 0)
      return RECIEVABLE_COMMAND[n].index;
  }

  // If the command was not found
  return COMMAND_INDEX_NOT_FOUND;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Process an incoming command and perform the corresponding action
// ARGUMENTS:   commandIndex - Index of the command to be processed
//              commandString - String representation of the command
// RETURN VALUE: None
void processCommand(int commandIndex, char* commandString) {
  // Check if the command index is valid
  if (commandIndex < 0 || commandIndex >= RECELABLE_NUM_COMMANDS) {
    dsprintf("Invalid command index\n");
    return;
  }

  // Execute the command function associated with the given index
  bool success = RECIEVABLE_COMMAND[commandIndex].executeFunction(commandString);

  // Output the result
  if (success) {
    dsprintf("Command successful\n\n");
  } else {
    dsprintf("Command execution failed\n\n");
    print_error(EXECUTEFUNCTION_FAILD, commandIndex);  // Set the error index
    ErrorState = STATE;
    STATE = ERROR;
  }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Convert a string to all uppercase characters
// ARGUMENTS:   str - The input string to be converted to uppercase
// RETURN VALUE: None
void makeStringUpperCase(char* str) {
  if (str == NULL) return;
  for (size_t i = 0; i < strlen(str); i++) {
    str[i] = (char)toupper(str[i]);
  }
}

