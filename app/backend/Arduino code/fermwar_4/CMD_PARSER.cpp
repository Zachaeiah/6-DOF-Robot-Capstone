
#include "Gloabls.h"
#include "CMD_PARSER.h"

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Get the index of a command from a string
// ARGUMENTS:   strLine - The input string containing the command
// RETURN VALUE: The index of the command if found, -1 otherwise
int getCommandIndex(char* strLine) {
  char strLine2[MAX_LINE_SIZE];
  strcpy(strLine2, strLine);

  // Find which command matches the known commands
  for (int n = 0; n < RECELABLE_NUM_COMMANDS; n++) {
    if (strstr(strLine2, RECIEVABLE_COMMAND[n].strCommand) != NULL) {
      return RECIEVABLE_COMMAND[n].index;
    }
  }

  // If the command was not found
  return -1;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Process an incoming command and perform the corresponding action
// ARGUMENTS:   commandIndex - Index of the command to be processed
//              commandString - String representation of the command
// RETURN VALUE: None
void processCommand(int commandIndex, char* commandString) {
  bool bSuccess = true;

  dsprintString(RECIEVABLE_COMMAND[commandIndex].strCommand);
  dsprintString("Parsing data: ");
  dsprintString(commandString);

  bSuccess = RECIEVABLE_COMMAND[commandIndex].executeFunction(commandString);
  
  if (bSuccess) {
    dsprintString("Command siccessfull\n\n");
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

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Calculate the length of a string up to a maximum length
// ARGUMENTS:   s - The input string
//              len - The maximum length to consider
// RETURN VALUE: The length of the string up to a maximum length
int strnlen(const char* s, int len) {
  int i = 0;
  for (; i < len && s[i] != '\0'; ++i)
    ;
  return i;
}
