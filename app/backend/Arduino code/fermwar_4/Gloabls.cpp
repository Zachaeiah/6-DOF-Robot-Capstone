#include "Gloabls.h"  // Include header file containing global declarations

File dataFile;  // Define a global File object named dataFile

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Map a float value from one range to another
// ARGUMENTS:   x - Value to map
//              in_min - Lower bound of the input range
//              in_max - Upper bound of the input range
//              out_min - Lower bound of the output range
//              out_max - Upper bound of the output range
// RETURN VALUE: Mapped value
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  float result = 0;  // Variable to hold the mapped value
  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;  // Map the value
  return result;  // Return the mapped value
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Print an error message
// ARGUMENTS:   error_index - Index of the error message in SYSTEM_ERROR array
//              ... - Additional arguments if error message format string requires
// RETURN VALUE: None
void print_error(int error_index, ...) {
  va_list args;  // Variable argument list
  error_index = abs(error_index);  // Ensure a positive error index
  va_start(args, error_index);  // Initialize variable argument list

  const ERROR_COMMAND* error = &SYSTEM_ERROR[error_index];  // Get the error command based on index

  char buffer[256];  // Buffer to hold the formatted error message
  vsnprintf(buffer, sizeof(buffer), error->strError, args);  // Format the error message
  dsprintf("Error `%d`: `%s` `Putting systems back into RESEVING_COMMAND state`\n", error_index, buffer);  // Print the error message

  va_end(args);  // End variable argument list
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Print a formatted string to Serial console and, if dataFile is valid, to the file
// ARGUMENTS:   fmt - Format string
// RETURN VALUE: Length of the formatted string
int dsprintf(const char* fmt, ...) {
  char buf[256];  // Buffer to hold formatted string, adjust size as needed
  va_list args;  // Variable argument list
  va_start(args, fmt);  // Initialize variable argument list
  int len = vsnprintf(buf, sizeof(buf), fmt, args);  // Format the string
  va_end(args);  // End variable argument list

  if (dataFile) {
    dataFile.print(buf);  // Print to file if file handle is valid
  }

  Serial.print(buf);  // Print to Serial console

  return len;  // Return the length of the formatted string
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
        print_error(INPUT_BUFFER_FULL);
        ErrorState = STATE;
        STATE = ERROR;  // Transition to the ERROR state
      }
    }
  }
}