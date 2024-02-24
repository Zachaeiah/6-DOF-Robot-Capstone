#include "EXECUTION.h"
#include "Gloabls.h"

static MOSHION RobotMoshionPlan = { NULL, 0 };  // Structure to hold motion plan data

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Allocate move data for robot motion
// ARGUMENTS:   strCommandLine - Command line containing parameters
// RETURN VALUE: True if allocation is successful, False otherwise
bool allocateMoveData(char* strCommandLine) {
  char paramaterNnumber = 1;  // Number of parameters expected
  char* token = NULL;         // Current token to analyze
  int NP = 0;                 // Number of points for motion plan


  // Check if data dump is in progress or if a motion plan is already allocated
  if (ReadDataDump == true || RobotMoshionPlan.Points != NULL) {
    print_error(MOSHION_PLAN_OVERRIGHTING);
    ErrorState = STATE;
    STATE = ERROR;  // Transition to the ERROR state
    return false;
  }

  // Iterate through and extract all the parameters
  for (char i = 0; i < paramaterNnumber; i++) {
    token = strtok(strCommandLine, seps);  // Get the current token

    if (token == NULL) {
      print_error(MISSING_DATA);
      ErrorState = STATE;
      STATE = ERROR;  // Transition to the ERROR state
      return false;
    }

    // Set the number of points in the motion plan
    if (i == 0) {
      NP = (int)atoi(token);
    }
  }

  // Allocate memory for the moshion
  RobotMoshionPlan.Points = (POINT_INTERP*)calloc(NP, sizeof(POINT_INTERP));
  RobotMoshionPlan.MOVECNT = NP;

  // If memory allocation fails, print an error message and transition to the ERROR state
  if (RobotMoshionPlan.Points == NULL) {
    print_error(MEMORY_ALLOCATION_FAILD);
    ErrorState = STATE;
    STATE = ERROR;  // Transition to the ERROR state
    return false;
  }
  ReadDataDump = true;  // Set data dump flag to true

  dsprintf("allocateMoveData");  // Debug message
  return true;                   // Return true to indicate successful allocation
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Stores motion data received over serial communication.
//              Expects a series of newline-terminated strings containing STEP_CNT and STEP_FR values.
// ARGUMENTS:   strCommandLine - Not used in this function.
// RETURN VALUE: True if the motion data is successfully stored, False otherwise.
bool storeMoshioin(char* strCommandLine) {
  POINT_INTERP pointMoshion;      // Structure to hold motion point data
  JOINT_INTERP motorMoshion;      // Structure to hold motor motion data
  char paramaterNnumber = 6;      // Number of parameters expected
  char* STEP_CNT_token = NULL;    // Current token to analyze for STEP_CNT
  char* STEP_FR_token = NULL;     // Current token to analyze for STEP_FR
  char* MotorEnable_token = NULL; // Token to analyze for motor enable hex value
  char inputBuffer[maxBufferSize];// Array to hold input received over serial
  char motorACTS = 0;             // Motor activation hex value
  int bufferIndex = 0;            // Initialize buffer index
  volatile int pointCNT = 0;      // Counter for motion points

  // Check if data dump is in progress or if a motion plan is already allocated
  if (ReadDataDump == false || RobotMoshionPlan.Points == NULL || RobotMoshionPlan.MOVECNT <= 0) {
    print_error(MOSHION_PLAN_NOT_ALLOCATIED);
    ErrorState = STATE;
    STATE = ERROR;  // Transition to the ERROR state
    return false;
  }

  // Get the hex value to set the motor activation pins
  MotorEnable_token = strtok(strCommandLine, seps);

  if (MotorEnable_token == NULL) {
    print_error(MISSING_DATA, 1);
    ErrorState = STATE;
    STATE = ERROR;  // Transition to the ERROR state
    return false;
  }

  // Convert the hexadecimal string to a char
  motorACTS = (char)strtol(MotorEnable_token, NULL, 16);

  // Set the motor activation pins according to the motor activation hex value
  for (int i = 0; i < 6; ++i) {
    // Check if the corresponding bit in the binary representation is set
    if (motorACTS & (1 << i)) {
      digitalWrite(MotorEnablePins[i], HIGH);  // Set the pin HIGH
    } else {
      digitalWrite(MotorEnablePins[i], LOW);   // Set the pin LOW
    }
  }

  Serial.printf("storing %d\n", RobotMoshionPlan.MOVECNT);

  // Loop through each motion point
  for (; pointCNT < RobotMoshionPlan.MOVECNT; pointCNT++) {
    memset(inputBuffer, '\0', sizeof(inputBuffer));  // Clear the input buffer

    // Wait until data is available on the serial line
    while (!Serial.available()) {
      // Do nothing
    }

    memset(inputBuffer, '\0', sizeof(inputBuffer));  // Clear the input buffer
    bufferIndex = 0;

    // Loop to read data until a newline character is encountered
    while (true) {
      // Wait until data is available on the serial line
      while (!Serial.available()) {
        // Do nothing
      }

      // Read the next character
      char inChar = Serial.read();

      // Check for newline character
      if (inChar == '\n') {
        inputBuffer[bufferIndex] = '\0';  // Terminate the input string
        break;                            // Exit the loop if newline character is encountered
      } else {
        // Add the character to the input buffer if it's not a newline
        if (bufferIndex < maxBufferSize - 2) {
          inputBuffer[bufferIndex++] = inChar;
        } else {
          // Print an error if the buffer is full
          print_error(INPUT_BUFFER_FULL);
          ErrorState = STATE;
          STATE = ERROR;  // Transition to the ERROR state
          return false;
        }
      }
    }

    // Iterate through and extract all the parameters
    for (int i = 0; i < paramaterNnumber; i++) {
      // Get the current token or continue from the last position
      STEP_CNT_token = strtok((i == 0) ? inputBuffer : NULL, seps);
      // Get the current token
      STEP_FR_token = strtok(NULL, seps);

      // Check if any data is missing
      if (STEP_CNT_token == NULL || STEP_FR_token == NULL) {
        print_error(MISSING_DATA, i);
        ErrorState = STATE;
        STATE = ERROR;  // Transition to the ERROR state
        return false;
      }

      // Convert strings to integers and store in motorMoshion struct
      motorMoshion.STEP_CNT = (int)atoi(STEP_CNT_token);
      motorMoshion.STEP_FR = (int)atoi(STEP_FR_token);
      // Store motorMoshion struct in pointMoshion struct
      pointMoshion.INTEPR[i] = motorMoshion;
    }
    // Store pointMoshion struct in RobotMoshionPlan.Points array
    pointMoshion.INDEX = pointCNT;
    RobotMoshionPlan.Points[pointCNT] = pointMoshion;
  }

  // Print stored motion data for debugging purposes
  for (int j = 0; j < RobotMoshionPlan.MOVECNT; j++) {
    Serial.printf("\nMoshion %d\n", j);
    for (int i = 0; i < 6; i++) {
      Serial.printf("index %d: STEP_CNT %d\tSTEP_FR %d\n", i, RobotMoshionPlan.Points[j].INTEPR[i].STEP_CNT, RobotMoshionPlan.Points[j].INTEPR[i].STEP_FR);
    }
  }

  // Print debug message
  dsprintf("executPlanedMove\n");
  return true;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:
// ARGUMENTS:
// RETURN VALUE:
bool executPlanedMove(char* strCommandLine) {
  dsprintf("executPlanedMove");
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:
// ARGUMENTS:
// RETURN VALUE:
bool setMoveDelay(char* strCommandLine) {
  dsprintf("setMoveDelay");
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:
// ARGUMENTS:
// RETURN VALUE:
bool PorOutWight(char* strCommandLine) {
  dsprintf("PorOutWight");
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:
// ARGUMENTS:
// RETURN VALUE:
bool ReaduC(char* strCommandLine) {
  dsprintf("ReaduC");
  return false;
}