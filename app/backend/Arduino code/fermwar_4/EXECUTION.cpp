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
// DESCRIPTION:
// ARGUMENTS:
// RETURN VALUE:
bool storeMoshioin(char* strCommandLine) {
  char paramaterNnumber = 6;    // Number of parameters expected
  char* STEP_CNT_token = NULL;  // Current token to analyze
  char* STEP_FR_token = NULL;
  volatile int pointCNT = 0;
  POINT_INTERP pointMoshion;
  JOINT_INTERP motorMoshion;
  char inputBuffer[maxBufferSize];  // array to hold input
  int bufferIndex = 0;              // Initialize buffer index

  // Check if data dump is in progress or if a motion plan is already allocated
  if (ReadDataDump == false || RobotMoshionPlan.Points == NULL || RobotMoshionPlan.MOVECNT <= 0) {
    print_error(MOSHION_PLAN_NOT_ALLOCATIED);
    ErrorState = STATE;
    STATE = ERROR;  // Transition to the ERROR state
    return false;
  }

  Serial.printf("storing %d\n", RobotMoshionPlan.MOVECNT);

  for (; pointCNT < RobotMoshionPlan.MOVECNT; pointCNT++) {
    memset(inputBuffer, '\0', sizeof(inputBuffer));  // clear the input buffer

    while (!Serial.available()) {
      // Wait until data is available on the serial line
    }

    memset(inputBuffer, '\0', sizeof(inputBuffer));  // Clear the input buffer
    bufferIndex = 0;

    // Loop to read data until a newline character is encountered
    while (true) {
      // Check if data is available
      while (!Serial.available()) {
        // Wait until data is available on the serial line
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


    Serial.printf("moahion sent %s\n", inputBuffer);

    // Iterate through and extract all the parameters
    for (int i = 0; i < paramaterNnumber; i++) {

      STEP_CNT_token = strtok((i == 0) ? inputBuffer : NULL, seps);  // Get the current token or continue from the last position
      STEP_FR_token = strtok(NULL, seps);                            // Get the current token

      if (STEP_CNT_token == NULL || STEP_FR_token == NULL) {
        print_error(MISSING_DATA, i);
        ErrorState = STATE;
        STATE = ERROR;  // Transition to the ERROR state
        return false;
      }
      motorMoshion.STEP_CNT = (int)atoi(STEP_CNT_token);
      pointMoshion.INTEPR[i] = motorMoshion;
    }

    pointMoshion.INDEX = pointCNT;
    Serial.printf("pointMoshion.INDEX: %d\n",pointMoshion.INDEX);

    RobotMoshionPlan.Points[pointCNT] = pointMoshion;
  }
  for (int j = 0; j < RobotMoshionPlan.MOVECNT; j++) {
    Serial.printf("\nMoshion %d\n", j);
    for (int i = 0; i < 6; i++) {
      Serial.printf("index %d: STEP_CNT %d\tSTEP_FR %d\n", i, RobotMoshionPlan.Points[j].INTEPR[i].STEP_CNT, RobotMoshionPlan.Points[j].INTEPR[i].STEP_FR);
    }
  }

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