#include "EXECUTION.h"


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
  POINT_INTERP pointMoshion;        // Structure to hold motion point data
  int paramaterNnumber = 7;        // Number of parameters expected
  char* token = NULL;               // Current token to analyze for STEP_CNT
  char inputBuffer[maxBufferSize];  // Array to hold input received over serial
  int bufferIndex = 0;              // Initialize buffer index
  volatile int pointCNT = 0;        // Counter for motion points

  // Check if data dump is in progress or if a motion plan is already allocated
  if (ReadDataDump == false || RobotMoshionPlan.Points == NULL || RobotMoshionPlan.MOVECNT <= 0) {
    print_error(MOSHION_PLAN_NOT_ALLOCATIED);
    ErrorState = STATE;
    STATE = ERROR;  // Transition to the ERROR state
    return false;
  }

  Serial.printf("storing %d\n", RobotMoshionPlan.MOVECNT);

  // Loop through each motion point
  for (; pointCNT < RobotMoshionPlan.MOVECNT; pointCNT++) {
    //  flush Serial input
    while (Serial.available()) Serial.read();
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
    for (int paramater = 0; paramater < paramaterNnumber; paramater++) {
      // Get the current token or continue from the last position
      token = strtok((paramater == 0) ? inputBuffer : NULL, seps);

      Serial.printf("paramater %d: %s\n", paramater, token);

      // Check if any data is missing
      if (token == NULL) {
        print_error(MISSING_DATA, paramater);
        ErrorState = STATE;
        STATE = ERROR;  // Transition to the ERROR state
        return false;
      }
      if (paramater == paramaterNnumber - 1) pointMoshion.TIME = (int)atoi(token);
      else pointMoshion.Frequency[paramater] = (int)atoi(token);
    }
    // Store pointMoshion struct in RobotMoshionPlan.Points array
    pointMoshion.INDEX = pointCNT;
    RobotMoshionPlan.Points[pointCNT] = pointMoshion;
  }

  // Print stored motion data for debugging purposes
  for (int j = 0; j < RobotMoshionPlan.MOVECNT; j++) {
    Serial.printf("\nMoshion: %d,\tTIME: %d\n", j, RobotMoshionPlan.Points[j].TIME);
    for (int i = 0; i < 6; i++) {
      Serial.printf("index %d: STEP_FR %d\n", i, RobotMoshionPlan.Points[j].Frequency[i]);
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
  volatile int CurrentPoint = 0;
  volatile int CurrentFrequency = 0;

  IntervalTimer PointTimer;

  PointTimer.begin(newPointISR, 0);

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



void newPointISR() {
  // Cache frequently accessed values
  const int moveCnt = RobotMoshionPlan.MOVECNT;
  const int* frequencies = RobotMoshionPlan.Points[CurrentPoint].Frequency;
  const int currentTime = RobotMoshionPlan.Points[CurrentPoint].TIME;
  int i = 0;

  // Check if CurrentPoint exceeds the total number of motion points
  if (CurrentPoint >= moveCnt) {
    PointTimer.end(); // End the timer if all motion points have been executed
    return;
  }

  // Unroll the loop manually for better performance
  while (i < 6) {
    // Process two channels simultaneously
    int frequency0 = frequencies[i];
    int frequency1 = frequencies[i + 1];

    // Update output pins and frequencies
    digitalWrite(SepperDirPins[i], frequency0 < 0 ? HIGH : LOW);
    analogWriteFrequency(StepperStepsPins[i], frequency0);
    digitalWrite(SepperDirPins[i + 1], frequency1 < 0 ? HIGH : LOW);
    analogWriteFrequency(StepperStepsPins[i + 1], frequency1);

    // Move to the next pair of channels
    i += 2;
  }

  // Update the time for the joint interpolated move
  PointTimer.update(currentTime);

  // Move to the next motion point
  CurrentPoint++;
}



