#include "EXECUTION.h"
#include <TeensyTimerTool.h>


static MOSHION RobotMoshionPlan = { NULL, 0 };  // Structure to hold motion plan data
volatile int CurrentPoint = 0;

//
IntervalTimer PointTimer;
// Create an array of timers
IntervalTimer pwmTimers[6];

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
  int paramaterNnumber = 7;         // Number of parameters expected
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

      Serial.printf("Par%d: %s ", paramater, token);

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
    Serial.println();
    // Store pointMoshion struct in RobotMoshionPlan.Points array
    pointMoshion.INDEX = pointCNT;
    RobotMoshionPlan.Points[pointCNT] = pointMoshion;
  }

  // Print stored motion data for debugging purposes
  for (int j = 0; j < RobotMoshionPlan.MOVECNT; j++) {
    Serial.printf("\nMoshion: %d,\tTIME: %d uS\n", j, RobotMoshionPlan.Points[j].TIME);
    for (int i = 0; i < 6; i++) {
      Serial.printf("index %d: STEP_FR %d\n", i, RobotMoshionPlan.Points[j].Frequency[i]);
    }
  }

  // Print debug message
  dsprintf("storeMoshioin\n");
  return true;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Executes a planned motion by setting up an IntervalTimer to call the newPointISR function.
// ARGUMENTS:   strCommandLine - Not used in this function.
// RETURN VALUE: Always returns false.
bool executPlanedMove(char* strCommandLine) {
  // Remove the local declaration of CurrentPoint
  // volatile int CurrentPoint = 0;
  CurrentPoint = 0;

  // Initialize an IntervalTimer to call the newPointISR function
  // IntervalTimer PointTimer; // No need to declare it here since it's already declared globally

  // Begin the IntervalTimer with newPointISR as the callback function and the specified interval
  if (!PointTimer.begin(newPointISR, 1)) {
    Serial.println("Timer did not work");
    // Handle error condition if timer initialization fails
    return false;
  } else {
    Serial.println("Timer started");
  }

  // Debug message indicating the start of executing a planned move
  dsprintf("executPlanedMove");

  return true;  // Indicate that the function always returns false
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



//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Interrupt Service Routine for processing motion points.
//              This function updates the output pins and frequencies for stepper motors based on motion points.
// ARGUMENTS:   None
// RETURN VALUE: None
void newPointISR() {
  // Cache frequently accessed values
  volatile int moveCnt = RobotMoshionPlan.MOVECNT;
  volatile int* frequencies = RobotMoshionPlan.Points[CurrentPoint].Frequency;
  volatile int currentTime = RobotMoshionPlan.Points[CurrentPoint].TIME;

  // Check if CurrentPoint exceeds the total number of motion points
  if (CurrentPoint >= moveCnt) {
    PointTimer.end();  // End the timer if all motion points have been executed

    // Unroll the loop manually for better performance
    for (volatile int i = 0; i < 6; i += 2) {
      // Set duty cycle for both channels
      analogWrite(StepperStepsPins[i], 0);
      analogWrite(StepperStepsPins[i + 1], 0);
    }
    return;
  }

  // Unroll the loop manually for better performance
  for (volatile int i = 0; i < 6; i += 2) {
    // Process two channels simultaneously
    volatile int frequency0 = frequencies[i];
    volatile int frequency1 = frequencies[i + 1];
    volatile int pin0 = StepperStepsPins[i];
    volatile int pin1 = StepperStepsPins[i+1];

    digitalWriteFast(SepperDirPins[i], frequency0 < 0 ? HIGH : LOW);
    analogWriteFrequency(pin0, frequency0);
    analogWriteFrequency(pin1, frequency1);

    analogWrite(pin0, dutyCycle);
    analogWrite(pin1, dutyCycle);
  }

  PointTimer.end();  // Stop the timer
  PointTimer.begin(newPointISR, currentTime);
  // Move to the next motion point
  CurrentPoint++;
}
