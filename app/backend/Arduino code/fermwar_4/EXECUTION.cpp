#include "EXECUTION.h"
#include <TeensyTimerTool.h>

// Declare the global motion plan variable
static MOSHION RobotMoshionPlan = { NULL, 0 };  // Structure to hold motion plan data

int MOSHIOSTATE = SETTINGUP;

// Declare and initialize the current point index
volatile int CurrentPoint = 0;

// Declare the timer objects
IntervalTimer PointTimer;
IntervalTimer pwmTimers[6];  // Array of timers for PWM signals (assuming 6 joints)

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Allocate move data for robot motion
// ARGUMENTS:   strCommandLine - Command line containing parameters
// RETURN VALUE: True if allocation is successful, False otherwise
bool allocateMoveData(char* strCommandLine) {
  char paramaterNnumber = 1;  // Number of parameters expected
  char* token = NULL;         // Current token to analyze
  int NP = 0;                 // Number of points for motion plan

  if (isState(MOSHIOSTATE, SETTINGUP)) {

    // Iterate through and extract all the parameters
    for (char i = 0; i < paramaterNnumber; i++) {
      token = strtok(strCommandLine, seps);  // Get the current token

      if (token == NULL) {
        print_error(MISSING_DATA, "allocateMoveData", i);
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
      print_error(MEMORY_ALLOCATION_FAILD, "allocateMoveData");
      ErrorState = STATE;
      STATE = ERROR;  // Transition to the ERROR state
      return false;
    }
    MOSHIOSTATE = SETUP;                                          // update the Moshion States machine that the moshion is setup
    Serial.printf("\nMoshionState changed to: %d\n", MOSHIOSTATE);  // tell the uP that the MoshionState has changed
    return true;                                                  // Return true to indicate successful allocation
  }
  return false;
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
  if (isState(MOSHIOSTATE, SETUP)) {

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
            print_error(INPUT_BUFFER_FULL, "storeMoshioin");
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

        // Check if any data is missing
        if (token == NULL) {
          print_error(MISSING_DATA, "storeMoshioin", paramater);
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
    // Serial.print("data Reacap\n");
    // int CurrentFrequency = 0;
    // for ( int point = 0; point <RobotMoshionPlan.MOVECNT; point++){
    //   for ( int frequency = 0; frequency < 6; frequency++){
    //     CurrentFrequency= RobotMoshionPlan.Points[point].Frequency[frequency];
    //     Serial.printf("Par%d %d ", frequency, CurrentFrequency);
    //   }
    //   Serial.printf("Par6 %d Point %d\n", RobotMoshionPlan.Points[point].TIME, RobotMoshionPlan.Points[point].INDEX);
    // }
    // update the Moshion States machine that the moshion READY to run
    MOSHIOSTATE = READY;
    Serial.printf("\nMoshionState changed to: %d\n", MOSHIOSTATE);  // tell the uP that the MoshionState has changed
    return true;                                                  // Return true to indicate successful allocation
  }
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Executes a planned motion by setting up an IntervalTimer to call the newPointISR function.
// ARGUMENTS:   strCommandLine - Not used in this function.
// RETURN VALUE: Always returns false.
bool executPlanedMove(char* strCommandLine) {
  if (isState(MOSHIOSTATE, READY)) {
    if (RobotMoshionPlan.Points[0].TIME == 0) {
      MOSHIOSTATE = SETTINGUP;
      Serial.printf("MoshionState changed to: %d\n", MOSHIOSTATE);
      return true;
    }
    CurrentPoint = 0;
    // Begin the IntervalTimer with newPointISR
    if (!PointTimer.begin(newPointISR, 1)) {
      // Handle error condition if timer initialization fails
      print_error(BAD_TIMMER_SETUP, "executPlanedMove");
      return false;  // return faild
    } else {
      MOSHIOSTATE = INMOSHION;
      Serial.printf("MoshionState changed to: %d\n", MOSHIOSTATE);  // tell the uP that the MoshionState has changed
      return true;                                                  // Indicate that the function always returns false
    }
  }
  return false;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:
// ARGUMENTS:
// RETURN VALUE:
bool getWight(char* strCommandLine) {
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
// DESCRIPTION: Check if the current state matches the desired state and print relevant messages if not.
// ARGUMENTS:
//   - CurrentState: The current state of the motion system.
//   - dState: The desired state to check against.
// RETURN VALUE: True if the current state matches the desired state, false otherwise.
bool isState(int CurrentState, int dState) {
  // Check if the current state matches the desired state
  if (CurrentState == dState) {
    return true;  // Return true if the states match
  } else if (CurrentState == SETTINGUP) {
    Serial.print("Motion needs to be prepared\n");  // Print message if motion is not prepared
  } else if (CurrentState == SETUP) {
    Serial.print("Motion needs to be set up\n");  // Print message if motion is not set up
  } else if (CurrentState == READY) {
    Serial.print("Motion needs to be executed\n");  // Print message if motion is not ready
  } else if (CurrentState == INMOSHION) {
    Serial.print("Motion is running\n");  // Print message if motion is running
  }
  return false;  // Return false as states do not match
}


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Interrupt Service Routine for processing motion points.
//              This function updates the output pins and frequencies for stepper motors based on motion points.
// ARGUMENTS:   None
// RETURN VALUE: None
void newPointISR() {
  PointTimer.end();  // Stop the timer
  // Cache frequently accessed values
  volatile int moveCnt = RobotMoshionPlan.MOVECNT;                              // Total number of motion points
  volatile int* frequencies = RobotMoshionPlan.Points[CurrentPoint].Frequency;  // Pointer to frequencies array
  volatile int currentTime = RobotMoshionPlan.Points[CurrentPoint].TIME;        // Current time

  // Check if CurrentPoint exceeds the total number of motion points
  if (CurrentPoint >= moveCnt) {
    PointTimer.end();  // End the timer if all motion points have been executed

    // Unroll the loop manually for better performance
    for (volatile int i = 0; i < 6; i++) {
      // Set duty cycle for both channels
      analogWrite(StepperStepsPins[i], 0);
      digitalWriteFast(SepperDirPins[i], 0);
    }
    // update the Moshion States machine that the moshion has finished and ready for the nest one
    MOSHIOSTATE = SETTINGUP;
    Serial.printf("MoshionState changed to: %d\n", MOSHIOSTATE);  // tell the uP that the MoshionState has changed
    free(RobotMoshionPlan.Points);
    return;
  }

  // Unroll the loop manually for better performance
  for (volatile int i = 0; i < 6; i++) {
    // Process two channels simultaneously
    volatile int frequency = frequencies[i];  // Get the Frequency channel 
    volatile int step = StepperStepsPins[i];  // step Pin for current channel 
    volatile int dir = SepperDirPins[i];      // Direction pin for current channel

    digitalWriteFast(dir, frequency <= 0 ? HIGH : LOW);      // Set direction based on frequency sign
    analogWriteFrequency(step, abs(frequency));              // Set frequency
    analogWrite(step, abs(frequency) <= 0 ? 0 : dutyCycle);  // Set duty cycle (if frequency is greater than 0)
  }


  PointTimer.begin(newPointISR, currentTime);  // Begin the timer for the next motion point
  // Move to the next motion point
  CurrentPoint++;
}
