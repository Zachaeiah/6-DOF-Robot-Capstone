#include "EXECUTION.h"
#include "Gloabls.h"

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Allocate move data for robot motion
// ARGUMENTS:   strCommandLine - Command line containing parameters
// RETURN VALUE: True if allocation is successful, False otherwise
bool allocateMoveData(char* strCommandLine) {
  char paramaterNnumber = 1;  // Number of parameters expected
  char* token = NULL;         // Current token to analyze
  int NP = 0;                  // Number of points for motion plan
  MOSHION RobotMoshionPlan = {NULL, 0};  // Structure to hold motion plan data

  // Check if data dump is in progress or if a motion plan is already allocated
  if (ReadDataDump == true || RobotMoshionPlan.Points != NULL){
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

  // Allocate memory for the joint angles
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
  return true;  // Return true to indicate successful allocation
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
bool executPlanedMove(char* strCommandLine) {
  dsprintf("executPlanedMove");
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