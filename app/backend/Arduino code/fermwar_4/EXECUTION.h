#ifndef EXECUTION_H
#define EXECUTION_H

#include "Gloabls.h"

// Structure to hold a motion point
typedef struct JOINT_INTERP {
  int STEP_CNT;   // Step count
  int STEP_FR;    // Step frequency
} JOINT_INTERP;

// Structure to hold a motion point
typedef struct POINT_INTERP {
  JOINT_INTERP INTEPR[6] = { 0 };  // Array of joint interpolations
  int INDEX;                        // Index
} POINT_INTERP;

// Structure to hold the motion 
typedef struct MOSHION {
  POINT_INTERP* Point; 
  int MOVECNT;    
} MOSHION;

extern MOSHION RobotMoshionPlan;

bool allocateMoveData(char* strCommandLine);

bool setMoveDelay(char* strCommandLine);

bool executPlanedMove(char* strCommandLine);

bool PorOutWight(char* strCommandLine);

bool ReaduC(char* strCommandLine);

#endif