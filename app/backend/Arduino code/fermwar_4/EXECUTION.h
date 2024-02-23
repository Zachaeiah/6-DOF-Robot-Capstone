#ifndef EXECUTION_H
#define EXECUTION_H

// Structure to hold interpolation data for a joint
typedef struct JOINT_INTERP {
  int STEP_CNT;  // Step count
  int STEP_FR;   // Step frequency
} JOINT_INTERP;

// Structure to hold a motion point with joint interpolations
typedef struct POINT_INTERP {
  JOINT_INTERP INTEPR[6] = { 0 };  // Array of joint interpolations (assuming 6 joints)
  int INDEX;                       // Index
} POINT_INTERP;

// Structure to hold a motion plan
typedef struct MOSHION {
  POINT_INTERP* Points;  // Array of motion points
  int MOVECNT;           // Total number of motion points
} MOSHION;

extern MOSHION RobotMoshionPlan;  // Global variable to store motion plan

// Function prototypes

// Allocate move data for robot motion
bool allocateMoveData(char* strCommandLine);

// Set the delay for a planned move
bool setMoveDelay(char* strCommandLine);

// Execute a planned move
bool executPlanedMove(char* strCommandLine);

// Perform output weight operation
bool PorOutWight(char* strCommandLine);

// Read microcontroller data
bool ReaduC(char* strCommandLine);

#endif  // End of header guard
