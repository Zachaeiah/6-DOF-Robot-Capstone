#ifndef EXECUTION_H
#define EXECUTION_H
#include "Gloabls.h"

// Constants
const int dutyCycle = 127;  // 50% duty cycle

// External variables
extern volatile int CurrentPoint;  // Current motion point index
extern IntervalTimer PointTimer;   // Declare the timer object globally

// Structure to hold a motion point with joint interpolations
typedef struct POINT_INTERP {
  int Frequency[6];  // Array of frequencies 
  int TIME;          // Time at which the frequencies are applied
  int INDEX;         // Index
} POINT_INTERP;

// Structure to hold a motion plan
typedef struct MOTION {
  POINT_INTERP* Points;  // Array of motion points
  int MOVECNT;           // Total number of motion points
} MOTION;

// Enumeration for Moshion States machine
enum MoshionStates {SETTINGUP, SETUP, READY, INMOSHION, MOSHIONERROR};

//----------------------------- Function Prototypes -------------------------------------------------------------------

// Allocate move data for robot motion
bool allocateMoveData(char* strCommandLine);

// Set the delay for a planned move
bool setMoveDelay(char* strCommandLine);

// store the data dump of the moshion
bool storeMoshioin(char* strCommandLine);

// Execute a planned move
bool executPlanedMove(char* strCommandLine);

// Perform output weight operation
bool getWight(char* strCommandLine);

// Read microcontroller data
bool ReaduC(char* strCommandLine);

// Define the ISR function
void newPointISR();

// check with sate is goint to run
bool isState(int CurrentState, int dState);

#endif  // End of header guard
