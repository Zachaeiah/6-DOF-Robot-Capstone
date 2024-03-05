#ifndef EXECUTION_H
#define EXECUTION_H
#include "Gloabls.h"

// Structure to hold a motion point with joint interpolations
typedef struct POINT_INTERP {
  int Frequency[6];  // Array of frequensy (assuming 6 joints)
  int TIME; // the time at witch the frequensy are
  int INDEX;                       // Index
} POINT_INTERP;

// Structure to hold a motion plan
typedef struct MOSHION {
  POINT_INTERP* Points;  // Array of motion points
  int MOVECNT;           // Total number of motion points
} MOSHION;

extern volatile int CurrentPoint; // Current motion point index
const int dutyCycle = 127; // 24% duty cycle

// Declare the timer object globally
extern IntervalTimer PointTimer;

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
bool PorOutWight(char* strCommandLine);

// Read microcontroller data
bool ReaduC(char* strCommandLine);

// Define the ISR function
void newPointISR();

#endif  // End of header guard
