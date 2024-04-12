#ifndef PANNLE_H  // Header guard to prevent multiple inclusion
#define PANNLE_H

#include "Gloabls.h"

#define RetrieveBTN 23
#define ReturnBTN 22
#define OPMT 21

extern unsigned long lastRetrieveBTNTime;
extern unsigned long lastReturnBTNTime;
extern unsigned long lastOPMTTime;
extern unsigned long debounceDelay;

extern int lastOPMTState;

void initPannle();
void RetrieveBTNInterrupt();
void ReturnBTNInterrupt();
void OPMTInterrupt();

#endif  // End of header guard
