#ifndef PANNLE_H  // Header guard to prevent multiple inclusion
#define PANNLE_H

#include "Gloabls.h"

#define RetrieveBTN 23
#define ReturnBTN 22
#define OPMT 21

extern unsigned long EndterBTNTime;
extern unsigned long debounceDelay;

extern bool Pre_OMPT_STATE;
extern bool Pre_RetrieveBTN_STATE;
extern bool Pre_ReturnBTN_STATE;

void initPannle();
void RetrieveBTNInterrupt();
void ReturnBTNInterrupt();
void OPMTInterrupt();

#endif  // End of header guard
