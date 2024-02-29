
#ifndef LOADCELL_H  // Header guard to prevent multiple inclusion
#define LOADCELL_H

#include "Gloabls.h"
#define dataPin 3
#define clockPin 2
#define LOAD_CELL_BAUDRATE 115200

void initLoadCell();
void calibrateLoadCell();

#endif  // End of header guard