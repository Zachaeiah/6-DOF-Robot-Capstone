#ifndef ENABLE_SHIFT_REG
#define ENABLE_SHIFT_REG

#include "Gloabls.h"

// Pin definitions for shift register control
#define MASTER_RESET 33    // Pin for master reset
#define SHIFT_CLK 34       // Pin for shift clock
#define SHIFT_LATCH 35     // Pin for shift register latch
#define OUTPUT_ENABLE 36   // Pin for output enable
#define SHIFT_DATA 37      // Pin for shift data

// Function prototypes
// DESCRIPTION: Initializes the shift register pins and sets initial states.
// ARGUMENTS: None
// RETURN VALUE: None
void MotorEnableShiftRegEnable();

// DESCRIPTION: Sets the motor enable pins using a shift register.
// ARGUMENTS: enablePins - a 16-bit number where each bit corresponds to a motor enable pin
// RETURN VALUE: None
void SetMotorEnable(int enablePins);

#endif
