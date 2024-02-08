#ifndef GLOABKS_H
#define GLOABKS_H

#include <math.h>     // math functions
#include <string.h>   // string functions
#include <stdarg.h>  // Include for variable argument lists
#include <ctype.h>    // character functions
#include <stdbool.h>  // bool definitions
#include <SD.h>
#include <Arduino.h>

extern bool connected; // Declare connected as an external variable
extern int STATE;      // Declare STATE as an external variable
extern File dataFile;


// Enumeration for state machine
enum States {
  IFIS,                // ON BOOT UP
  IDLE,                // IDLE STATE
  INIT,                // INITIALIZATION
  RESEVING_COMMAND,    // RECEIVING COMMAND FROM SERIAL
  PROCESSING_COMMAND,  // PROCESSING RECEIVED COMMAND
  DATA_DUMP,           // THE DATA DUMP OF THE MOTION
  EXECUTING_COMMAND,   // EXECUTING THE RECEIVED COMMAND
  LOADING_PART,        // LOADING THE PART FROM THE PART BIN INTO THE BASKET
  WEIGHING_DIPARCHER,  // WEIGHING THE FULL BASKET OF PARTS
  DEPARTING,           // SENDING THE FULL BASKET OUT
  ARRIVING,            // A PART HAS ARRIVED AT ARRIVALS
  DUMPING_ARRIVAL,     // EMPTYING THE PART OUT OF THE CELLED BASKET
  WEIGHING_ARRIVAL,    // WEIGHING THE BASKET WITH THE EXCESS PARTS
  ESTOP,               // EMERGENCY STOP STATE
  ERROR,               // When an error ocors
};

void dsprintString(const char *message); // print string to Serial line and sd card for logging
void dsprintInt(int value);
void dsprintLong(long value);
void dsprintDouble(double value);
float mapf(float, float, float, float, float);  // Map a float value from one range to another

#endif