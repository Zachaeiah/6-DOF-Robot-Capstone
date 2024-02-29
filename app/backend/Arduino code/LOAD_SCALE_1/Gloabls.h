#ifndef GLOABKS_H  // Header guard to prevent multiple inclusion
#define GLOABKS_H

#include <math.h>    
#include "HX711.h"
#include <LiquidCrystal_I2C.h>

extern int error_index;    // Index of the current error
extern int STATE;          // Current state of the state machine
extern int ErrorState;     // State at which an error occurred

extern const int CANBUS_BAUDRATE;
extern const int SERIAL_BAUDRATE;
const int maxBufferSize = 1024;          // Maximum buffer size for incoming data

const char LCD_ADDRESS = 0x27;
const char LCD_DIM[2] = {0x14, 0x2};

extern LiquidCrystal_I2C lcd;
extern HX711 LoadCell;

// Error codes
#define NO_ERROR 0                      // No error
#define COMMAND_INDEX_NOT_FOUND -1      // Command index does not exist

// Enumeration for state machine
enum States {
  IFIS,                // ON BOOT UP
  IDLE,                // IDLE STATE
  INIT,                // INITIALIZATION
  RESEVING_COMMAND_S,    // RECEIVING COMMAND FROM SERIAL
  RESEVING_COMMAND_C// RECEIVING COMMAND FROM CAN BUSS
}; // Add a semicolon here

#endif  // End of header guard