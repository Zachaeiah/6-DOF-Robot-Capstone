//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include "Gloabls.h"
#include "LOADCELL.h"

const int SERIAL_BAUDRATE = 115200;
int STATE = 0;                          // Current state of the system
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_DIM[0], LCD_DIM[1]);


void setup() {
  STATE = IFIS;  // Set initial state to IFIS (ON BOOT UP)
}


void loop() {
  char inputBuffer[maxBufferSize];  // Static array to hold input
  char* token;
  int commandIndex = COMMAND_INDEX_NOT_FOUND;

  switch (STATE) {
    case IFIS:
      STATE = INIT;  // Transition to INIT state
      break;

    case INIT:
      INITuC();  // Initialize microcontroller
      break;

    case RESEVING_COMMAND_C:
      break;
  }

}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Initialize microcontroller
// ARGUMENTS:   None
// RETURN VALUE: None
void INITuC() {

  Serial.begin(SERIAL_BAUDRATE);        // Initialize serial communication
  while (Serial.available()) Serial.read(); //  flush Serial input
  lcd.init();  // initialize the lcd
  initLoadCell();

  // setup Canbuss with teensy 4.1 uC Here

  STATE = RESEVING_COMMAND_C;

}