#include "Pannle.h"

unsigned long lastRetrieveBTNTime = 0;
unsigned long lastReturnBTNTime = 0;
unsigned long lastOPMTTime = 0;
unsigned long debounceDelay = 50;  // Adjust this value as needed

// Define variables
int lastRetrieveBTNState = HIGH;  // Initial state of the button
int lastReturnBTNState = HIGH;    // Initial state of the button
int lastOPMTState = HIGH;         // Initial state of the switch

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Initializes the panel by setting pin modes and attaching interrupt service routines.
// ARGUMENTS: None
// RETURN VALUE: None
void initPannle() {
  // Set pin modes for buttons and switches
  pinMode(RetrieveBTN, INPUT_PULLUP);  // Set the pin as input with pull-up
  pinMode(ReturnBTN, INPUT_PULLUP);    // Set the pin as input with pull-up
  pinMode(OPMT, INPUT_PULLUP);                // Set the pin as input with pull-up

  // Attach interrupt service routines to buttons and switch
  attachInterrupt(digitalPinToInterrupt(RetrieveBTN), RetrieveBTNInterrupt, FALLING);  // Attach interrupt on falling edge
  attachInterrupt(digitalPinToInterrupt(ReturnBTN), ReturnBTNInterrupt, FALLING);      // Attach interrupt on falling edge
  attachInterrupt(digitalPinToInterrupt(OPMT), OPMTInterrupt, CHANGE);                 // Attach interrupt on change
}

void RetrieveBTNInterrupt() {
  EndterBTNTime = millis();
  if (millis() - EndterBTNTime > debounceDelay) {
    if (!digitalRead(RetrieveBTN){
      Serial.println("RetrieveBTN DOWN");
    }
  }
}

void ReturnBTNInterrupt() {
  EndterBTNTime = millis();
  if (millis() - EndterBTNTime > debounceDelay) {
    if (!digitalRead(ReturnBTN){
      Serial.println("ReturnBTN DOWN");
    }
  }
}

void OPMTInterrupt() {
  static int SWITCH_STATE = 0;
  EndterBTNTime = millis();
  if (millis() - EndterBTNTime > debounceDelay) {
    lastOPMTTime = millis();

    if (SWITCH_STATE) {
      Serial.println("OPMT 1");
      SWITCH_STATE = 0;
    } else {
      Serial.println("OPMT 0");
      SWITCH_STATE = 1;
    }
  }
  lastOPMTState = currentState;
}
