#include "Pannle.h"

unsigned long EndterBTNTime = 0;
unsigned long debounceDelay = 20;

bool Pre_OMPT_STATE = 1;
bool Pre_RetrieveBTN_STATE = 1;
bool Pre_ReturnBTN_STATE = 1;

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Initializes the panel by setting pin modes and attaching interrupt service routines.
// ARGUMENTS: None
// RETURN VALUE: None
void initPannle() {
  // Set pin modes for buttons and switches
  pinMode(RetrieveBTN, INPUT_PULLUP);  // Set the pin as input with pull-up
  pinMode(ReturnBTN, INPUT_PULLUP);    // Set the pin as input with pull-up
  pinMode(OPMT, INPUT_PULLUP);         // Set the pin as input with pull-up

  // Attach interrupt service routines to buttons and switch
  attachInterrupt(digitalPinToInterrupt(RetrieveBTN), RetrieveBTNInterrupt, FALLING);  // Attach interrupt on falling edge
  attachInterrupt(digitalPinToInterrupt(ReturnBTN), ReturnBTNInterrupt, FALLING);      // Attach interrupt on falling edge
  attachInterrupt(digitalPinToInterrupt(OPMT), OPMTInterrupt, CHANGE);                 // Attach interrupt on change
}

void RetrieveBTNInterrupt() {
  for (int i = 0; i< 8;){
    EndterBTNTime = millis();
    while (!(millis() - EndterBTNTime > debounceDelay))
    ;
    if (digitalRead(RetrieveBTN) == LOW){
      i++;
    }
    else{
      i = 0;
    }
  }
  if (digitalRead(RetrieveBTN) == LOW) {
    Serial.println("RetrieveBTN DOWN");
  }
}

void ReturnBTNInterrupt() {
  for (int i = 0; i< 8;){
    EndterBTNTime = millis();
    while (!(millis() - EndterBTNTime > debounceDelay))
    ;
    if (digitalRead(ReturnBTN) == LOW){
      i++;
    }
    else{
      i=0;
    }
  }
  if (digitalRead(ReturnBTN) == LOW) {
    Serial.println("ReturnBTN DOWN");
  }
}

void OPMTInterrupt() {
  EndterBTNTime = millis();
  while (!(millis() - EndterBTNTime > debounceDelay))
    ;
  if ((digitalRead(OPMT) == HIGH) && (Pre_OMPT_STATE == 0)) {
    Serial.println("OPMT 1");
    Pre_OMPT_STATE = 1;

  } else if ((digitalRead(OPMT) == LOW) && (Pre_OMPT_STATE == 1)) {
    Serial.println("OPMT 0");
    Pre_OMPT_STATE = 0;
  }
}
