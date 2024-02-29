#include "ENABLE_SHIFT_REG.h"

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Initializes the shift register pins and sets initial states.
// ARGUMENTS: None
// RETURN VALUE: None
void MotorEnableShiftRegEnable(){
  // Setup pin modes for shift register control pins
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  pinMode(MASTER_RESET, OUTPUT);
  pinMode(OUTPUT_ENABLE, OUTPUT);
  
  // Set initial states: master reset off (active LOW), output enable LOW (active LOW)
  digitalWrite(MASTER_RESET, HIGH); // turn off master reset
  digitalWrite(OUTPUT_ENABLE, LOW); // enable the output
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Sets the motor enable pins using a shift register.
// ARGUMENTS: enablePins - a 16-bit number where each bit corresponds to a motor enable pin
// RETURN VALUE: None
void SetMotorEnable(int enablePins){
  digitalWrite(MASTER_RESET, LOW); // turn on master reset
  digitalWrite(MASTER_RESET, HIGH); // turn off master reset

  // Shift out the 16-bit number
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, enablePins >> 8); // Shift out the high byte
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, enablePins & 0xFF); // Shift out the low byte
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}
