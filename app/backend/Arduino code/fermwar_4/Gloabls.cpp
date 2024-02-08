#include "Gloabls.h"

void dsprintString(const char *message) {
  // Serial print the message
  Serial.println(message);

  // Save the message to the SD card
  File dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(message);
    dataFile.close();
  } else {
    Serial.println("Error opening file for writing!");
  }
}

void dsprintInt(int value) {
  // Serial print the value
  Serial.println(value);

  // Save the value to the SD card
  File dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(value);
    dataFile.close();
  } else {
    Serial.println("Error opening file for writing!");
  }
}

void dsprintLong(long value) {
  // Serial print the value
  Serial.println(value);

  // Save the value to the SD card
  File dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(value);
    dataFile.close();
  } else {
    Serial.println("Error opening file for writing!");
  }
}

void dsprintDouble(double value) {
  // Serial print the value
  Serial.println(value);

  // Save the value to the SD card
  File dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(value);
    dataFile.close();
  } else {
    Serial.println("Error opening file for writing!");
  }
}


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Map a float value from one range to another
// ARGUMENTS:   x - Value to map
//              in_min - Lower bound of the input range
//              in_max - Upper bound of the input range
//              out_min - Lower bound of the output range
//              out_max - Upper bound of the output range
// RETURN VALUE: Mapped value
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  float result = 0;
  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return result;
}
