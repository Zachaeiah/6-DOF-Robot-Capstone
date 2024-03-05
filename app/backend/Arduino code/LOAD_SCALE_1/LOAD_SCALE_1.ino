#include "Arduino.h"
#include "CAN_COMMUNICATION.h"
#include <HX711.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

// use scale.set_offset(4294948396); and scale.set_scale(611.934936);

#define dataPin 3
#define clockPin 2

const char LCD_ADDRESS = 0x27;
const char LCD_DIM[2] = {0x14, 0x2};

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_DIM[0], LCD_DIM[1]);
HX711 myScale;

void calibrateScale();
void setupScale();


void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight(); // Turn on the backlight
  myScale.begin(dataPin, clockPin);

  //calibrateScale();
  setupScale();
}


void loop() {
  if (myScale.is_ready()) {
    float weight = myScale.get_units(1); // Get weight reading from the load cell

    // Convert weight to 2-byte integer (grams)
    uint16_t weightInt = static_cast<uint16_t>(weight);
    unsigned char msgData[2];
    msgData[0] = weightInt & 0xFF; // Lower byte
    msgData[1] = (weightInt >> 8) & 0xFF; // Upper byte

    // Print weight to LCD screen
    lcd.clear(); // Clear the LCD screen
    lcd.setCursor(0, 0); // Set cursor to the first column of the first row
    lcd.print("Weight: "); // Print label
    lcd.setCursor(0, 1);
    lcd.print(weight); // Print weight
  }
  delay(800); // Delay before next loop iteration
}

bool sendWeightOverCAN(unsigned char *msg, unsigned char len) {
  unsigned long id = 0x123; // Choose a CAN message ID for weight data
  return sendCANMessage(id, msg, len); // Send weight data over CAN
}

void calibrateScale()
{

  Serial.println("\n\nCALIBRATION\n===========");
  Serial.println("remove all weight from the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println("and press enter\n");
  while (Serial.available() == 0);

  Serial.println("Determine zero weight offset");
  myScale.tare(20);  // average 20 measurements.
  uint32_t offset = myScale.get_offset();

  Serial.print("OFFSET: ");
  Serial.println(offset);
  Serial.println();


  Serial.println("place a weight on the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println("enter the weight in (whole) grams and press enter");
  uint32_t weight = 0;
  while (Serial.peek() != '\n')
  {
    if (Serial.available())
    {
      char ch = Serial.read();
      if (isdigit(ch))
      {
        weight *= 10;
        weight = weight + (ch - '0');
      }
    }
  }
  Serial.print("WEIGHT: ");
  Serial.println(weight);
  myScale.calibrate_scale(weight, 20);
  float scale = myScale.get_scale();

  Serial.print("SCALE:  ");
  Serial.println(scale, 6);

  Serial.print("\nuse scale.set_offset(");
  Serial.print(offset);
  Serial.print("); and scale.set_scale(");
  Serial.print(scale, 6);
  Serial.print(");\n");
  Serial.println("in the setup of your project");

  // Store offset and scale to EEPROM
  EEPROM.put(0, offset);
  EEPROM.put(sizeof(uint32_t), scale);

  Serial.println("\nStored offset and scale to EEPROM.\n\n");
}

void setupScale() {
  // Read offset and scale from EEPROM
  uint32_t storedOffset;
  float storedScale;
  EEPROM.get(0, storedOffset);
  EEPROM.get(sizeof(uint32_t), storedScale);

  // Set offset and scale to HX711 scale object
  myScale.set_offset(storedOffset);
  myScale.set_scale(storedScale);
}
