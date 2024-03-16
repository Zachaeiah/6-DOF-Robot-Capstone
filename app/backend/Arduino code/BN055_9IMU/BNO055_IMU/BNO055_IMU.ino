#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool calibrateGyro = false;

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData, gravityData;
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  if (calibrateGyro) {
    bno.setExtCrystalUse(true);
    delay(100);
    Serial.println("Calibrating gyro... Please keep the sensor steady.");
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.setSensorOffsets(bno.getQuat());
    calibrateGyro = false;
  } else {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  }

  printEvent(&gravityData);
  printOrientation(&orientationData);

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "SET") {
      calibrateGyro = true;
    }
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printEvent(sensors_event_t* event) {
  Serial.print("Gravity:\tx=");
  Serial.print(event->acceleration.x);
  Serial.print("\t|\ty=");
  Serial.print(event->acceleration.y);
  Serial.print("\t|\tz=");
  Serial.println(event->acceleration.z);
}

void printOrientation(sensors_event_t* event) {
  Serial.print("Orientation:\tHeading = ");
  Serial.print(event->orientation.x);
  Serial.print("°\t|\tRoll = ");
  Serial.print(event->orientation.y);
  Serial.print("°\t|\tPitch = ");
  Serial.print(event->orientation.z);
  Serial.println("°");
}
