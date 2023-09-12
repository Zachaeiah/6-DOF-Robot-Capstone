#include <SD.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SD_CS_PIN BUILTIN_SDCARD // Define the chip select pin for the SD card module
 SPIF.setMOSI(7);  // Audio shield has MOSI on pin 7
 SPIF.setSCK(14);  // Audio shield has SCK on pin 14

const int capacity = 5; // Define the capacity for the motor list

typedef enum {
    FORWARD,
    BACKWARD
} Direction;

// Structure to hold motor settings
typedef struct Motor{
    char name[50];
    int index;
    double STEP_RESOLUTION;
    double current;
    int stepPin;
    int dirPin;
    int speed;
} Motor;

// Structure to create a list of motors
typedef struct MotorList {
    Motor* motors;
    int count;
}MotorList;

// Function to create a new motor
Motor createMotor(const char* name, int index, double step_resolution, double current,
                  int stepPin, int dirPin, int speed) {
    Motor motor;
    strncpy(motor.name, name, sizeof(motor.name) - 1);
    motor.index = index;
    motor.STEP_RESOLUTION = step_resolution;
    motor.current = current;
    motor.stepPin = stepPin;
    motor.dirPin = dirPin;
    motor.speed = speed;
    return motor;
}

// Function to create a list of motors
struct MotorList createMotorList(int capacity) {
    struct MotorList list;
    list.motors = (Motor*)malloc(capacity * sizeof(Motor));
    list.count = 0;
    return list;
}

void addMotorToList(struct MotorList* list, const Motor* motor) {
    if (list->count < capacity) {
        list->motors[list->count++] = *motor;
    }
}

// Function to save the motor list to a text file
void saveToTextFile(const char* filename, const struct MotorList* list) {
    File file = SD.open(filename, FILE_WRITE);
    if (file) {
        for (int i = 0; i < list->count; ++i) {
            file.print("Name: ");
            file.println(list->motors[i].name);
            file.print("Index: ");
            file.println(list->motors[i].index);
            file.print("Step Resolution: ");
            file.println(list->motors[i].STEP_RESOLUTION);
            file.print("Current: ");
            file.println(list->motors[i].current);
            file.print("Step Pin: ");
            file.println(list->motors[i].stepPin);
            file.print("Dir Pin: ");
            file.println(list->motors[i].dirPin);
            file.print("Speed: ");
            file.println(list->motors[i].speed);
            file.println(); // Add a blank line between motors
        }
        file.close();
    }
}

// Function to initialize the SD card
bool initializeSDCard() {
    if (!SD.begin(SD_CS_PIN)) {
        printf("Failed to initialize SD card.\n");
        return false;
    }
    return true;
}

// Function to read the motor list from the text file
struct MotorList readFromTextFile(const char* filename) {
    struct MotorList motorList = createMotorList(capacity);

    File file = SD.open(filename, FILE_READ);
    if (file) {
        char line[100];
        int index = 0;
        while (file.available()) {
            file.readBytesUntil('\n', line, sizeof(line));
            if (strncmp(line, "Name: ", 6) == 0) {
                sscanf(line, "Name: %s", motorList.motors[index].name);
            }
            else if (strncmp(line, "Index: ", 7) == 0) {
                sscanf(line, "Index: %d", &motorList.motors[index].index);
            }
            else if (strncmp(line, "Step Resolution: ", 17) == 0) {
                sscanf(line, "Step Resolution: %lf", &motorList.motors[index].STEP_RESOLUTION);
            }
            else if (strncmp(line, "Current: ", 9) == 0) {
                sscanf(line, "Current: %lf", &motorList.motors[index].current);
            }
            else if (strncmp(line, "Step Pin: ", 10) == 0) {
                sscanf(line, "Step Pin: %d", &motorList.motors[index].stepPin);
            }
            else if (strncmp(line, "Dir Pin: ", 9) == 0) {
                sscanf(line, "Dir Pin: %d", &motorList.motors[index].dirPin);
            }
            else if (strncmp(line, "Speed: ", 7) == 0) {
                sscanf(line, "Speed: %d", &motorList.motors[index].speed);
            }
            else if (strlen(line) == 0) {
                // Reached the end of motor settings, move to the next motor
                ++index;
            }
        }
        file.close();
    }
    return motorList;
}

void setup() {
    // Initialize the SD card
    if (!initializeSDCard()) {
        while (1) {
            // Error, SD card not initialized
        }
    }

    // Create a list of motors and set their settings
    struct MotorList motorList = createMotorList(capacity);

    Motor motor1 = createMotor("Motor1", 1, 1.8, 1.0, 11, 12, 1000);
    Motor motor2 = createMotor("Motor2", 2, 0.9, 0.5, 21, 22, 800);
    // Add more motors as needed...

    addMotorToList(&motorList, &motor1);
    addMotorToList(&motorList, &motor2);
    // Add more motors as needed...

    // Save the motor list to a text file
    saveToTextFile("motor_settings.txt", &motorList);

    // Free the motor list
    free(motorList.motors);

    // Read the motor list from the text file
    struct MotorList readMotorList = readFromTextFile("motor_settings.txt");

    // Display each motor's information
    Serial.begin(9600);
    Serial.println("Motor List:");
    for (int i = 0; i < readMotorList.count; ++i) {
        Serial.print("Motor Name: ");
        Serial.println(readMotorList.motors[i].name);
        Serial.print("Index: ");
        Serial.println(readMotorList.motors[i].index);
        Serial.print("Step Resolution: ");
        Serial.println(readMotorList.motors[i].STEP_RESOLUTION);
        Serial.print("Current: ");
        Serial.println(readMotorList.motors[i].current);
        Serial.print("Step Pin: ");
        Serial.println(readMotorList.motors[i].stepPin);
        Serial.print("Dir Pin: ");
        Serial.println(readMotorList.motors[i].dirPin);
        Serial.print("Speed: ");
        Serial.println(readMotorList.motors[i].speed);
        Serial.println();
    }

    // Free the motor list
    free(readMotorList.motors);

    while (1) {
        // Your main code goes here
    }
}

void loop() {
    // This loop is not used in this example, as we have a while loop in the setup function
}
