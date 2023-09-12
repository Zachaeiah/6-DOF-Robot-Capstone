//---------------------------- library prototypes ---------------------------------------------------------------
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>     // math functions
#include <string.h>   // string functions
#include <ctype.h>    // character functions
#include <stdbool.h>  // bool definitions

//---------------------------- Program Constants ----------------------------------------------------------------------


const double ABS_THETA_DEG_MAX[6] = { 180.0 };  // Maximum magnitude of bace angle in degrees

const int COMMAND_INDEX_NOT_FOUND = -1;  // Index value indicating command not found

const double DEFULT_STEP_PER_REV = 75000;
const double RSOLUTIONS[16] = { 1, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.4, 0.2, 0.1, 0.08, 0.05, 0.04, 0.02, 0.015 };
const double STEP_RESOLUTIONS[6] = {
  RSOLUTIONS[0],
  RSOLUTIONS[0],
  RSOLUTIONS[0],
  RSOLUTIONS[0],
  RSOLUTIONS[0],
  RSOLUTIONS[0]
};  // Possible resolutions for the driver

const long STEP_PER_REV[6] = {
  (long)(DEFULT_STEP_PER_REV / STEP_RESOLUTIONS[0]),
  (long)(DEFULT_STEP_PER_REV / STEP_RESOLUTIONS[1]),
  (long)(DEFULT_STEP_PER_REV / STEP_RESOLUTIONS[2]),
  (long)(DEFULT_STEP_PER_REV / STEP_RESOLUTIONS[3]),
  (long)(DEFULT_STEP_PER_REV / STEP_RESOLUTIONS[4]),
  (long)(DEFULT_STEP_PER_REV / STEP_RESOLUTIONS[5])
};  // Steps per rotation

const double DEG_PER_STEP[6] = {
  360.0 / STEP_PER_REV[0],
  360.0 / STEP_PER_REV[1],
  360.0 / STEP_PER_REV[2],
  360.0 / STEP_PER_REV[3],
  360.0 / STEP_PER_REV[4],
  360.0 / STEP_PER_REV[5],
};  // Degrees per step

const long baceHomeAngle = 0;          // Home angle for the bace joint
const long shoulderHomeAngle = 0;      // Home angle for the shoulder joint
const long elbowHomeAngle = 0;         // Home angle for the elbow joint
const long elbowRevolutHomeAngle = 0;  // Home angle for the elbowRevolut joint
const long wristHomeAngle = 0;         // Home angle for the wrist joint
const long wristRevolutHomeAngle = 0;  // Home angle for the wristRevolut joint

const char* seps = "\t,\n ;:";  // Delimiters for tokenizing the line string
const double ERROR_VALUE = -1;  // Error value

#define COMMAND_STRING_ARRAY_SIZE 502  // size of array to store commands written by sprintf_s for robot. 
                                       // NOTE: 2 elements must be reserved for trailing '\n' and '\0'

#define MAX_LINE_SIZE 1002  // size of array to store a line from a file. 
                            // NOTE: 2 elements must be reserved for trailing '\n' and '\0'


enum MOTOR_SPEED { MOTOR_SPEED_LOW,
                   MOTOR_SPEED_MEDIUM,
                   MOTOR_SPEED_HIGH };  // motor speed

enum CURRENT_ANGLES { GET_CURRENT_ANGLES,
                      UPDATE_CURRENT_ANGLES };  // used to get/update current SCARA angles

enum COMMAND_INDEX  // list of all command indexes
{
  ROTATE_JOINT,    // Set all six angles
  MOTOR_SPEED,     // set the robot rotashion speed
  GRIPPER_CLOSE,   // close the gripper
  GRIPPER_OPEN,    // open the gripper
  GRIPPER_OFFSET,  // Set the Gripper offset
  END,             // turn the robot off
  HOME,            // move the robot to its home posishion
  NUM_COMMANDS     // The Number of commands
};

enum MOTOR_NAMES { BACE,
                   SHOULDER,
                   ELBOW,
                   ELBOWREVOLUT,
                   WRIST,
                   WRISTREVOLUT };

//---------------------------- Program Global ----------------------------------------------------------------------

// Defines pins for bace stepper motor
#define baceStepPin 2
#define baceDirPin 3

// Defines pins for shoulder stepper motor
#define shoulderStepPin 4
#define shoulderDirPin 5

// Defines pins for elbow stepper motor
#define elbowStepPin 9
#define elbowDirPin 10

// Defines pins for elbow Revolut stepper motor
#define elbowRevolutStepPin 11
#define elbowRevolutDirPin 12

// Defines pins for wrist stepper motor
#define wristStepPin 30
#define wristDirPin 31

// Defines pins for wrist Revolut stepper motor
#define wristRevolutStepPin 32
#define wristRevolutDirPin 33

Servo Gripper;       // Gripper servo
Servo GripperAngle;  // Gripper angle servo


// structure to map command keyword string to a command index
typedef struct COMMAND {
  const int index;
  const char* strCommand;
} COMMAND;

// robot angles
typedef struct JOINT_ANGLES {
  double thetaDeg[6] = { 0 };
} JOINT_ANGLES;

// tooltip coordinates
typedef struct TOOL_ORIENTATION {
  double theta1Deg, theta2Deg;
  double x, y, z;  // the poshion of the grippers active area
} TOOL_ORIENTATION;


// Setup the stepper motor
MultiStepper steppersControl;

AccelStepper bace(AccelStepper::FULL2WIRE, baceStepPin, baceDirPin);
AccelStepper shoulder(AccelStepper::FULL2WIRE, shoulderStepPin, shoulderDirPin);
AccelStepper elbow(AccelStepper::FULL2WIRE, elbowStepPin, elbowDirPin);
AccelStepper elbowRevolut(AccelStepper::FULL2WIRE, elbowRevolutStepPin, elbowRevolutDirPin);
AccelStepper wrist(AccelStepper::FULL2WIRE, wristStepPin, wristDirPin);
AccelStepper wristRevolut(AccelStepper::FULL2WIRE, wristRevolutStepPin, wristRevolutDirPin);



//----------------------------- Globals -------------------------------------------------------------------------------
// global array of command keyword string to command index associations
const COMMAND m_Commands[NUM_COMMANDS] = { { ROTATE_JOINT, "ROTATE_JOINT" }, { MOTOR_SPEED, "MOTOR_SPEED" }, { GRIPPER_CLOSE, "GRIPPER_CLOSE" }, { GRIPPER_OPEN, "GRIPPER_OPEN" }, { GRIPPER_OFFSET, "GRIPPER_OFFSET" }, { END, "END" }, { HOME, "HOME" } };

//----------------------------- Function Prototypes -------------------------------------------------------------------

int nint(double);                              // Computes the nearest integer from a double value
double degToRad(double);                       // Returns angle in radians from input angle in degrees
double radToDeg(double);                       // Returns angle in degrees from input angle in radians
double mapAngle(double);                       // Make sure inverseKinematic angles are mapped in range the robot understands
void makeStringUpperCase(char*);               // Makes an input string all uppercase
void robotAngles(JOINT_ANGLES*, int);          // Gets or updates the current SCARA angles
bool setJointAngles(char*);                    // Set the angles of the joints based on the input command string
bool setMotorSpeed(char*);                     // Set the motor speed based on the input command string
bool setGripperState(char*);                   // Turn the gripper on or off based on the input command string
bool setGripperRevolutAngle(char*);            // Set the gripper angle based on the input command string
void processCommand(int commandIndex, char*);  // Processes a command string from the Serial input
void RunStepperAngles(JOINT_ANGLES);           // Set the angles of the robot's joints and run the stepper motors accordingly
void stepperMotorSetup();                      // Setup everything for the stepper motors
void ServoSetup();                             // Setup the servo motors
int getCommandIndex(char*);                    // Get the command keyword index from a string

void setup() {

  // JOINT_ANGLES Home = { baceHomeAngle, shoulderHomeAngle, elbowHomeAngle,
  //                       elbowRevolutHomeAngle, wristHomeAngle, wristRevolutHomeAngle };  // Define home angles for the robot

  // robotAngles(&Home, UPDATE_CURRENT_ANGLES);  // Update the robot's current angles to the home angles
  // ServoSetup();                               // Setup the servo motor

  bace.setMaxSpeed(800);
  shoulder.setMaxSpeed(800);
  elbow.setMaxSpeed(800);
  elbowRevolut.setMaxSpeed(2000);
  wrist.setMaxSpeed(2000);
  wristRevolut.setMaxSpeed(2000);

  // add the each stepper motor to the controller
  steppersControl.addStepper(bace);
  steppersControl.addStepper(shoulder);
  steppersControl.addStepper(elbow);
  steppersControl.addStepper(elbowRevolut);
  steppersControl.addStepper(wrist);
  steppersControl.addStepper(wristRevolut);

  Serial.begin(9600);  // Start serial communication at 9600 baud

  while (!Serial);

  // JOINT_ANGLES testAngles = { { 0, 0, 0, 0, 0, 0 } };


  // testAngles.thetaDeg[WRIST] = 90;
  // RunStepperAngles(testAngles);
  // delay(500);

  // testAngles.thetaDeg[ELBOWREVOLUT] = 90;
  // RunStepperAngles(testAngles);
  // delay(500);

  // testAngles.thetaDeg[WRISTREVOLUT] = 180;
  // RunStepperAngles(testAngles);
  // delay(500);

  // testAngles.thetaDeg[ELBOWREVOLUT] = 0;
  // testAngles.thetaDeg[WRISTREVOLUT] = 0;
  // RunStepperAngles(testAngles);
  // delay(500);

  // testAngles.thetaDeg[WRIST] = 0;
  // RunStepperAngles(testAngles);
  // delay(500);

  // Serial.println("done");
}

char strLine[MAX_LINE_SIZE] = { '\0' };  // Define input buffer
int commandIndex = -1;                   // Command index
char *token = NULL, *next_token = NULL;  // Token pointers for command processing


void loop() {
  if (Serial.available() > 0) {                         // If there is data available
    char incoming = Serial.read();                      // Read the incoming character
    if (incoming == '\n') {                             // If the incoming character is a newline
      strLine[strnlen(strLine, MAX_LINE_SIZE)] = '\0';  // Terminate the input string

      Serial.println("Prossessing command");
      // Get the command index and process it
      makeStringUpperCase(strLine);  // Make line string all uppercase (makes commands case-insensitive)
      commandIndex = getCommandIndex(strLine);
      token = strtok(strLine, seps);
      token = strtok(NULL, "");  // Get the rest of the line as the next token
      processCommand(commandIndex, token);

      // Reset command variable to wait for the next command
      memset(strLine, '\0', sizeof(strLine));                         // Clear the input buffer
      Serial.println("Done prossessing command");
    } else {                                                          // If the incoming character is not a newline
      if (strnlen(strLine, MAX_LINE_SIZE - 1) < MAX_LINE_SIZE - 2) {  // If there is room in the input buffer
        strncat(strLine, &incoming, 1);                               // Add the incoming character to the input buffer
      } else {                                                        // If the input buffer is full
        Serial.println("Input buffer full, command ignored.");
      }
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  processes a command referenced by the commandIndex.  Parses the command string from the file and
//               packages up the command to be sent to the robot if no errors found.
// ARGUMENTS:    commandIndex:  index of the command keyword string
//               strCommandLine: command line from the file in the form of a string
// RETURN VALUE: none
void processCommand(int commandIndex, char* strCommandLine) {
  bool bSuccess = true;

  switch (commandIndex) {
    case ROTATE_JOINT:
      Serial.println("ROTATE_JOINT command selected");
      bSuccess = setJointAngles(strCommandLine);
      break;

    case MOTOR_SPEED:
      Serial.println("MOTOR_SPEED command selected");
      bSuccess = setMotorSpeed(strCommandLine);
      break;

    case GRIPPER_CLOSE:
      Serial.println("GRIPPER_CLOSE command selected, N.A");
      break;

    case GRIPPER_OPEN:
      Serial.println("GRIPPER_OPEN command selected, N.A");
      break;

    case GRIPPER_OFFSET:
      Serial.println("GRIPPER_OFFSET command selected, N.A");
      break;

    case END:
      Serial.println("END command selected, N.a");
      break;

    case HOME:
      Serial.println("HOME command selected, N.a");
      break;

    default:
      Serial.println("unknown command!\n");
  }
  if (bSuccess) Serial.println("Command sent to robot!\n\n");
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  will set all the joint and revolut angles
// ARGUMENTS:    angles: holdes all 6 angles for all the joints
// RETURN VALUE: none
void RunStepperAngles(JOINT_ANGLES angles) {
  long MAX_STEP_COUNT = 0;
  double angle = 0;
  double scalingFactor = 0;
  long stepCount = 0;
  long angleStep[6] = { 0 };

  for (int i = 0; i < 6; i++) {
    MAX_STEP_COUNT = STEP_PER_REV[i];
    angle = angles.thetaDeg[i];
    scalingFactor = MAX_STEP_COUNT / 360.0;
    stepCount = (long)(angle * scalingFactor);

    // Handle out-of-range step counts
    stepCount = stepCount < (-MAX_STEP_COUNT / 2) ? (-MAX_STEP_COUNT / 2) : stepCount;
    stepCount = stepCount > (MAX_STEP_COUNT / 2) ? (MAX_STEP_COUNT / 2) : stepCount;

    angleStep[i] = stepCount;

    Serial.print("angle ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(angleStep[i]);
  }
  steppersControl.moveTo(angleStep);
  steppersControl.runSpeedToPosition();  // Blocks until all are in position

  robotAngles(&angles, UPDATE_CURRENT_ANGLES);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: will fined what index the command is
// ARGUMENTS:    strLine:  A file line string.
// RETURN VALUE: the index of the command
int getCommandIndex(char* strLine) {

  char strLine2[MAX_LINE_SIZE];
  strcpy(strLine2, strLine);

  // find which command matches the known commands
  for (int n = 0; n <= NUM_COMMANDS; n++) {
    // is the given string the same as the N'th command
    if (strstr(strLine2, m_Commands[n].strCommand) != NULL) {
      return m_Commands[n].index;
    }
  }

  // if the command was not found
  return -1;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  will set the joints to the given angles
// ARGUMENTS:    the string wiht the info for seting the angles
// RETURN VALUE: true if the command ran, false if not
bool setJointAngles(char* strCommandLine) {
  const int paramaterNnumber = 6;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  JOINT_ANGLES angles = { 0, 0 };  // holds the angles of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      angles.thetaDeg[i] = (double)strtod(token, &token);

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    RunStepperAngles(angles);
  }
  return goodCom;
}



//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  will set the Gripper angle to the given angle
// ARGUMENTS:    the string wiht the info for seting the angle
// RETURN VALUE: true if the command ran, false if not
bool setMotorSpeed(char* strCommandLine) {
  const int paramaterNnumber = 1;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  double motorSpeed[6] = { 0 };    // holds the angle of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      motorSpeed[i] = (double)strtod(token, &token);  // set the first token to the theta1Deg

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    bace.setMaxSpeed(motorSpeed[BACE]);
    shoulder.setMaxSpeed(motorSpeed[SHOULDER]);
    elbow.setMaxSpeed(motorSpeed[ELBOW]);
    elbowRevolut.setMaxSpeed(motorSpeed[ELBOWREVOLUT]);
    wrist.setMaxSpeed(motorSpeed[WRIST]);
    wristRevolut.setMaxSpeed(motorSpeed[WRISTREVOLUT]);
  }
  return goodCom;
}

bool setGripperState(char* strCommandLine) {
  const int paramaterNnumber = 1;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  long GripperState = 0;           // holds the angle of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      if (i == 0) GripperState = (double)strtol(token, &token, 10);  // set the first token to the theta1Deg

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    if (GripperState > 0) {
      Gripper.write(500);
    } else if (GripperState <= 0) {
      Gripper.write(800);
    }
  }
  return goodCom;
}

bool setGripperAngle(char* strCommandLine) {
  const int paramaterNnumber = 1;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  double angle = 0;                // holds the angle of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      if (i == 0) angle = (double)strtod(token, &token);  // set the first token to the theta1Deg

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    angle = map(angle, -90, 90, 230, 720);
    GripperAngle.write((long)angle);
  }
  return goodCom;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle understood by the robot (-PI <= ang <= +PI)
// ARGUMENTS:    ang: the angle in radians
// RETURN VALUE: the mapped angle in radians
double mapAngle(double angRad) {
  angRad = fmod(angRad, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

  // map into range -PI <= ang <= +PI
  if (angRad > PI)
    angRad -= 2.0 * PI;
  else if (angRad < -PI)
    angRad += 2.0 * PI;

  return angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg) {
  return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad) {
  return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  computes nearest integer to given double
// ARGUMENTS:    d: double value
// RETURN VALUE: nearest int
int nint(double d) {
  return (int)floor(d + 0.5);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  makes a string all upper case characters
// ARGUMENTS:    str:  the string memory address
// RETURN VALUE: none
void makeStringUpperCase(char* str) {
  if (str == NULL) return;  // safety!

  for (size_t i = 0; i < strlen(str); i++) str[i] = (char)toupper(str[i]);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  get or update current robot shoulder and elbow angles
// ARGUMENTS:    pAngles:  shoulder/joint angles.
//               getOrUpdate:  set to UPDATE_CURRENT_ANGLES to update the current angles
//                             set to GET_CURRENT_ANGLES to retrieve the current angles
// RETURN VALUE: none
void robotAngles(JOINT_ANGLES* pAngles, int getOrUpdate) {
  static JOINT_ANGLES currentAngles = { baceHomeAngle, shoulderHomeAngle, elbowHomeAngle,
                                        elbowRevolutHomeAngle, wristHomeAngle, wristRevolutHomeAngle };  // NOTE:  robot must be in home position when program starts!

  if (pAngles == NULL)  // safety
  {
    Serial.println("NULL JOINT_ANGLES pointer! (robotAngles)");
    return;
  }

  if (getOrUpdate == UPDATE_CURRENT_ANGLES)
    currentAngles = *pAngles;
  else if (getOrUpdate == GET_CURRENT_ANGLES)
    *pAngles = currentAngles;
  else
    Serial.println("Unknown value for getOrUpdate (robotAngles)");
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  float result = 0;
  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return result;
}

size_t strnlen(const char* s, size_t len) {
  size_t i = 0;
  for (; i < len && s[i] != '\0'; ++i)
    ;
  return i;
}
