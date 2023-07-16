#include <math.h>
#include <stdlib.h>
//---------------------------------------------------------------------------------------------------------------------

enum MOTOR_DIR { FORWARDS,
                 BACKWARDS };

typedef struct STEP_PLAN {
  double totalDelay;
  long* delays;
} STEP_PLAN;

typedef struct CONTROL_FEATURS {
  const long pulesRev;
  const double degStep;
  double maxSpeed;
  int direcstion;
} CONTROL_FEATURS;

typedef struct STEPPER_MOTOR {
  const char* name;
  const int stepPin;
  const int dirPin;
  const double current;
  CONTROL_FEATURS control;
} STEPPER_MOTOR;

typedef struct MOTOR_CONTROLLER {
  STEPPER_MOTOR Motors[6];
  size_t motorNum;

} MOTOR_CONTROLLER;

const int MotorPins_len = 12;
int MotorPins[MotorPins_len] = { 2, 3, 4, 5, 6, 9, 10, 11, 12, 26, 27, 30 };

//---------------------------------------------------------------------------------------------------------------------

STEPPER_MOTOR bace = { "bace", MotorPins[0], MotorPins[1], 2.84, 400, 1.1, 100. FORWARDS };
STEPPER_MOTOR shoulder = { "shoulder", MotorPins[2], MotorPins[3], 2.84, 400, 1.1, 100,  FORWARDS };
STEPPER_MOTOR elbow = { "elbow", MotorPins[4], MotorPins[5], 2.84, 400, 1.1, 100, 1, FORWARDS };
STEPPER_MOTOR elbowRevolut = { "elbowRevolut", MotorPins[6], MotorPins[7], 2.84, 400, 1.1, 100, FORWARDS };
STEPPER_MOTOR wrist = { "wrist", MotorPins[8], MotorPins[9], 2.84, 400, 1.1, 100, 1, FORWARDS };
STEPPER_MOTOR wristRevolut = { "wristRevolut", MotorPins[10], MotorPins[11], 2.84, 400, 1.1, 100, FORWARDS };

STEPPER_MOTOR robotMotors[6] = { bace, shoulder, elbow, elbowRevolut, wrist, wristRevolut };

//---------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  for (int pin = 0; pin < MotorPins_len; pin++) {
    pinMode(MotorPins[pin], OUTPUT);
  }

  long Steps[6] = { 1000, 2000, 4000, 8000, 16000, 32000 };

  MultyLinRotate(robotMotors, Steps);

  //singleLinRotate(wristRevolut, 5000);
}

//---------------------------------------------------------------------------------------------------------------------
void loop() {
}

void MultyLinRotate(STEPPER_MOTOR Motor[], long Steps[]) {

  long currentDelay;
  double motorPlans[6] = { 0 };
  double largestTime = 0;
  int largestTimeIndex = 0;
  double oldSpeeds[6] = { 0 };

  double maxSpeed = 0;
  double degStep = 0;
  double gostTime = 0;


  largestTime = motorPlans[0];

  for (int plan = 0; plan < 6; plan++) {

    maxSpeed = Motor[plan].control.maxSpeed;
    degStep = Motor[plan].control.degStep;

    for (int step = 0; step < Steps[plan]; step++) {
      currentDelay = (long)(((degStep * 60.0) / (maxSpeed * 360.0)) * pow(10, 6));
      motorPlans[plan] += (double)currentDelay / 1000000.0;
    }
    if (motorPlans[plan] > largestTime) {
      largestTime = motorPlans[plan];
      largestTimeIndex = plan;  // Track the index of the largest time
    }
  }

  Serial.print("Goal time is:");
  Serial.println(largestTime);

  for (int plan = 0; plan < 2; plan++) {
    if (plan == largestTimeIndex) {
      continue;
    }
    oldSpeeds[plan] = Motor[plan].control.maxSpeed;
    Motor[plan].control.maxSpeed = ((double)Steps[plan]) / largestTime;

    maxSpeed = Motor[plan].control.maxSpeed;
    degStep = Motor[plan].control.degStep;

    for (int step = 0; step < Steps[plan]; step++) {
      currentDelay = (long)(((degStep * 60.0) / (maxSpeed * 360.0)) * pow(10, 6));
      gostTime += (double)currentDelay / 1000000.0;
    }

    Serial.print("Motor: ");
    Serial.println(plan);
    Serial.print("old time");
    Serial.println(motorPlans[plan]);
    Serial.print("New time");
    Serial.println(gostTime);
    Serial.print("Old Speed: ");
    Serial.println(oldSpeeds[plan]);
    Serial.print("New Speed: ");
    Serial.println(Motor[plan].control.maxSpeed);
  }
}


// long LinAccelDeaccel(STEPPER_MOTOR Motor, long steps, long step) {
//   double maxSpeed = Motor.control.maxSpeed;
//   double slope = Motor.control.AccelPFP;
//   double degStep = Motor.control.degStep;
//   long delta = (long)(maxSpeed / slope);
//   long accelOffset = slope * steps;
//   long alfa = steps / 2;
//   double CurrentSpeed = 0;
//   long plulseDealay = 0;

//   if (steps > (2 * delta)) {
//     if ((0 <= step) && (step <= delta)) {
//       CurrentSpeed = slope * step;
//     } else if ((delta < step) && (step <= (steps - delta))) {
//       CurrentSpeed = maxSpeed;
//     } else if (((steps - delta) < step) && (step <= steps)) {
//       CurrentSpeed = -slope * step + accelOffset;
//     } else if (steps < step) {
//       CurrentSpeed = 0;
//     }
//   } else if (steps <= 2 * delta) {
//     if ((0 <= step) && (step <= alfa)) {
//       CurrentSpeed = slope * step;
//     } else if (step > alfa) {
//       CurrentSpeed = -slope * step + (2 * alfa);
//     }
//   }

//   plulseDealay = (long)(((degStep * 60.0) / (CurrentSpeed * 360.0)) * pow(10, 6));

//   if (plulseDealay > 50000) {
//     plulseDealay = 10000;
//   }

//   return plulseDealay;
// }

//---------------------------------------------------------------------------------------------------------------------
void singleLinRotate(STEPPER_MOTOR Motor, long steps) {

  long plulseDealay = 0;
  double maxSpeed = Motor.control.maxSpeed;
  double degStep = Motor.control.degStep;
  for (int step = 1; step <= steps; step++) {
    plulseDealay = (long)(((degStep * 60.0) / (maxSpeed * 360.0)) * pow(10, 6));
    singleStep(Motor, plulseDealay);
  }
}



//---------------------------------------------------------------------------------------------------------------------
void singleStep(STEPPER_MOTOR motor, long plulseDealay) {
  digitalWrite(motor.dirPin, motor.control.direcstion);
  digitalWrite(motor.stepPin, HIGH);
  delayMicroseconds(plulseDealay);
  digitalWrite(motor.stepPin, LOW);
  delayMicroseconds(plulseDealay);
}


//---------------------------------------------------------------------------------------------------------------------
void solveSystem(float A[][4], float b[], float x[], int size) {
  // Gaussian elimination algorithm

  // Forward elimination
  for (int k = 0; k < size - 1; k++) {
    for (int i = k + 1; i < size; i++) {
      float factor = A[i][k] / A[k][k];
      for (int j = k; j < size; j++) {
        A[i][j] -= factor * A[k][j];
      }
      b[i] -= factor * b[k];
    }
  }

  // Back-substitution
  x[size - 1] = b[size - 1] / A[size - 1][size - 1];
  for (int i = size - 2; i >= 0; i--) {
    float sum = b[i];
    for (int j = i + 1; j < size; j++) {
      sum -= A[i][j] * x[j];
    }
    x[i] = sum / A[i][i];
  }
}
