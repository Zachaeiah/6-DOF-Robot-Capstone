#include <math.h>
//---------------------------------------------------------------------------------------------------------------------
typedef struct CONTROL_FEATURS {
  const long pulesRev;
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
//---------------------------------------------------------------------------------------------------------------------

STEPPER_MOTOR Motor1 = { "test3", 6, 9, 2.84, 400, 100, 1 };

//---------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(Motor1.dirPin, OUTPUT);
  pinMode(Motor1.stepPin, OUTPUT);

  singleLinRotate(Motor1, 5000);

}

//---------------------------------------------------------------------------------------------------------------------
void loop() {
}

//---------------------------------------------------------------------------------------------------------------------
void singleLinRotate(STEPPER_MOTOR motor, long steps) {

  long plulseDealay = 0;
  double degStep = 360 / (double)(motor.control.pulesRev);
  for (int step = 1; step <= steps; step++) {
    plulseDealay = ((degStep * 60.0) / (motor.control.maxSpeed * 360)) * pow(10, 6);
    Serial.println(plulseDealay);
    singleStep(motor, plulseDealay);
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
