/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include "src/InstrumentedPID.h"

#define PIN_INPUT 0
#define PIN_OUTPUT 3

// Define Variables we'll be connecting to
double setpoint;
double pidInput;
double pidOutput;

StaticJsonDocument<512> jsonDoc;

// Specify the links and initial tuning parameters
double Kp=2;
double Ki=5;
double Kd=1;

InstrumentedPID myPID(Kp, Ki, Kd, DIRECT);

void setup() {
  // Initialize the variables we're linked to
  pidInput = analogRead(PIN_INPUT);
  setpoint = 100;

  // Turn the PID on
  myPID.start(pidInput, pidOutput);
}

void loop() {
  Serial.begin(9600);
  while (!Serial) continue;
  
  pidInput = analogRead(PIN_INPUT);
  if (myPID.compute(pidInput, setpoint, &pidOutput)) {
    analogWrite(PIN_OUTPUT, pidOutput);
  }
}
