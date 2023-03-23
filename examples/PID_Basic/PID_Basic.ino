/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <InstrumentedPID.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

// Define Variables we'll be connecting to
double setpoint;
double input;
double output;

// Specify the links and initial tuning parameters
double Kp=2;
double Ki=5;
double Kd=1;

InstrumentedPID myPID(Kp, Ki, Kd, DIRECT);

void setup() {
  // Initialize the variables we're linked to
  input = analogRead(PIN_INPUT);
  setpoint = 100;

  // Turn the PID on
  myPID.start(input, output);
}

void loop() {
  input = analogRead(PIN_INPUT);
  myPID.compute(input, setpoint, &output);
  analogWrite(PIN_OUTPUT, output);
}
