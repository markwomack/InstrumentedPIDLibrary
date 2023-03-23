/********************************************************
 * PID Proportional on measurement Example
 * Setting the PID to use Proportional on measurement will 
 * make the output move more smoothly when the setpoint 
 * is changed.  In addition, it can eliminate overshoot
 * in certain processes like sous-vides.
 ********************************************************/

#include <InstrumentedPID.h>

// Define Variables we'll be connecting to
double setpoint;
double pidInput;
double pidOutput;

// Specify the links and initial tuning parameters
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior
InstrumentedPID myPID(2, 5, 1, P_ON_M, DIRECT);

void setup() {
  //initialize the variables we're linked to
  pidInput = analogRead(0);
  setpoint = 100;

  //turn the PID on
  myPID.start(pidInput, pidOutput);
}

void loop() {
  pidInput = analogRead(0);
  myPID.compute(pidInput, setpoint, &pidOutput);
  analogWrite(3, pidOutput);
}
