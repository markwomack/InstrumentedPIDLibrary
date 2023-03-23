/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 ********************************************************/

#include <InstrumentedPID.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

// Define Variables we'll be connecting to
double setpoint;
double pidInput;
double pidOutput;

// Define the aggressive and conservative Tuning Parameters
double aggKp=4;
double aggKi=0.2;
double aggKd=1;
double consKp=1;
double consKi=0.05;
double consKd=0.25;

//Specify the links and initial tuning parameters
InstrumentedPID myPID(consKp, consKi, consKd, DIRECT);

void setup() {
  //initialize the variables we're linked to
  pidInput = analogRead(PIN_INPUT);
  setpoint = 100;

  //turn the PID on
  myPID.start(pidInput, pidOutput);
}

void loop()
{
  pidInput = analogRead(PIN_INPUT);

  double gap = abs(setpoint - pidInput); //distance away from setpoint
  if (gap < 10) {
    //we're close to setpoint, use conservative tuning parameters
    myPID.setTunings(consKp, consKi, consKd);
  } else {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.setTunings(aggKp, aggKi, aggKd);
  }

  myPID.compute(pidInput, setpoint, &pidOutput);
  analogWrite(PIN_OUTPUT, pidOutput);
}
