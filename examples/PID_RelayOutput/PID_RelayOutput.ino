/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/

#include <InstrumentedPID.h>

#define PIN_INPUT 0
#define RELAY_PIN 6

// Define Variables we'll be connecting to
double setpoint;
double pidInput;
double pidOutput;

// Specify the links and initial tuning parameters
double Kp=2;
double Ki=5;
double Kd=1;
InstrumentedPID myPID(Kp, Ki, Kd, DIRECT);

unsigned long windowSize = 5000;
unsigned long windowStartTime;

void setup() {
  windowStartTime = millis();

  //initialize the variables we're linked to
  setpoint = 100;

  //tell the PID to range between 0 and the full window size
  myPID.setOutputLimits(0, windowSize);

  //turn the PID on
  myPID.start(pidInput, pidOutput);
}

void loop() {
  pidInput = analogRead(PIN_INPUT);
  myPID.compute(pidInput, setpoint, &pidOutput);

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if ((millis() - windowStartTime) > windowSize) {
    //time to shift the Relay Window
    windowStartTime += windowSize;
  }
  
  if (pidOutput < (millis() - windowStartTime)) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }

}
