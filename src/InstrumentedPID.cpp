/******************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License.
 *
 * Reformatted and modernized by Mark Womack (https://github.com/markwomack).
 * Functionality is the same, but the method names have changed slightly.
 *****************************************************************************/

#include <Arduino.h>

#include "InstrumentedPID.h"

/* The parameters specified here are those for for which we can't set up
 * reliable defaults, so we need to have the user set them.
 */
InstrumentedPID::InstrumentedPID(double Kp, double Ki, double Kd,
    PIDMode pidMode, Direction direction) {
  _isRunning = false;

  // Default output limit corresponds to the arduino pwm limits
  setOutputLimits(0, 255);

  // Default Controller Sample Time is 0.1 seconds
  _sampleTime = 100;

  setDirection(direction);
  setTunings(Kp, Ki, Kd, pidMode);

  _lastTime = millis() - _sampleTime;
}

/* To allow backwards compatability for v1.1, or for people that just want
 * to use Proportional on Error without explicitly saying so
 */
InstrumentedPID::InstrumentedPID(double Kp, double Ki, double Kd,
    Direction direction):InstrumentedPID::InstrumentedPID(Kp, Ki, Kd,
      P_ON_E, direction) {
  // Nothing else to be done
}

// Starts the PID so it can start computing.
void InstrumentedPID::start(double initialInput, double initialOutput) {
  _outputSum = max(_outMin, min(initialOutput, _outMax));
  _lastInput = initialInput;
  _isRunning = true;
}

// Stops the PID, calls to compute() will do nothing.
void InstrumentedPID::stop() {
  _isRunning = false;
}
    
/* This, as they say, is where the magic happens.  this function should be
 * called every time "void loop()" executes.  the function will decide for
 * itself whether a new pid Output needs to be computed.  returns true when
 * the output is computed, false when nothing has been done.
 */
bool InstrumentedPID::compute(double input, double setpoint, double* output) {

   if (!_isRunning) {
     return false;
   }
   
   unsigned long now = millis();
   unsigned long timeChange = (now - _lastTime);
   
   if (timeChange >= _sampleTime) {
     // Compute all the working error variables
     double error = setpoint - input;
     double dInput = (input - _lastInput);
     _outputSum += (_Ki * error);

     // Add Proportional on Measurement, if P_ON_M is specified
     if (_pidMode == P_ON_M) {
       // Add Proportional on Measurement, if P_ON_M is specified
       _outputSum -= _Kp * dInput;
     } else {
       // Add Proportional on Error, if P_ON_E is specified
       _outputSum += _Kp * error;
     }

     _outputSum = max(_outMin, min(_outputSum, _outMax));

     // Compute Rest of PID Output
     _outputSum -= _Kd * dInput;
     
     // Return the output value
     *output = max(_outMin, min(_outputSum, _outMax));

     // Remember some variables for next time
     _lastInput = input;
     _lastTime = now;
     return true;
  }

  return false;
}

/* This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 */
void InstrumentedPID::setTunings(double Kp, double Ki, double Kd, PIDMode pidMode) {

  if (Kp < 0 || Ki < 0 || Kd < 0) {
    return;
  }

  _pidMode = pidMode;

  _dispKp = Kp;
  _dispKi = Ki;
  _dispKd = Kd;

  double sampleTimeInSec = ((double)_sampleTime)/1000;
  _Kp = Kp;
  _Ki = Ki * sampleTimeInSec;
  _Kd = Kd / sampleTimeInSec;

  if (_direction == REVERSE) {
    _Kp = (0 - _Kp);
    _Ki = (0 - _Ki);
    _Kd = (0 - _Kd);
  }
}

/* Set Tunings using the current POn setting
 */
void InstrumentedPID::setTunings(double Kp, double Ki, double Kd){
  setTunings(Kp, Ki, Kd, _pidMode); 
}

/* Sets the period, in milliseconds, at which the calculation is performed
 */
void InstrumentedPID::setSampleTime(unsigned long sampleTime)
{
  if (sampleTime > 0) {
    double ratio  = (double)sampleTime / (double)_sampleTime;
    _Ki *= ratio;
    _Kd /= ratio;
    _sampleTime = (unsigned long)_sampleTime;
  }
}

/* This function will be used far more often than SetInputLimits. While
 * the input to the controller will generally be in the 0-1023 range (which is
 * the default already), the output will be a little different. Maybe they'll
 * be doing a time window and will need 0-8000 or something. Or maybe they'll
 * want to clamp it from 0-125. Who knows. At any rate, that can all be done
 * here.
 */
void InstrumentedPID::setOutputLimits(double outputMin, double outputMax)
{
  if (outputMin >= outputMax) {
    return;
  }
  _outMin = outputMin;
  _outMax = outputMax;

  if (_isRunning) {
    _outputSum = max(_outMin, min(_outputSum, _outMax));
  }
}

/* The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 */
void InstrumentedPID::setDirection(Direction direction)
{
   if (_isRunning && direction != _direction) {
    _Kp = (0 - _Kp);
    _Ki = (0 - _Ki);
    _Kd = (0 - _Kd);
   }
   _direction = direction;
}

/* Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 */
double InstrumentedPID::getKp(){ return  _dispKp; }
double InstrumentedPID::getKi(){ return  _dispKi;}
double InstrumentedPID::getKd(){ return  _dispKd;}
bool InstrumentedPID::isRunning(){ return _isRunning; }
PIDMode InstrumentedPID::getPIDMode(){ return _pidMode; }
Direction InstrumentedPID::getDirection(){ return _direction; }

