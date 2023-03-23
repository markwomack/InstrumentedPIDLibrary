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

#include "PID_v1.h"

/* The parameters specified here are those for for which we can't set up
 * reliable defaults, so we need to have the user set them.
 */
PID::PID(double* inputPtr, double* outputPtr, double* setpointPtr,
         double Kp, double Ki, double Kd, int pOn, int controllerDirection) {
  _inputPtr = inputPtr;
  _outputPtr = outputPtr;
  _setpointPtr = setpointPtr;
  _inAuto = false;

  // Default output limit corresponds to the arduino pwm limits
  setOutputLimits(0, 255);

  // Default Controller Sample Time is 0.1 seconds
  _sampleTime = 100;

  setControllerDirection(controllerDirection);
  setTunings(Kp, Ki, Kd, pOn);

  _lastTime = millis() - _sampleTime;
}

/* To allow backwards compatability for v1.1, or for people that just want
 * to use Proportional on Error without explicitly saying so
 */
PID::PID(double* inputPtr, double* outputPtr, double* setpointPtr,
        double Kp, double Ki, double Kd, int controllerDirection)
    :PID::PID(inputPtr, outputPtr, setpointPtr, Kp, Ki, Kd,
              P_ON_E, controllerDirection) {
  // Nothing else to be done
}

/* This, as they say, is where the magic happens.  this function should be
 * called every time "void loop()" executes.  the function will decide for
 * itself whether a new pid Output needs to be computed.  returns true when
 * the output is computed, false when nothing has been done.
 */
bool PID::compute() {

   if (!_inAuto) {
     return false;
   }
   
   unsigned long now = millis();
   unsigned long timeChange = (now - _lastTime);
   
   if (timeChange >= _sampleTime) {
     // Compute all the working error variables
     double input = *_inputPtr;
     double error = *_setpointPtr - input;
     double dInput = (input - _lastInput);
     _outputSum += (_Ki * error);

     // Add Proportional on Measurement, if P_ON_M is specified
     if (!_pOnE) {
       _outputSum -= _Kp * dInput;
     }

     _outputSum = max(_outMin, min(_outputSum, _outMax));

     // Add Proportional on Error, if P_ON_E is specified
     double output = (_pOnE) ? _Kp * error : 0;

     // Compute Rest of PID Output
     output += _outputSum - _Kd * dInput;
     output = max(_outMin, min(output, _outMax));

     *_outputPtr = output;

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
void PID::setTunings(double Kp, double Ki, double Kd, int pOn) {

  if (Kp < 0 || Ki < 0 || Kd < 0) {
    return;
  }

  _pOn = pOn;
  _pOnE = (pOn == P_ON_E);

  _dispKp = Kp;
  _dispKi = Ki;
  _dispKd = Kd;

  double sampleTimeInSec = ((double)_sampleTime)/1000;
  _Kp = Kp;
  _Ki = Ki * sampleTimeInSec;
  _Kd = Kd / sampleTimeInSec;

  if (_controllerDirection == REVERSE) {
    _Kp = (0 - _Kp);
    _Ki = (0 - _Ki);
    _Kd = (0 - _Kd);
  }
}

/* Set Tunings using the current POn setting
 */
void PID::setTunings(double Kp, double Ki, double Kd){
  setTunings(Kp, Ki, Kd, _pOn); 
}

/* Sets the period, in milliseconds, at which the calculation is performed
 */
void PID::setSampleTime(unsigned long sampleTime)
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
void PID::setOutputLimits(double outputMin, double outputMax)
{
  if (outputMin >= outputMax) {
    return;
  }
  _outMin = outputMin;
  _outMax = outputMax;

  if (_inAuto) {
    *_outputPtr = max(_outMin, min(*_outputPtr, _outMax));
    _outputSum = max(_outMin, min(_outputSum, _outMax));
  }
}

/* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 */
void PID::setMode(int mode) {
  bool newAuto = (mode == AUTOMATIC);
    
  // If we just went from manual to auto
  if (newAuto && !_inAuto){  
    initialize();
  }
  _inAuto = newAuto;
}

/* Does all the things that need to happen to ensure a bumpless transfer
 * from manual to automatic mode.
 */
void PID::initialize()
{
  _outputSum = *_outputPtr;
  _lastInput = *_inputPtr;
  
  _outputSum = max(_outMin, min(_outputSum, _outMax));
}

/* The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 */
void PID::setControllerDirection(int direction)
{
   if (_inAuto && direction != _controllerDirection) {
    _Kp = (0 - _Kp);
    _Ki = (0 - _Ki);
    _Kd = (0 - _Kd);
   }
   _controllerDirection = direction;
}

/* Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 */
double PID::getKp(){ return  _dispKp; }
double PID::getKi(){ return  _dispKi;}
double PID::getKd(){ return  _dispKd;}
int PID::getMode(){ return  _inAuto ? AUTOMATIC : MANUAL;}
int PID::getDirection(){ return _controllerDirection;}

