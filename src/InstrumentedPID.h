//
// This Library is licensed under the MIT License, see the LICENSE file for
// more details.
//
// The original version of this code was released by Brett Beauregard under
// the MIT license. It can be found at:
//
//   https://github.com/br3ttb/Arduino-PID-Library
//

#ifndef INSTRUMENTED_PID_H
#define INSTRUMENTED_PID_H

enum Direction {
  DIRECT = 0,
  REVERSE = 1
};

enum PIDMode {
  P_ON_E,  // Proportional on error (typical and default)
  P_ON_M   // Proportional on measurement
};
    
class InstrumentedPID {
  public:

    // Constructor with initial tuning parameters, pidMode, and direction.
    InstrumentedPID(double Kp, double Ki, double Kd, PIDMode pidMode, Direction direction);                                          

    // Constructor with initial tuning parameters and direction.
    InstrumentedPID(double Kp, double Ki, double Kd, Direction direction);
    
    // Starts the PID so it can start computing.
    void start(double initialInput, double initialOutput);
    
    // Stops the PID, calls to compute() will do nothing.
    void stop();

    // Performs the PID calculation. It should be
    // called every time loop() cycles. ON/OFF and
    // calculation frequency can be set using SetMode
    // SetSampleTime respectively
    bool compute(double input, double setpoint, double* returnOutput);

    // Clamps the output to a specific range. 0-255 by default, but
    // it's likely the user will want to change this depending on
    // the application
    void setOutputLimits(double outputMin, double outputMax);

    // While most users will set the tunings once in the 
    // constructor, this function gives the user the option
    // of changing tunings during runtime for Adaptive control
    // overload for specifying proportional mode.
    void setTunings(double Kp, double Ki, double Kd);                                  
    void setTunings(double Kp, double Ki, double Kd, PIDMode pidMode);         	  

    // Sets the Direction, or "Action" of the controller. DIRECT
    // means the output will increase when error is positive. REVERSE
    // means the opposite.  it's very unlikely that this will be needed
    // once it is set in the constructor.
    void setDirection(Direction direction);
    
    // Sets the frequency, in milliseconds, with which 
    // the PID calculation is performed. Default is 100.
    void setSampleTime(unsigned long sampleTime);
										  
    double getKp();
    double getKi();
    double getKd();
    bool isRunning();
    PIDMode getPIDMode();
    Direction getDirection();

  private:

    // The tuning parameters in user-entered format for display purposes.
    double _dispKp;
    double _dispKi;
    double _dispKd;
    
    double _Kp;  // * (P)roportional Tuning Parameter
    double _Ki;  // * (I)ntegral Tuning Parameter
    double _Kd;  // * (D)erivative Tuning Parameter

    bool _isRunning;    
    Direction _direction;
    PIDMode _pidMode;   
    unsigned long _sampleTime;
    double _outMin;
    double _outMax;      	  
    double _outputSum;

    // Instrumentation    	  
    unsigned long _lastTime;
    double _lastInput;
    double _lastKiValue;
    double _lastKpValue;
    double _lastKdValue;
    double _lastError;
    double _lastDiffInput;
    double _lastSetpoint;
    double _lastOutputSum;
};

#endif // PID_v1_h

