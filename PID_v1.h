/******************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License.
 *
 * Reformatted and modernized by Mark Womack (https://github.com/markwomack).
 * Functionality is the same, but the method names have changed slightly.
 *****************************************************************************/

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

class PID {
  public:

    // Constants used in the functions below
    #define AUTOMATIC 1
    #define MANUAL 0
    #define DIRECT 0
    #define REVERSE 1
    #define P_ON_M 0
    #define P_ON_E 1

    // Links the PID to the Input, Output, and Setpoint.
    // Initial tuning parameters are also set here.
    // (overload for specifying proportional mode)
    PID(double* inputPtr, double* outputPtr, double* setpointPtr,
        double Kp, double Ki, double Kd, int pOn, int controllerDirection);                                          

    // Links the PID to the Input, Output, and Setpoint.
    // Initial tuning parameters are also set here.
    PID(double* inputPtr, double* outputPtr, double* setpointPtr,
        double Kp, double Ki, double Kd, int controllerDirection);
	
    // Sets PID to either Manual (0) or Auto (non-0)
    void setMode(int mode);

    // Performs the PID calculation. It should be
    // called every time loop() cycles. ON/OFF and
    // calculation frequency can be set using SetMode
    // SetSampleTime respectively
    bool compute();

    // Clamps the output to a specific range. 0-255 by default, but
    // it's likely the user will want to change this depending on
    // the application
    void setOutputLimits(double outputMin, double outputMax);

    // While most users will set the tunings once in the 
    // constructor, this function gives the user the option
    // of changing tunings during runtime for Adaptive control
    // overload for specifying proportional mode.
    void setTunings(double Kp, double Ki, double Kd);                                  
    void setTunings(double Kp, double Ki, double Kd, int pOn);         	  

    // Sets the Direction, or "Action" of the controller. DIRECT
    // means the output will increase when error is positive. REVERSE
    // means the opposite.  it's very unlikely that this will be needed
    // once it is set in the constructor.
    void setControllerDirection(int direction);
    
    // Sets the frequency, in milliseconds, with which 
    // the PID calculation is performed. Default is 100.
    void setSampleTime(unsigned long sampleTime);
										  
    double getKp();
    double getKi();
    double getKd();
    int getMode();
    int getDirection();

  private:
    void initialize();

    // The tuning parameters in user-entered format for display purposes.
    double _dispKp;
    double _dispKi;
    double _dispKd;
    
    double _Kp;  // * (P)roportional Tuning Parameter
    double _Ki;  // * (I)ntegral Tuning Parameter
    double _Kd;  // * (D)erivative Tuning Parameter

    int _controllerDirection;
    int _pOn;

    // Pointers to the Input, Output, and Setpoint variables.
    // This creates a hard link between the variables and the 
    // PID, freeing the user from having to constantly tell us
    // what these values are.  with pointers we'll just know.
    double* _inputPtr;
    double* _outputPtr;
    double* _setpointPtr;
                                  	  
    unsigned long _lastTime;
    double _outputSum;
    double _lastInput;

    unsigned long _sampleTime;
    double _outMin;
    double _outMax;
    bool _inAuto;
    bool _pOnE;
};

#endif // PID_v1_h

