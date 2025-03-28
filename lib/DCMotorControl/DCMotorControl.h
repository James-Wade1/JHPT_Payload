/**********************************************************************************************
 * Teensy DC Motor Control Library
 * written by Zachary Hammond <zack.hammond@intusurg.com> , <hammondzm@gmail.com>
 * This library provides PID control of a DC motor with an H-bridge driver in drive-brake mode and an incremental quadrature encoder for position feedback.
 **********************************************************************************************/

/* Encoder Library, for measuring quadrature encoder signals
    http://www.pjrc.com/teensy/td_libs_Encoder.html
    teensy has 2 hardware quadrature encoder interfaces. This is less than we need and requires mess code so we will just use the Encoder Library  */

// Edits made by Chris Paul as part of the Soft Robotics Usevitch Project from 2023-2024
#ifndef DCMOTORCONTROL_H
#define DCMOTORCONTROL_H

#include <Encoder.h>

// Hardware Settings
// For the teensy, more information on PWM can be found here https://www.pjrc.com/teensy/td_pulse.html
#define PWMBits 10
#define PWMResolution 255  //((2^PWMBits)-1) // add the largest number that can be stored. 8bit = 255, 10 bit = 1023 ...
#define PWMFrequency 58593.75

// Drive Mode
// #define DRIVE_BRAKE
// #define DRIVE_COAST
#define SIGN_MAGNITUDE
// #define LOCKED_ANITPHASE

#if defined(DRIVE_BRAKE) || defined(DRIVE_COAST) || defined(SIGN_MAGNITUDE) || defined(LOCKED_ANITPHASE)
// Drive mode specified
#else
#error Drive mode not specified
#endif

typedef enum { DC_Automatic,
               DC_Manual } DCMotorControlState_t;  // different states the controller can be put in.

class DCMotorControl {
   public:
    // DCMotorControl(uint8_t EnablePin, uint8_t DirectionPinA, uint8_t DirectionPinB, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin);		// Constructor
    DCMotorControl(uint8_t DirectionPin, uint8_t DirectionPinB, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin);  // Constructor used when Enable pin is not used. (EnablePin set to 255)
    DCMotorControl(uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin);         // added back in so that I could use the new DRV8874 Single Brushed DC Motor Driver Carrier
    // This motor controller recommends using only one direction pin and one PWM pin.
    bool run();                                              // runs the PID computation. This should be called periodically at a rate equal to _ControlRate_us (in microseconds)
    void initializeController();                             // resets the PID to prevent strange results when switching control modes.
    void setDutyCycle(float DutyCycle);                      // changes the PWM duty cycle manually
    void setControllerGains(float Kp, float Ki, float Kd);   // Sets the PID gains
    void setControlRate(uint32_t NewControlRate);             // set the member variable that stores the control rate. This should match the rate that run is called.
    void setCurrentPositionTicks(int32_t NewPositionTicks);      // set the member variable that stores the current position with input measured in encoder counts
    void setCurrentPositionInches(float NewPositionInches);  // set the member variable that stores the current position with input measured in inches (uses member variable _TicksPerInch)
    bool setMinimumDutyCycle(float MinimumDutyCycle);        // A motor will require a minimum PWM duty cycle to move the motor. The PID wont output a dutycycle lower than MinimumDutyCycle
    void setMode(DCMotorControlState_t);
    void stopMotor();
    bool disableMotor();
    bool enableMotor();
    bool setMotorEnable(bool MotorEnabled);
    void addToDesiredPositionTicks(int32_t TicksToAdd);
    void addToDesiredPositionInches(float InchesToAdd);
    void setDesiredPositionTicks(int32_t NewPositionTicks);
    void setDesiredPositionInches(float NewPositionInches);
    void setDeadbandTicks(int16_t DeadbandTicks);                               // If the current position is within the deadband of the desired position, the motors will not move.
    void setDeadbandTicks(int16_t InnerDeadbandTicks, int16_t OuterDeadbandTicks);  // two deadband layers. the motors stop when inside the inner layer and dont start gain until outside the outer layer.
    void setDeadbandDutyCycle(float DeadbandDutyCycle);                     // If the duty cycle magnitude is less than the duty cycle deadband the output duty cycle will be zero
    void setTicksPerInch(float TicksPerInch);                               // If there is a relationship between linear distance and encoder counts set that relationship here
    void setTicksPerRevolution(float TicksPerRevolution);                   // Enter the number of encoder counts per revolution
    void setParameters(float, float, float, int, int, float, float, float, float);
    void setMaxDutyCycleDelta(float DutyCycle);
    void setDutyCycleStall(float DutyCycle);

    int32_t getDesiredPositionTicks();
    float getDesiredPositionInches();
    int32_t getCurrentPositionTicks();
    float getCurrentPositionInches();
    float getKp();
    float getKi();
    float getKd();
    DCMotorControlState_t getMode();
    float getDutyCycle();   // returns the duty cycle as computed by the PID controller
    float getPWMOutput();   // returns the PWM duty cycle that is output. The computed duty cycle is coerced to a range defined by deadbands and minimum duty cycle.
    float getCurrentRPM();  // This function is only valid while in the Automatic mode. Otherwise the user must read ticks themselves.
    uint32_t getControlRate();
    int16_t getDeadbandTicks();
    int16_t getDeadbandDutyCycle();
    float getTicksPerInch();
    float getTicksPerRevolution();
    float getMinimumPWM();
    float getMaxDutyCycleDelta();
    float getDutyCycleStall();

   private:
    uint8_t _DrivePin;       // This pin will be PWM'ed to control average applied voltage to the motor
    uint8_t _EnablePin;      // This pin enables and disables the motor driver. If not needed set as 255.
    uint8_t _DirectionPinA;  // First pin that controls the direction of the motor
    uint8_t _DirectionPinB;  // Second pin that controls the direction of the motor
    uint8_t _DirectionPin;   // Single pin that controls the direction of the motor
    double _DutyCycle;
    double _PWMOutput;
    int32_t _DesiredPositionTicks;
    int32_t _CurrentPositionTicks;
    int32_t _LastTicks;
    int32_t _ErrorTicks;
    float _Kp;
    float _Ki;
    float _Kd;
    double _IntegratedError;
    double _DerivativeError;
    double _CurrentRPM;
    double _LastError;
    uint32_t _ControlRate_us;
    int16_t _DeadbandTicks;
    int16_t _InnerDeadbandTicks;
    int16_t _OuterDeadbandTicks;
    double _DeadbandDutyCycle;
    double _TicksPerInch;
    double _TicksPerRevolution;
    double _MinimumPWM;  // The smallest PWM that can move the motor
    double _MaxDutyCycleDelta;
    double _DutyCycleStall;

    Encoder* _Ticks;
    DCMotorControlState_t _CurrentState;

    void setDutyCycle(void);  // sets the duty cycle to the member variable _DutyCycle
};

#endif