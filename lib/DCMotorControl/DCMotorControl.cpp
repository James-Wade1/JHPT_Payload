#include "DCMotorControl.h"
// MY EDITED FILE !!!
// TODO:
// TODOs will be listed within the code to improve clarity. Search document for TODO
// TODO: Dont integrate error while in DeadbandDutyCycle. no just remove deadband duty cycle

DCMotorControl::DCMotorControl(uint8_t EnablePin, uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin)  // the enable pin seems to never be used?
{
    _DrivePin = DrivePin;
    _EnablePin = EnablePin;
    _DirectionPin = DirectionPin;

    // Direction and PWM output
    pinMode(_EnablePin, OUTPUT);
    pinMode(_DirectionPin, OUTPUT);
    pinMode(_DrivePin, OUTPUT);

    // Configure PWM to match hardware capabilities
    // analogWriteFrequency(_DrivePin, PWMFrequency);
    // analogWriteResolution(PWMBits);

    // set default parameters
    _ControlRate_us = 10000;
    _DeadbandTicks = 10;
    _TicksPerInch = ((50 * 64) / (3.14159265359 * 0.713));
    _TicksPerRevolution = (50 * 64);
    _DeadbandDutyCycle = 0;
    _DeadbandTicks = 0;
    setControllerGains(1, 0, 0);
    _MinimumPWM = 0;
    _CurrentState = DC_Manual;
    _DutyCycleStall = 101.0;
    _MaxDutyCycleDelta = 201.0;
    // Connect encoder pins to quadrature encoder library
    _Ticks = new Encoder(Encoder1Pin, Encoder2Pin);
}
// DCMotorControl::DCMotorControl( uint8_t DirectionPinA, uint8_t DirectionPinB, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //This is now legacy code, if this is used it will be incorrect.
// {
// 	// if we do not want to use an enable pin set that value to 255
// 	_DrivePin = DrivePin;
// 	_EnablePin = 255;
// 	_DirectionPinA = DirectionPinA;
// 	_DirectionPinB = DirectionPinB;

// 	//Direction and PWM output
// 	pinMode(_EnablePin, OUTPUT);
// 	pinMode(_DirectionPinA, OUTPUT);
// 	pinMode(_DirectionPinB, OUTPUT);
// 	pinMode(_DrivePin, OUTPUT);

// 	//Configure PWM to match hardware capabilities
// 	//analogWriteFrequency(_DrivePin, PWMFrequency);
// 	//analogWriteResolution(PWMBits);

// 	// set default parameters
// 	_ControlRate_us = 10000; //units of microseconds.
// 	_DeadbandTicks = 10;
// 	_TicksPerInch = ((50*64)/(3.14159265359*0.713));
// 	_TicksPerRevolution =  (50*64);
// 	_DeadbandDutyCycle = 5;
// 	_DeadbandTicks = 0;
// 	_InnerDeadbandTicks = 0;
// 	_OuterDeadbandTicks = 0;
// 	setControllerGains(1,0,0);
// 	_MinimumPWM = 10;
// 	_CurrentState = DC_Manual;
//   _DutyCycleStall = 101.0;
//   _MaxDutyCycleDelta = 201.0;
// 	// Connect encoder pins to quadrature encoder library
// 	_Ticks = new Encoder(Encoder1Pin, Encoder2Pin);
// 	//throw an error
// 	// cerr << "This constructor is deprecated. Please use the constructor that takes a single direction pin." << endl;
// }
DCMotorControl::DCMotorControl(uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) {
    // if we do not want to use an enable pin set that value to 255
    _DrivePin = DrivePin;
    _EnablePin = 255;
    _DirectionPin = DirectionPin;

    // Direction and PWM output
    //  pinMode(_EnablePin, OUTPUT);
    pinMode(_DirectionPin, OUTPUT);
    pinMode(_DrivePin, OUTPUT);

    // Configure PWM to match hardware capabilities
    // analogWriteFrequency(_DrivePin, PWMFrequency);
    // analogWriteResolution(PWMBits);

    // set default parameters
    _ControlRate_us = 10000;
    _DeadbandTicks = 10;
    _TicksPerInch = (24.5) * (6678.6 / 43.98);
    _TicksPerRevolution = (6678.624);
    _DeadbandDutyCycle = 5;
    _DeadbandTicks = 30;
    _InnerDeadbandTicks = 30;
    _OuterDeadbandTicks = 30;
    setControllerGains(0.1, 0.003, 0.01);
    _MinimumPWM = 10;
    _CurrentState = DC_Manual;
    _DutyCycleStall = 101.0;
    _MaxDutyCycleDelta = 10;
    // Connect encoder pins to quadrature encoder library
    _Ticks = new Encoder(Encoder1Pin, Encoder2Pin);  // a pointer to an instance of the Encoder class.
}
// This function should be run repeatedly with a period of _ControlRate_us microseconds
bool DCMotorControl::run() {
    // check to see if we are in the manual state. if we are in manual then exit.
    if (DC_Manual == _CurrentState) {
        return false;
    }
    else if (DC_Automatic == _CurrentState) {
        float LastDutyCycle = _DutyCycle;
        // store the last tick count
        _LastTicks = _CurrentPositionTicks;
        // read the new tick count
        _CurrentPositionTicks = _Ticks->read();
        // calculate error
        _ErrorTicks = _DesiredPositionTicks - _CurrentPositionTicks;
        // Check if we are in the deadband measured in ticks
        if (abs(_ErrorTicks) < _DeadbandTicks) {
            _DutyCycle = 0;
            _IntegratedError = 0;  // Reset integrator so we dont jump in the wrong direction when setpoint changes.
            // ToDo: Do I want to set integrated error to zero? Maybe this is only goo for systems that stay put with no control input.
            // Deadband Levels
            _DeadbandTicks = _OuterDeadbandTicks;
        } else {
            _DeadbandTicks = _InnerDeadbandTicks;  // ToDo: More efficient way to set this?
            // Add current error to integrated error variable.
            // The gain Ki is applied here so that if the gains are changed on the fly the contribution
            //     from the integrated error does not jump.
            static float newIntegratedError = 0;
            newIntegratedError = _Ki * _ErrorTicks;
            _IntegratedError += newIntegratedError;
            // Calculate Derivative Error and store current error for next cycle
            //  TODO?: Reduce "derivative kick" by implementing "derivative on measurement"
            _DerivativeError = (_ErrorTicks - _LastError);
            _LastError = _ErrorTicks;
            // Duty cycle equals proportional gain times error plus integrated error times integrator gain
            _DutyCycle = _Kp * _ErrorTicks + _IntegratedError + _Kd * _DerivativeError;

            // Prevent Integrator Windup
            // if the duty cycle is above 100
            if (_DutyCycle > 100) {
                // set duty cycle to 100
                _DutyCycle = 100;
                // subtract current error from integrated error variable to prevent windup
                _IntegratedError -= newIntegratedError;
            }
            // else if duty cycle is below -100
            else if (_DutyCycle < -100.0) {
                // set duty cycle to -100
                _DutyCycle = -100.0;
                // subtract current error from integrated error variable to prevent windup
                _IntegratedError -= newIntegratedError;
            }

            // limit duty cycle change
            if (_DutyCycle - LastDutyCycle > _MaxDutyCycleDelta) {
                _DutyCycle = LastDutyCycle + _MaxDutyCycleDelta;
            } else if (_DutyCycle - LastDutyCycle < -_MaxDutyCycleDelta) {
                _DutyCycle = LastDutyCycle - _MaxDutyCycleDelta;
            }

            // I COMMENTED THIS OUT TO DEBUG AND IT FIXED THE PROBLEM OF GETTING RANDOM ENCODER VALUES.
            // check for stall. Stall if we are up to a high duty cycle but not moving.
            //   if((abs(LastDutyCycle) >= _DutyCycleStall) & (_LastTicks == _CurrentPositionTicks)){
            //     // kick it into manual and set Duty Cycle to zero
            //     _DutyCycle = 0;
            //     //_CurrentState = DC_Manual;
            //   }

            // TODO: Add capability to support "Proportional on Measurement" (feedforward)
        }
    }
    // set duty cycle
    setDutyCycle();
    return true;
}

void DCMotorControl::initializeController() {
    // goal is to avoid huge jerk during the transition
    // We will set the _IntegratedError to the previous duty cycle so that it keeps doing what it was doing
    _IntegratedError = _DutyCycle;
    // TODO?: After implementing "derivative on measurement" set LastInput = Input to reduce derivative spike on initialization
}

void DCMotorControl::setDutyCycle(void) {
    // values less than the deadband should be cast to zero
    // values above the deadband should be mapped between MinPWM and 100
    static double DCdiff = 0.0;
    DCdiff = abs(_DutyCycle) - _DeadbandDutyCycle;
    if (DCdiff <= 0.001) {
        _PWMOutput = 0;
    } else {
        if (_DutyCycle > 0) {
            _PWMOutput = DCdiff * (100.0 - _MinimumPWM) / ((double)(100.0 - _DeadbandDutyCycle)) + _MinimumPWM;
        } else {
            _PWMOutput = -1.0 * DCdiff * (100 - _MinimumPWM) / ((double)(100 - _DeadbandDutyCycle)) - _MinimumPWM;
        }
    }
    // Now that the duty cycle has been mapped write it to the hardware

#ifdef DRIVE_BRAKE
    if (_PWMOutput >= 0) {
        // clear direction pin
        digitalWrite(_DirectionPinA, LOW);
        if (_PWMOutput >= 100) {
            analogWrite(_DrivePin, PWMResolution);
        } else {
            analogWrite(_DrivePin, (_PWMOutput / 100.0) * PWMResolution);
        }
    } else {
        // set direction pin
        digitalWrite(_DirectionPin, HIGH);
        if (_PWMOutput <= -100) {
            analogWrite(_DrivePin, 0);
        } else {
            analogWrite(_DrivePin, PWMResolution - (-_PWMOutput / 100.0) * PWMResolution);
        }
    }
#elif defined SIGN_MAGNITUDE
    if (_PWMOutput >= 0) {  // reduntant code? map between min_pwm and 255.
        // clear direction pin
        digitalWrite(_DirectionPin, HIGH);  // need to set the other direction pin to the opposite value.

        if (_PWMOutput >= 100.0) {
            analogWrite(_DrivePin, PWMResolution);
        } else {
            analogWrite(_DrivePin, (_PWMOutput / 100.0) * PWMResolution);
        }
    } else {
        // set direction pin
        digitalWrite(_DirectionPin, LOW);
        if (_PWMOutput <= -100.0) {
            analogWrite(_DrivePin, PWMResolution);
        } else {
            analogWrite(_DrivePin, (-_PWMOutput / 100.0) * PWMResolution);
        }
    }
#elif defined DRIVE_COAST
    if (_PWMOutput >= 0) {
        // clear direction pin
        digitalWrite(_DirectionPin, LOW);

        digitalWrite(_DrivePin, HIGH);
        if (_PWMOutput >= 100) {
            analogWrite(_EnablePin, PWMResolution);
        } else {
            analogWrite(_EnablePin, (_PWMOutput / 100.0) * PWMResolution);
        }
    } else {
        // set direction pin
        digitalWrite(_DirectionPin, HIGH);
        digitalWrite(_DrivePin, LOW);
        if (_PWMOutput <= -100) {
            analogWrite(_EnablePin, PWMResolution);
        } else {
            analogWrite(_EnablePin, (-_PWMOutput / 100.0) * PWMResolution);
        }
    }
#endif
}

void DCMotorControl::setDutyCycle(float DutyCycle) {
#ifdef DRIVE_BRAKE
    if (DC_Manual == _CurrentState) {
        if (DutyCycle >= 0) {
            // clear direction pin
            digitalWrite(_DirectionPin, LOW);

            if (DutyCycle >= 100) {
                analogWrite(_DrivePin, PWMResolution);
            } else {
                analogWrite(_DrivePin, (DutyCycle / 100.0) * PWMResolution);
            }
        } else {
            // set direction pin
            digitalWrite(_DirectionPin, HIGH);
            if (DutyCycle <= -100) {
                analogWrite(_DrivePin, 0);
            } else {
                analogWrite(_DrivePin, PWMResolution - (-DutyCycle / 100.0) * PWMResolution);
            }
        }
    }
#elif defined SIGN_MAGNITUDE
    if (DC_Manual == _CurrentState) {
        if (DutyCycle >= 0) {
            // clear direction pin
            digitalWrite(_DirectionPin, LOW);

            if (DutyCycle >= 100) {
                analogWrite(_DrivePin, PWMResolution);
            } else {
                analogWrite(_DrivePin, (DutyCycle / 100.0) * PWMResolution);
            }
        } else {
            // set direction pin
            digitalWrite(_DirectionPin, HIGH);
            if (DutyCycle <= -100) {
                analogWrite(_DrivePin, PWMResolution);
            } else {
                analogWrite(_DrivePin, (-DutyCycle / 100.0) * PWMResolution);
            }
        }
    }
#elif defined DRIVE_COAST
    if (_PWMOutput >= 0) {
        // clear direction pin
        digitalWrite(_DirectionPin, LOW);

        digitalWrite(_DrivePin, HIGH);
        if (_PWMOutput >= 100) {
            analogWrite(_EnablePin, PWMResolution);
        } else {
            analogWrite(_EnablePin, (DutyCycle / 100.0) * PWMResolution);
        }
    } else {
        // set direction pin
        digitalWrite(_DirectionPin, HIGH);
        digitalWrite(_DrivePin, LOW);
        if (_PWMOutput <= -100) {
            analogWrite(_EnablePin, PWMResolution);
        } else {
            analogWrite(_EnablePin, (-DutyCycle / 100.0) * PWMResolution);
        }
    }
#endif
}

void DCMotorControl::setControllerGains(float Kp, float Ki, float Kd) {
    //    float SampleTimeInSec = ((float)_ControlRate_us)/1000000;
    _Kp = Kp;
    _Ki = Ki;  //* SampleTimeInSec;
    _Kd = Kd;  // / SampleTimeInSec;
               // Ki and Kd are scaled by the sample time so that changes in the sample time doesnt impact our control gains
}

void DCMotorControl::setControlRate(uint32_t NewControlRate) {
    if (NewControlRate > 0) {
        //   float ratio  = (float)NewControlRate / (float)_ControlRate_us;
        //   // automatically adjust control gains to account for change in control rate
        //   _Ki *= ratio;
        //   _Kd /= ratio;
        _ControlRate_us = NewControlRate;
    }
}

void DCMotorControl::setCurrentPositionTicks(int32_t NewPositionTicks) {
    _Ticks->write(NewPositionTicks);
}

void DCMotorControl::setCurrentPositionInches(float NewPositionInches) {
    _Ticks->write((int32_t)(NewPositionInches * _TicksPerInch));
}

bool DCMotorControl::setMinimumDutyCycle(float DutyCycle) {
    if (0.0 <= DutyCycle && DutyCycle < 100.0) {
        _MinimumPWM = DutyCycle;
        return true;
    } else
        return false;
}

// changes the internal state of the controller to give different behavior.
void DCMotorControl::setMode(DCMotorControlState_t Mode) {
    // TODO?: If we have more state transition events then create new function to handle state transitions.
    // do a controller initialization when we change to automatic
    if ((DC_Automatic == Mode)) {
        if (DC_Automatic != _CurrentState)
            initializeController();
    } else if (DC_Manual == Mode) {
    }
    // change the state
    _CurrentState = Mode;
}
void DCMotorControl::stopMotor() {
    _DutyCycle = 0;
    setDutyCycle();
    _CurrentState = DC_Manual;
}

bool DCMotorControl::disableMotor() {
    if (_EnablePin == 255) {
        setMode(DC_Manual);
        return false;
    } else {
        // digitalWrite( _EnablePin, LOW );
        setMode(DC_Manual);
        return true;
    }
}

bool DCMotorControl::enableMotor() {
    if (_EnablePin == 255) {
        setMode(DC_Manual);
        return false;
    } else {
        // digitalWrite( _EnablePin, HIGH );
        setMode(DC_Manual);
        return true;
    }
}
bool DCMotorControl::setMotorEnable(bool MotorEnabled) {
    if (_EnablePin == 255) {
        setMode(DC_Manual);
        return false;
    } else {
        setMode(DC_Manual);
        // digitalWrite( _EnablePin, MotorEnabled );
        return true;
    }
}

void DCMotorControl::addToDesiredPositionTicks(int32_t TicksToAdd) {
    _DesiredPositionTicks = _DesiredPositionTicks + TicksToAdd;
}

void DCMotorControl::addToDesiredPositionInches(float InchesToAdd) {
    _DesiredPositionTicks = _DesiredPositionTicks + (InchesToAdd * _TicksPerInch);
}

void DCMotorControl::setDesiredPositionTicks(int32_t NewPositionTicks) {
    _DesiredPositionTicks = NewPositionTicks;
}

void DCMotorControl::setDesiredPositionInches(float NewPositionInches) {
    _DesiredPositionTicks = NewPositionInches * _TicksPerInch;
}

void DCMotorControl::setDeadbandTicks(int16_t DeadbandTicks) {
    _DeadbandTicks = DeadbandTicks;
    _InnerDeadbandTicks = DeadbandTicks;
    _OuterDeadbandTicks = DeadbandTicks;
}
void DCMotorControl::setDeadbandTicks(int16_t InnerDeadbandTicks, int16_t OuterDeadbandTicks) {
    if (InnerDeadbandTicks < OuterDeadbandTicks) {
        _DeadbandTicks = InnerDeadbandTicks;
        _InnerDeadbandTicks = InnerDeadbandTicks;
        _OuterDeadbandTicks = OuterDeadbandTicks;
    } else {
        _DeadbandTicks = OuterDeadbandTicks;
        _InnerDeadbandTicks = OuterDeadbandTicks;
        _OuterDeadbandTicks = InnerDeadbandTicks;
    }
}

void DCMotorControl::setDeadbandDutyCycle(float DeadbandDutyCycle) {
    _DeadbandDutyCycle = DeadbandDutyCycle;
}

void DCMotorControl::setTicksPerInch(float TicksPerInch) {
    _TicksPerInch = TicksPerInch;
}

void DCMotorControl::setTicksPerRevolution(float TicksPerRevolution) {
    _TicksPerRevolution = TicksPerRevolution;
}

void DCMotorControl::setMaxDutyCycleDelta(float DutyCycle) {
    _MaxDutyCycleDelta = abs(DutyCycle);
}

void DCMotorControl::setDutyCycleStall(float DutyCycle) {
    _DutyCycleStall = abs(DutyCycle);
}

void DCMotorControl::setParameters(float Kp, float Ki, float Kd, int ControlRate_us, int DeadbandTicks, float DeadbandDutyCycle, float TicksPerInch, float TicksPerRevolution, float MinimumDutyCycle) {
    setControlRate(ControlRate_us);
    setDeadbandTicks(DeadbandTicks);
    setDeadbandDutyCycle(DeadbandDutyCycle);
    setTicksPerInch(TicksPerInch);
    setTicksPerRevolution(TicksPerRevolution);
    setMinimumDutyCycle(MinimumDutyCycle);
    setControllerGains(Kp, Ki, Kd);
}

int32_t DCMotorControl::getDesiredPositionTicks() { return _DesiredPositionTicks; }
float DCMotorControl::getDesiredPositionInches() { return _DesiredPositionTicks / _TicksPerInch; }
int32_t DCMotorControl::getCurrentPositionTicks() { return _Ticks->read(); }
float DCMotorControl::getCurrentPositionInches() { return _Ticks->read() / _TicksPerInch; }
float DCMotorControl::getKp() { return _Kp; }
float DCMotorControl::getKi() { return _Ki; }
float DCMotorControl::getKd() { return _Kd; }
DCMotorControlState_t DCMotorControl::getMode() { return _CurrentState; }
float DCMotorControl::getDutyCycle() { return _DutyCycle; }
float DCMotorControl::getPWMOutput() { return _PWMOutput; }
float DCMotorControl::getCurrentRPM() { return ((_CurrentPositionTicks - _LastTicks) / (float)_TicksPerRevolution) / (float)_ControlRate_us * 60.0e6; }
uint32_t DCMotorControl::getControlRate() { return _ControlRate_us; }
int16_t DCMotorControl::getDeadbandTicks() { return _DeadbandTicks; }
int16_t DCMotorControl::getDeadbandDutyCycle() { return _DeadbandDutyCycle; }
float DCMotorControl::getTicksPerInch() { return _TicksPerInch; }
float DCMotorControl::getTicksPerRevolution() { return _TicksPerRevolution; }
float DCMotorControl::getMinimumPWM() { return _MinimumPWM; }
float DCMotorControl::getMaxDutyCycleDelta() { return _MaxDutyCycleDelta; }
float DCMotorControl::getDutyCycleStall() { return _DutyCycleStall; }
