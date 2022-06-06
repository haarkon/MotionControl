#include "mbed.h"
#include "motionControl.h"

motionControl::motionControl (TIM_TypeDef *LTIM, TIM_TypeDef *RTIM, PinName LPWM, PinName RPWM, PinName LDIR, PinName RDIR)
{
    _lEncoder = new Nucleo_Encoder_16_bits (LTIM);
    _rEncoder = new Nucleo_Encoder_16_bits (RTIM);

    _lMotor = new PwmOut (LPWM);
    _rMotor = new PwmOut (RPWM);

    _lDirection = new DigitalOut (LDIR,0);
    _rDirection = new DigitalOut (RDIR,0);

    _lMotor->period_us(50);
    _lMotor->write(0.0f);
    _rMotor->period_us(50);
    _rMotor->write(0.0f);

    _motionControlThread.start(callback(this,&motionControl::motionControlFct));

    _tick.attach(callback (this,&motionControl::getEncoderPosition), 1ms);

    _lPosition = _lEncoder->GetCounter();
    _rPosition = _rEncoder->GetCounter();
}

void motionControl::getEncoderPosition (void) {
    _newLValue = _lEncoder->GetCounter();
    _newRValue = _rEncoder->GetCounter();
    _myEventFlags.set(1);
}

void motionControl::motionControlFct (void) {
    
    float lPwm, rPwm, lSetPoint, rSetPoint, lSpeedError, rSpeedError, lDerivativeSpeedError, rDerivativeSpeedError;
    float lDist, rDist;
    float alpha, curvature, deltaX, deltaY;
    long  lDistTick, rDistTick;

    static float lIntegralSpeedError, rIntegralSpeedError, oldLSpeed, oldRSpeed;

    while (true) {
        _myEventFlags.wait_any(1);

        // Delta on each wheels in ticks (2000 ticks per turn)
        lDistTick = _newLValue - _lPosition;
        rDistTick = _newRValue - _rPosition;
        
        // Saving new wheels position
        _lPosition = _newLValue;
        _rPosition = _newRValue;

        // Calculating distances in meters
        lDist = (float)lDistTick / STEPDISTANCE;
        rDist = (float)rDistTick / STEPDISTANCE;

        // Calculating speeds in meters/seconds
        _lSpeed = lDist * 1000.0f;
        _rSpeed = rDist * 1000.0f;

        // computing speed error (ie : asked speed - real speed)
        lSpeedError = _lSpeedValue - _lSpeed;
        rSpeedError = _rSpeedValue - _rSpeed;

        // computing integral speed error (ie : sum of all speed errors)
        lIntegralSpeedError += lSpeedError;
        rIntegralSpeedError += rSpeedError;

        // Limitating integral speed error (to avoid overspeed after contact)
        if (lIntegralSpeedError > 10.0f) lIntegralSpeedError = 10.0f;
        if (lIntegralSpeedError < -10.0f) lIntegralSpeedError = -10.0f;
        if (lIntegralSpeedError > 10.0f) rIntegralSpeedError = 10.0f;
        if (lIntegralSpeedError < -10.0f) rIntegralSpeedError = -10.0f;

        // Computing derivative speed error (ie : difference of errors) 
        lDerivativeSpeedError = _lSpeed - oldLSpeed;
        rDerivativeSpeedError = _rSpeed - oldRSpeed;
        oldLSpeed = _lSpeed;
        oldRSpeed = _rSpeed;

        // Computing speed set point (ie : the speed I need to go to match the required value - Kp, Ki and Kd must be tuned using numeric Ziegler-Nicholls)
        lSetPoint = (_Kp * lSpeedError) + (_Ki * lIntegralSpeedError) + (_Kd * lDerivativeSpeedError);
        rSetPoint = (_Kp * rSpeedError) + (_Ki * rIntegralSpeedError) + (_Kd * rDerivativeSpeedError);

        // Modify as PWM 
        lPwm = lSetPoint / 1.3f;
        rPwm = rSetPoint / 1.3f;

        // Limitating PWM to gross enveloppe
        if (lPwm > 1.0f) lPwm = 1.0f;
        if (rPwm > 1.0f) rPwm = 1.0f;
        if (lPwm < -1.0f) lPwm = -1.0f;
        if (rPwm < -1.0f) rPwm = -1.0f;

        // Fixing direction
        if (lPwm < 0) {
            lPwm = -lPwm;
            _lDirection->write(1);
        } else {
            _lDirection->write(0);
        }

        if (rPwm < 0) {
            rPwm = -rPwm;
            _rDirection->write(1);
        } else {
            _rDirection->write(0);
        }

        // Applying orders
        _lMotor->write(lPwm);
        _rMotor->write(rPwm);

        // Odometry
        if (lDistTick != rDistTick) {
        // If in a curve (ie : distances on each wheels are not the same)
            // Computing the curvature and the angle done by the gravity center of the robot
            curvature = halfCourse * (rDist + lDist) / (rDist - lDist);
            alpha = (lDist + rDist) / (2.0f * curvature);

            // Computing displacement in cartesian coordinates
            deltaX = curvature * cos (alpha + _tPosition);
            deltaY = curvature * sin (alpha + _tPosition);

        } else {
        // If in straight line (ie : same distance on each wheels)
            // Computing displacement in cartesian coordinates 
            deltaX = lDist * cos (_tPosition);
            deltaY = lDist * sin (_tPosition);
            alpha = 0;
        }
        
        // computing the robot's gravity center new coordinates
        _tPosition += alpha;
        _xPosition += deltaX;
        _yPosition += deltaY; 
    }
}

void motionControl::setKp (float Kp){
    _Kp = Kp;
}

void motionControl::setKi (float Ki){
    _Ki = Ki;
}

void motionControl::setKd (float Kd){
    _Kd = Kd;
}

float motionControl::getKp (void){
    return _Kp;
}

float motionControl::getKi (void){
    return _Ki;
}

float motionControl::getKd (void){
    return _Kd;
}

void motionControl::setSpeed (float lSpeed, float rSpeed){
    _lSpeedValue = lSpeed;
    _rSpeedValue = rSpeed;
}

void motionControl::stop (void){
    _lSpeedValue = 0.0f;
    _rSpeedValue = 0.0f;
}

void motionControl::getSpeed (float *lSpeed, float *rSpeed){
    *lSpeed = _lSpeed;
    *rSpeed = _rSpeed;
}

void motionControl::resetPosition (void) {
    _xPosition = 0.0f;
    _yPosition = 0.0f;
    _tPosition = 0.0f;
}

void motionControl::setPosition (float X, float Y, float Theta) {
    _xPosition = X;
    _yPosition = Y;
    _tPosition = Theta;
}

void motionControl::getPosition (float *X, float *Y, float *Theta){
    *X = _xPosition;
    *Y = _yPosition;
    *Theta = _tPosition;
}

void motionControl::setXPosition (float X){
    _xPosition = X;
}

float motionControl::getXPosition (void){
    return _xPosition;
} 

void motionControl::setYPosition (float Y){
    _yPosition = Y;
}

float motionControl::getYPosition (void){
    return _yPosition;
}  

void motionControl::setTPosition (float Theta){
    _tPosition = Theta;
}

float motionControl::getTPosition (void){
    return _tPosition;
} 

void motionControl::setParameters (float Kp, float Ki, float Kd){
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void motionControl::getParameters (float *Kp, float *Ki, float *Kd){
    *Kp = _Kp;
    *Ki = _Ki;
    *Kd = _Kd;
}
