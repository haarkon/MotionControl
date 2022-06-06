/**
 * @author Hugues Angelis
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * Motion Control System - using Both IFX9201S (Full bridge - Infineon) and AS5047 (14 bits magnetic angular position sensor - Austria Semiconductors) this system
 * allows the control of the motion of the robot.
 *
 * Datasheets : IFX9201S : https://www.infineon.com/cms/en/product/power/motor-control-ics/brushed-dc-motor-driver-ics/integrated-full-bridge-driver/ifx9201sg/
 *              AS5047P  : https://ams.com/en/as5047p
 */

#ifndef _MC_H_
#define _MC_H_ 
 
/** Includes */
#include "mbed.h"
#include "math.h"
#include "Nucleo_Encoder_16_bits.h"

/** Defines */
#define STEPDISTANCE      0.00015708f
#define halfCourse        0.0115f                

class motionControl {
public :

/**
 * \class motionControl : motionControl.h
 * MotionControl : non blocking function allowing to drive a robot with efficiency using IFX9201S (full bridge) and AS5047P (14-bit on axis magnetic rotary position sensor) 
 * \brief More informations at https://www.bosch-sensortec.com/products/motion-sensors/magnetometers-bmm150/
 * \note We use a mikroBus Geomagnetic Click board as a compass. The card data are available at https://www.mikroe.com/geomagnetic-click
 * \note As the Dataready pin of the BMM150 is left unconnected on the mikroBus card, we use a preset that allows BMM150 to perform continuously the measurment of each axis
 *
 * \code
 * #include "mbed.h"
 * #include "motionControl.h"
 * 
 * 
 * DigitalOut ledV (LED1, 0);
 * DigitalOut ledR (LED2, 0);
 * DigitalOut ledB (LED3, 0);
 * 
 * InterruptIn jack (PC_10);
 * 
 * motionControl robot (TIM3, TIM4, PE_6, PE_5, PE_3, PE_10);
 * 
 * int main()
 * {
 *     float lSpeed, rSpeed;
 *     float Kp=1, Ki=0, Kd=0;
 *     float X = 0.0f, Y = 0.0f, T = 0.0f;
 * 
 *     robot.stop();
 *     robot.resetPosition();
 *     robot.setParameters (Kp, Ki, Kd);
 * 
 *     while (true) {
 *         ledR = !ledR;
 *
 *         robot.setSpeed(0.5, 0.5);
 * 
 *         do {
 *             robot.getPosition (&X, &Y, &T);
 *         } while (X < 1.0f);
 * 
 *         robot.stop();
 * 
 *         do {
 *             robot.getSpeed (&lSpeed, &rSpeed);
 *         } while ((lSpeed > 0) && (rSpeed > 0));
 * 
 *         robot.setSpeed(0.5, -0.5);
 * 
 *         do {
 *             robot.getPosition (&X, &Y, &T);
 *         } while (X > 0.0f);
 * 
 *         robot.stop();
 * 
 *         ThisThread::sleep_for(5s);
 *     }
 * }
 * \endcode
 */

    // Constructor
    motionControl (TIM_TypeDef *LTIM, TIM_TypeDef *RTIM, PinName LPWM, PinName RPWM, PinName LDIR, PinName RDIR);

    void setSpeed (float lSpeed, float rSpeed);
    void stop (void);
    void getSpeed (float *lSpeed, float *rSpeed);

    void resetPosition (void);
    void setPosition (float X, float Y, float Theta);
    void getPosition (float *X, float *Y, float *Theta);

    void setXPosition (float X);
    float getXPosition (void); 

    void setYPosition (float Y);
    float getYPosition (void); 

    void setTPosition (float Theta);
    float getTPosition (void); 

    void setParameters (float Kp, float Ki, float Kd);
    void getParameters (float *Kp, float *Ki, float *Kd);

    void setKp (float Kp);
    float getKp (void);

    void setKi (float Ki);
    float getKi (void);
    
    void setKd (float Kd);
    float getKd (void);
    
private :
    void getEncoderPosition (void);
    void motionControlFct (void);

    Nucleo_Encoder_16_bits *_lEncoder;
    Nucleo_Encoder_16_bits *_rEncoder;

    DigitalOut *_lDirection;
    DigitalOut *_rDirection;

    PwmOut *_lMotor;
    PwmOut *_rMotor;

    Ticker _tick;

    Thread _motionControlThread;

    EventFlags _myEventFlags;

    long _lPosition, _rPosition;
    long _newLValue, _newRValue;

    float _lSpeed, _rSpeed, _lSpeedValue, _rSpeedValue;
    float _Kp, _Ki, _Kd;
    float _xPosition, _yPosition, _tPosition;
};

#endif