/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "motionControl.h"


DigitalOut ledV (LED1, 0);
DigitalOut ledR (LED2, 0);
DigitalOut ledB (LED3, 0);

InterruptIn jack (PC_10);

motionControl robot (TIM3, TIM4, PE_6, PE_5, PE_3, PE_10);

int main()
{
    float lSpeed, rSpeed;
    float Kp=1, Ki=0, Kd=0;
    float X = 0.0f, Y = 0.0f, T = 0.0f;

    robot.stop();
    robot.resetPosition();
    robot.setParameters (Kp, Ki, Kd);

    while (true) {
        ledR = !ledR;
    
        robot.setSpeed(0.5, 0.5);

        do {
            robot.getPosition (&X, &Y, &T);
        } while (X < 1.0f);

        robot.stop();

        do {
            robot.getSpeed (&lSpeed, &rSpeed);
        } while ((lSpeed > 0) && (rSpeed > 0));

        robot.setSpeed(-0.5, -0.5);

        do {
            robot.getPosition (&X, &Y, &T);
        } while (X > 0.0f);

        robot.stop();

        ThisThread::sleep_for(5s);
    }
}
