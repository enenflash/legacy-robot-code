#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#pragma once

#include <iostream>
#include <Arduino.h>

class Motor {
    public:
    int PWM_PIN;
    int DIR_PIN;
    Motor(int pwm_pin=0, int dir_pin=0);
    /* speed => -100->100 */
    void run(float speed);
    void stop();
};

#endif // MOTOR