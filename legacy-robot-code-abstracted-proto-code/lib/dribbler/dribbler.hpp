#ifndef _DRIBBLER_HPP_
#define _DRIBBLER_HPP_

#pragma once

#include <iostream>
#include <Arduino.h>
#include "pins.h"

class DribblerMotor {
    public:
    int DIR_PIN, PWM_PIN;
    DribblerMotor(int dir_pin, int pwm_pin);
    void run(float speed=100);
    void stop();
};

#endif