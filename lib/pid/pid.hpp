#ifndef _PID_HPP_
#define _PID_HPP_

#pragma once

#include "vector.hpp"


class PID {
public:
    // Variable Initilisations
    const float PROPORTIONAL_CONSTANT = 2.0;
    const float INTEGRAL_CONSTANT = 0.0005;
    const float DERIVETIVE_CONSTANT = 0.1;
    float MINIMUM_PID_SPEED = 30;
    double previousError, integral, derivitive;
    float angle, speed;


    // Calculating Speed
   

    Vector moveTo(float target_x, float target_y, float x, float y, float max_speed, double dt);

private:
    double compute(double error, double dt);
    float min(float var1, float var2);
    float max(float var1, float var2);
};





#endif