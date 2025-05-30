#ifndef _MOTOR_CONTROLLER_HPP_
#define _MOTOR_CONTROLLER_HPP_

#pragma once

#include <iostream>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "pins.h"
#include "vector.hpp"
#include "motor.hpp"

class MotorController {
    public:
    Motor TL;
    Motor TR;
    Motor BL;
    Motor BR;

    float rotation_const;

    MotorController(float rotation_constant=1);

    private:
    std::array<float, 4> get_motor_speeds(float angle, float speed, float rotation);

    public:
    void run_motors(float speed, float angle, float rotation);
    void run_raw(float tl_raw, float tr_raw, float bl_raw, float br_raw);
    void stop_motors();
};

#endif // MOTOR CONTROLLER