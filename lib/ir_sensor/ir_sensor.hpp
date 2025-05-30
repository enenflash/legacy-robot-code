#ifndef _IR_SENSOR_HPP_
#define _IR_SENSOR_HPP_

#pragma once

#include <iostream>
#include <Arduino.h>
#include <cmath>

class IRSensor {
    public:
    float magnitude, angle;
    bool read_success;
    bool read_serial(float* result, int num_floats);
    void update(); // call update every loop to read serial
    void angle_correction(float heading);
    float get_magnitude();
    float get_angle();
};

#endif