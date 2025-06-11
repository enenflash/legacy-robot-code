#ifndef _LINE_SENSOR_HPP_
#define _LINE_SENSOR_HPP_

#pragma once

#include <iostream>
#include <Arduino.h>
#include <cmath>

class LineSensor {
    public:
    float distance, angle;
    bool read_success;
    bool read_serial(float* result, int num_floats);
    void update(); // call update every loop to read serial
    void angle_correction(float heading);
    float get_distance();
    float get_angle();
};

#endif