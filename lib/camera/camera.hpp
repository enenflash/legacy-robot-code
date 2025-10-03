#ifndef CAMERA_HPP
#define CAMERA_HPP
#pragma once

#include <iostream>
#include <Arduino.h>
#include <cmath>

class Camera {
    public:
    int goal_x = -1;
    int goal_y = -1;

    bool read_success = false;

    bool read_serial(int* result, int num_ints);
    void update(); // call update every loop to read serial
};

#endif