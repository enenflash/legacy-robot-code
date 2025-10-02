#ifndef _OUTPUT_DATA_HPP_
#define _OUTPUT_DATA_HPP_

#pragma once

#include <iostream>

// A struct for robot outputs (angle refers to the movement angle)
struct OutputData {
    float angle;
    float speed;
    float rotation;
    bool dribbler_on;
};

#endif