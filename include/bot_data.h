#ifndef _BOT_DATA_HPP_
#define _BOT_DATA_HPP_

#pragma once

#include <iostream>
#include "vector.hpp"

struct BotData {
    float heading;
    Vector pos_vector;
    float ball_strength;
    float ball_angle;
    Vector line_vector;
    Vector velocity;
};

#endif