#ifndef _BOT_DATA_HPP_
#define _BOT_DATA_HPP_

#pragma once

#include <iostream>
#include "vector.hpp"

using namespace std;

struct BotData {
    bool possession;
    float heading;
    Vector pos_vector;
    Vector opp_goal_vector;
    float ball_strength;
    float ball_angle;
};

#endif