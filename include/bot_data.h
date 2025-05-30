#ifndef _BOT_DATA_HPP_
#define _BOT_DATA_HPP_

#pragma once

#include <iostream>
#include "vector.hpp"

using namespace std;

struct BotData {
    bool possession;
    float heading;
    Vector pos_vec;
    Vector ball_vec;
    Vector line_vec;
};

#endif