#ifndef _MODE_HPP_
#define _MODE_HPP_

#pragma once

#include <iostream>
#include <cmath>

#include "constants.h"
#include "bot_data.h"
#include "vector.hpp"

using namespace std;

class Mode {
    protected:
    float angle;
    float speed;
    float rotation;

    public:
    Mode();
    float get_angle();
    float get_speed();
    float get_rotation();
    virtual void update(BotData &self_data) = 0;
};

class ShingGetBehindBall : public Mode {
    public:
    void update(BotData &self_data);
    float find_move_angle(Vector goal_vec, float tolerance, float ball_angle, float ball_magnitude);
};

#endif