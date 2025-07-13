#ifndef _MODE_HPP_
#define _MODE_HPP_

#pragma once

#include <iostream>
#include <cmath>

#include "constants.h"
#include "bot_data.h"
#include "output_data.h"
#include "vector.hpp"
#include "position_system.hpp"
#include "pid.hpp"

class Mode {
    protected:
    float angle;
    float speed;
    float rotation;
    bool dribbler_on;

    public:
    virtual OutputData update(BotData &self_data, BotData &other_data, float loop_time) = 0;
};

class OneRobot : public Mode {
    private:
    float find_move_angle(Vector goal_vector, float ball_angle, float ball_strength);
    public:
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

class Defend : public Mode {
    public:
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

class GoToRobot : public Mode {
    public:
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

#endif