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
    float get_rotation(float target_angle, float heading);
    virtual OutputData update(BotData &self_data, BotData &other_data, float loop_time) = 0;
};

class OneRobot : public Mode {
    private:
    float find_move_angle(Vector goal_vector, float ball_angle, float ball_strength);
    public:
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

class CalibrateAndReturn : public Mode {
    private:
    Vector previous_line_vec;
    public:
    uint8_t step;
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

class Defend : public Mode {
    public:
    CalibrateAndReturn calib_and_return;
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

class BetterDefend : public Mode {
    private:
    Vector target_posv;
    int status;

    public:
    const int RETURNING = 0;
    const int DEFENDING = 1;
    BetterDefend();
    void reset();
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);

    // outsiders can view status but can't modify it
    const int& get_status_code() const {
        return this->status;
    };
    // use by calling the function
};

class StayInLines : public Mode {
    private:
    Vector previous_line_vec;
    public:
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

class GoToRobot : public Mode {
    public:
    OutputData update(BotData &self_data, BotData &other_data, float loop_time);
};

#endif