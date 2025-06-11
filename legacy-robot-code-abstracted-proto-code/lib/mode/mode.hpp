#ifndef _MODE_HPP_
#define _MODE_HPP_

#pragma once

#include <iostream>
#include <cmath>

#include "constants.h"
#include "bot_data.h"
#include "vector.hpp"
#include "position_system.hpp"

#define ORBIT_BALL 0
#define TARGET_GOAL_OTOS 1
#define DEFEND 2

class Mode {
    protected:
    float angle;
    float speed;
    float rotation;
    bool dribbler_on;

    public:
    Mode();
    float get_angle();
    float get_speed();
    float get_rotation();
    bool get_dribbler_on();
    Mode* get_pointer();
    virtual void update(BotData &self_data) = 0;
};

// no stay within lines, speed reduced to 80, no dribbler
class StandardMode : public Mode {
    public:
    void update(BotData &self_data);
};

class ShingGetBehindBall : public Mode {
    public:
    void update(BotData &self_data);
    float find_move_angle(Vector goal_vec, float tolerance, float ball_angle, float ball_magnitude);
};

class IROnly : public Mode {
    public:
    void update(BotData &self_data);
};

// mode for getting behind the ball (orbit)
class OrbitBall : public Mode {
    public:
    void update(BotData &self_data);
};

// mode for targeting the goal using OTOS
class TargetGoalOTOS : public Mode {
    public:
    void update(BotData &self_data);
};

class Goalie : public Mode {
    public:
    void update(BotData &self_data);
};

#endif