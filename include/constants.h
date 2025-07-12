#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include "vector.hpp"

const int FIELD_WIDTH = 182;
const int FIELD_LENGTH = 243;
const float GOAL_DIST_FROM_CENTRE = 91.5;
const int GOAL_WIDTH = 45;

const int ULTRASONIC_TOLERANCE = 20;
const float ULTRASONIC_TO_ROBOT = 9.5;

const float FORWARD_TOLERANCE = M_PI/10;

const float MAX_SPEED = 100;

const Vector opp_goal_pos_vector(0, 78.5);
const Vector own_goal_pos_vector(0, -78.5);

#endif