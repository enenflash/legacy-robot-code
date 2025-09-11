#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include "vector.hpp"

// Measurements
const int FIELD_WIDTH = 182;
const int FIELD_LENGTH = 243;
const float GOAL_DIST_FROM_CENTRE = 91.5;
const int GOAL_WIDTH = 45;
const Vector opp_goal_pos_vector(0, 91.5);
const Vector own_goal_pos_vector(0, -91.5);

// Tolerance and Distances
const int ULTRASONIC_TOLERANCE = 20;
const float ULTRASONIC_TO_ROBOT = 9.5;
const float FORWARD_TOLERANCE = M_PI/10;
const float DEFEND_DIST = 15;

// Limits
const float MAX_SPEED = 100;
const float ball_triangulation_angle_limit = 1*M_PI/18; // 10 degrees

#endif