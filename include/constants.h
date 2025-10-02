#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include "vector.hpp"

// Settings
const bool RELEASE_BALL = true;

// Measurements
const int FIELD_WIDTH = 182;
const int FIELD_LENGTH = 243;
const float GOAL_DIST_FROM_CENTRE = 91.5;
const int GOAL_WIDTH = 45;
const Vector opp_goal_pos_vector(0, 90.0);
const Vector own_goal_pos_vector(0, -82.0);

// Tolerance and Distances
const int ULTRASONIC_TOLERANCE = 20;
const float ULTRASONIC_TO_ROBOT = 9.5;
const float FORWARD_TOLERANCE = M_PI/10;
const float DEFEND_DIST = 30; // not used by better defend
const int SLOW_DOWN_DIST = 15;

// BETTER DEFEND
const float DEFEND_Y = 20;
const float DEFEND_X = 22;
const float THETA = std::atan2(DEFEND_X, DEFEND_Y);
const float DEFEND_OFFSET = std::pow(std::pow(DEFEND_X, 2) + std::pow(DEFEND_Y, 2), 0.5)/sin(M_PI-THETA*2)*sin(THETA) - DEFEND_Y;
const float DEFEND_CENTRE_Y = - GOAL_DIST_FROM_CENTRE - DEFEND_OFFSET;

// Limits
const float MAX_SPEED = 100;
const float RELEASE_SPEED = 30;
const float ball_triangulation_angle_limit = 1*M_PI/18; // 10 degrees

#endif