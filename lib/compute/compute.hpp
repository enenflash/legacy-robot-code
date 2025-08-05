#ifndef _COMPUTE_HPP_
#define _COMPUTE_HPP_

#pragma once

#include "constants.h"
#include "vector.hpp"

class Compute {
    public:
    bool can_triangulate(Vector posv1, float ball_angle1, Vector posv2, float ball_angle2);
    Vector triangulate_ball(Vector posv1, float ball_angle1, Vector posv2, float ball_angle2);
    bool check_collision(float clearance, Vector posv1, float mv_angle1, float speed1, Vector posv2, float mv_angle2, float speed2);
};

#endif