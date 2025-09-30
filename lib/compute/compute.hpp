#ifndef _COMPUTE_HPP_
#define _COMPUTE_HPP_

#pragma once

#include "constants.h"
#include "vector.hpp"

class Compute {
    public:
    /* ---------------------------- BASIC CONVERSIONS --------------------------- */
    static float bearing_to_unit_angle(float bearing_angle);
    static float unit_angle_to_bearing(float unit_angle);

    /* ---------------------- COLLISIONS AND TRUE POSITIONS --------------------- */
    static bool can_triangulate(Vector posv1, float ball_angle1, Vector posv2, float ball_angle2);
    static Vector triangulate_ball(Vector posv1, float ball_angle1, Vector posv2, float ball_angle2);
    static bool check_collision(float clearance, Vector posv1, float mv_angle1, float speed1, Vector posv2, float mv_angle2, float speed2);

    /* --------------------------- CAMERA CALCULATIONS -------------------------- */
    static float screenx_to_angle(int screen_x);
    static int angle_to_screenx(float angle);

    static Vector goal_target_px_to_rposv(Vector posv, int screen_x);
};

#endif