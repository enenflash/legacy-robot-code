#include "compute.hpp"

/* -------------------------------------------------------------------------- */
/*                              BASIC CONVERSIONS                             */
/* -------------------------------------------------------------------------- */

// Accepts bearing angle in degrees and converts it to unit circle angle radians
float Compute::bearing_to_unit_angle(float bearing_angle) {
    float unit_angle = bearing_angle;
    if (unit_angle < 0) unit_angle += 360;
    return 2*M_PI - unit_angle*M_PI/180;
}

// Accepts unit circle angle in radians and converts it to bearing angle degrees
float Compute::unit_angle_to_bearing(float unit_angle) {
    float bearing_angle = 360-unit_angle*180/M_PI;
    if (bearing_angle >= 180) bearing_angle -= 360;
    return bearing_angle;
}

/* -------------------------------------------------------------------------- */
/*                        COLLISIONS AND TRUE POSITIONS                       */
/* -------------------------------------------------------------------------- */

bool Compute::can_triangulate(Vector posv1, float ball_angle1, Vector posv2, float ball_angle2) {
    // colinear - ball in same direction
    if (abs(ball_angle1-ball_angle2) <= ball_triangulation_angle_limit) {
        return false;
    }
    // colinear - ball in between
    if (abs(fmodf(ball_angle1 + M_PI, 2 * M_PI)-ball_angle2) <= ball_triangulation_angle_limit) {
        return false;
    }
    return true;
}

Vector Compute::triangulate_ball(Vector posv1, float ball_angle1, Vector posv2, float ball_angle2) {
    Vector ballv1 = Vector::from_heading(ball_angle1, 1);
    Vector ballv2 = Vector::from_heading(ball_angle2, 1);
    float time1 = ((posv1.i-posv2.i)/ballv2.i - (posv1.j-posv2.j)/ballv2.j)/(ballv1.j/ballv2.j - ballv1.i/ballv2.i);
    Vector ball_posv = Vector(
        posv1.i + time1 * ballv1.i,
        posv1.j + time1 * ballv1.j
    );
    return ball_posv;
}

bool Compute::check_collision(float clearance, Vector pos_a, float mv_angle_a, float speed_a, Vector pos_b, float mv_angle_b, float speed_b) {
    Vector vel_a = Vector::from_heading(mv_angle_a, speed_a);
    Vector vel_b = Vector::from_heading(mv_angle_b, speed_b);
    Vector a_pos_b = pos_a.relative_to(pos_b);
    Vector a_vel_b = vel_a.relative_to(vel_b);

    float lambda = -(a_pos_b.dot(a_vel_b))/(a_vel_b.dot(a_vel_b));
    Vector change_in_pos = a_vel_b.scale(lambda);
    float closest_distance = Vector(change_in_pos.i+a_pos_b.i, change_in_pos.j+a_pos_b.j).magnitude();

    return closest_distance <= clearance;
}

/* -------------------------------------------------------------------------- */
/*                             CAMERA CALCULATIONS                            */
/* -------------------------------------------------------------------------- */

// give these correct values then move them to constants.h and to camera code
const int FOV_angle = 60;
const int full_image_dim[2] = {1000, 1000};
const int cropped_image_dim[2] = {900, 900};
const float angle_per_pixel[2] = {FOV_angle/full_image_dim[0], FOV_angle/full_image_dim[1]};

// move this to camera code
#include <array>
std::array<int, 2> get_pixel_pos_relative_to_centre(float raw_pixel_pos[2]) {
    std::array<int, 2> relative_pixel_pos = {
        raw_pixel_pos[0] - cropped_image_dim[0]/2,
        raw_pixel_pos[1] - cropped_image_dim[1]/2
    };
    return relative_pixel_pos;
}

// Converts the x-coordinate of a point on the camera image to an angle (unit circle format)
// screen_x is received as the distance from the centre of the image
float Compute::screenx_to_angle(int screen_x) {
    return M_PI/2 - screen_x * angle_per_pixel[0];
}

// Converts an angle of an object (unit circle format) to the camera screen x-coordinate
// screen_x is returned as the distance from the centre of the image
int Compute::angle_to_screenx(float angle) {
    return (M_PI/2 - angle)/angle_per_pixel[0];
}

// Converts the goal target pixel position to a relative position vector
// Note: the angle is only dependent on the camera footage, however the distance uses the OTOS
Vector Compute::goal_target_px_to_rposv(Vector posv, int screen_x) {
    float goal_target_angle = this->screenx_to_angle(screen_x);
    Vector goal_target_rposv = Vector(
        (opp_goal_pos_vector.j - posv.j)/tan(goal_target_angle),
        opp_goal_pos_vector.j - posv.j
    );
    return goal_target_rposv;
}