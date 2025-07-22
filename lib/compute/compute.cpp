#include "compute.hpp"

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