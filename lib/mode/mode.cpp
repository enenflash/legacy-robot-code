#include "mode.hpp"

Mode::Mode() {
    this->angle = 0;
    this->speed = 100;
    this->rotation = 0;
}

float Mode::get_angle() { return this->angle; }
float Mode::get_speed() { return this->speed; }
float Mode::get_rotation() { return this->rotation; }

void ShingGetBehindBall::update(BotData &self_data) {
    this->speed = 100;
    this->rotation = fmodf(M_PI + self_data.opp_goal_vector.heading() - self_data.heading - M_PI/2, 2*M_PI) - M_PI;
    this->angle = this->find_move_angle(self_data.opp_goal_vector, FORWARD_TOLERANCE, self_data.ball_angle, self_data.ball_strength);
}

float ShingGetBehindBall::find_move_angle(Vector goal_vec, float tolerance, float ball_angle, float ball_magnitude) {
    float angle_diff = M_PI / 2 - goal_vec.heading();
    if (ball_magnitude < 40) {
        return ball_angle;
    }
    if (ball_angle > goal_vec.heading() - tolerance && ball_angle < goal_vec.heading() + tolerance) {
        // return goal_vec.heading();
        return goal_vec.heading(); // move forward
    }
    else if ((ball_angle > goal_vec.heading() + tolerance) || (ball_angle < -M_PI / 2 + angle_diff)) {
        return ball_angle + M_PI / 18 * 6; // turn right
    }
    else if ((ball_angle < goal_vec.heading() - tolerance)) {
        return ball_angle - M_PI / 18 * 6; // turn left
    }
    return 0.0;
}