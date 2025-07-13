#include "mode.hpp"

PID linear_pid;

OutputData OneRobot::update(BotData &self_data, BotData &other_data, float loop_time) {
    Vector opp_goal_vector = opp_goal_pos_vector.relative_to(self_data.pos_vector);
    this->rotation = opp_goal_vector.heading() - self_data.heading - M_PI/2;
    while (rotation > M_PI) rotation -= 2*M_PI;
    while (rotation < -M_PI) rotation += 2*M_PI;

    this->angle = this->find_move_angle(opp_goal_vector, self_data.ball_angle, self_data.ball_strength);
    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + M_PI;
    }

    this->speed = MAX_SPEED;
    this->dribbler_on = true;

    if (self_data.ball_strength == 0  && self_data.line_vector.magnitude() == 0) {
        this->speed = 0;
        this->dribbler_on = false;
    }
    if (self_data.ball_strength < 40) {
        this->dribbler_on = false;
    }

    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=this->rotation, .dribbler_on=this->dribbler_on };
}

float OneRobot::find_move_angle(Vector goal_vec, float ball_angle, float ball_magnitude) {
    float angle_diff = M_PI / 2 - goal_vec.heading();
    if (ball_magnitude < 40) {
        return ball_angle;
    }
    if (ball_angle > goal_vec.heading() - FORWARD_TOLERANCE && ball_angle < goal_vec.heading() + FORWARD_TOLERANCE) {
        return goal_vec.heading(); // move forward
    }
    else if ((ball_angle > goal_vec.heading() + FORWARD_TOLERANCE) || (ball_angle < -M_PI / 2 + angle_diff)) {
        return ball_angle + M_PI / 18 * 6; // turn right
    }
    else if ((ball_angle < goal_vec.heading() - FORWARD_TOLERANCE)) {
        return ball_angle - M_PI / 18 * 6; // turn left
    }
    return 0.0;
}

OutputData Defend::update(BotData &self_data, BotData &other_data, float loop_time) {
    Vector target_pos(0, 0);
    // If in goal square
    if (self_data.pos_vector.i > -GOAL_WIDTH/2 && self_data.pos_vector.i < GOAL_WIDTH/2 && self_data.pos_vector.j <= -65 && self_data.ball_strength != 0) {
        this->rotation = self_data.ball_angle - self_data.heading - M_PI/2;
        Vector ball_vector = Vector::from_heading(self_data.ball_angle, DEFEND_DIST);
        target_pos = Vector(own_goal_pos_vector.i+ball_vector.i, own_goal_pos_vector.j+ball_vector.j);
    }
    else {
        this->rotation = -self_data.heading;
        target_pos = Vector(own_goal_pos_vector.i, own_goal_pos_vector.j+15);
    }

    // limit rotation to -180->180
    while (this->rotation > M_PI) this->rotation -= 2*M_PI;
    while (this->rotation < -M_PI) this->rotation += 2*M_PI;
    Vector movement = linear_pid.get_movement(self_data.pos_vector, target_pos, MAX_SPEED, loop_time);
    this->angle = movement.heading();
    this->speed = movement.magnitude();
    this->dribbler_on = false;

    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=this->rotation, .dribbler_on=this->dribbler_on };
}

// Matches the other robot's heading and goes behind it (for testing bluetooth communication)
OutputData GoToRobot::update(BotData &self_data, BotData &other_data, float loop_time) {
    this->rotation = other_data.heading - self_data.heading - M_PI/2;
    while (rotation > M_PI) rotation -= 2*M_PI;
    while (rotation < -M_PI) rotation += 2*M_PI;
    Vector target_pos = Vector(other_data.pos_vector.i, other_data.pos_vector.j-20);
    Vector movement = linear_pid.get_movement(self_data.pos_vector, target_pos, MAX_SPEED, loop_time);
    this->angle = movement.heading();
    this->speed = movement.magnitude();
    this->dribbler_on = false;
    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=this->rotation, .dribbler_on=this->dribbler_on };
}