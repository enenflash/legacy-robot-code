#include "mode.hpp"

Mode::Mode() {
    this->angle = 0;
    this->speed = 100;
    this->rotation = 0;
    this->dribbler_on = false;
}
float Mode::get_angle() { return this->angle; }
float Mode::get_speed() { return this->speed; }
float Mode::get_rotation() { return this->rotation; }
bool Mode::get_dribbler_on() { return this->dribbler_on; }
Mode* Mode::get_pointer() { return this; }

void StandardMode::update(BotData &self_data) {
    this->rotation = fmodf(self_data.heading-M_PI, 2*M_PI) + M_PI;
    this->dribbler_on = false;
    // if no ball found, don't move
    if (self_data.ball_strength == 0 && self_data.line_vector.magnitude() == 0) {
        this->angle = 0, this->speed = 0;
        return;
    }
    this->speed = 80;
    // if ball far, move directly towards ball
    if (self_data.ball_strength > 40) {
        this->angle = self_data.ball_angle;
        return;
    }
    // ball on left
    if (self_data.ball_angle >= M_PI/2 + self_data.heading + FORWARD_TOLERANCE && self_data.ball_angle < 3*M_PI/2 + self_data.heading) {
        this->angle = self_data.ball_angle + M_PI / 18 * 7;
        return;
    }
    // ball on right
    if (self_data.ball_angle <= self_data.opp_goal_vector.heading() - FORWARD_TOLERANCE || self_data.ball_angle >= 3*M_PI/2 + self_data.heading) {
        this->angle = self_data.ball_angle - M_PI / 18 * 7;
        return;
    }
    this->angle = 0;
}

void OrbitBall::update(BotData &self_data) {
    Serial.print("Ball angle received: "); Serial.println(self_data.ball_angle*180/PI);
    // face the goal at all times
    this->rotation = fmodf(self_data.opp_goal_vector.heading()-self_data.heading-M_PI/2 - M_PI, 2*M_PI) + M_PI;
    // if opposite goal angle is greater than 180 just face forward (OTOS has probably drifted)
    // if (self_data.opp_goal_vector.heading() > M_PI) {
    //     this->rotation = fmodf(-self_data.heading-M_PI, 2*M_PI) + M_PI;
    // }
    // if no ball found, don't move
    if ((self_data.ball_strength == 0) && (self_data.line_vector.magnitude() == 0)) {
        Serial.println("Case 1");
        this->angle = 0, this->speed = 0, this->dribbler_on = false;
        return;
    }

    this->speed = 100;
    if (!PositionSystem::within_opp_goal_range(self_data.pos_vector)) this->dribbler_on = true;
    else this->dribbler_on = false;

    // if on the line, move away from line direction (except if in front of opponent goal)
    if (self_data.line_vector.magnitude() != 0) { //&& !PositionSystem::within_opp_goal_range(self_data.pos_vector)) {
        this->angle = self_data.line_vector.heading() + M_PI;
        return;
    }
    // if ball far, move directly towards ball
    if (self_data.ball_strength < 40) {
        Serial.println("Case 2");
        this->angle = self_data.ball_angle;
        return;
    }
    // ball on left
    if ((self_data.ball_angle >= M_PI/2 + self_data.heading + FORWARD_TOLERANCE) && (self_data.ball_angle < 3*M_PI/2 + self_data.heading)) {
        Serial.println("Case 3");
        this->angle = self_data.ball_angle + M_PI / 18 * 7;
        return;
    }
    // ball on right
    if ((self_data.ball_angle <= M_PI/2 + self_data.heading - FORWARD_TOLERANCE) || (self_data.ball_angle >= 3*M_PI/2 + self_data.heading)) {
        Serial.println("Case 4");
        this->angle = self_data.ball_angle - M_PI / 18 * 7;
        return;
    }
    this->angle = 0;
    Serial.println("Case 5");
}

void TargetGoalOTOS::update(BotData &self_data) {
    this->speed = 100;
    if (!PositionSystem::within_opp_goal_range(self_data.pos_vector)) this->dribbler_on = true;
    else this->dribbler_on = false;
    // face the goal
    this->rotation = fmodf(self_data.opp_goal_vector.heading()-self_data.heading-M_PI/2 - M_PI, 2*M_PI) + M_PI;
    // move towards goal
    this->angle = self_data.opp_goal_vector.heading();
}

// ignore this
void IROnly::update(BotData &self_data) {
    // get rotation from heading
    if (self_data.heading <= 180) this->rotation = -self_data.heading;
    else this->rotation = 360-self_data.heading;

    // no ball detected
    if (self_data.ball_strength == 0 && self_data.line_vector.magnitude() == 0) {
        this->speed = 0;
        this->angle = 0;
        this->dribbler_on = false;
        return;
    }
    this->speed = 100;
    this->dribbler_on = true;

    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + M_PI;
        return;
    }
    if (self_data.ball_strength > 40) {
        this->angle = self_data.ball_angle;
        return;
    }
    if ((self_data.ball_angle > M_PI/2 - FORWARD_TOLERANCE) && (self_data.ball_angle < M_PI/2 + FORWARD_TOLERANCE)) {
        this->angle = M_PI;
        return;
    }
}

// based on shings old code (untested)
void ShingGetBehindBall::update(BotData &self_data) {
    this->speed = 100;
    this->rotation = fmodf(M_PI + self_data.opp_goal_vector.heading() - self_data.heading - M_PI/2, 2*M_PI) - M_PI;
    this->angle = this->find_move_angle(self_data.opp_goal_vector, FORWARD_TOLERANCE, self_data.ball_angle, self_data.ball_strength);
    this->dribbler_on = true;

    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + M_PI;
    }
    if (self_data.ball_strength == 0 && self_data.line_vector.magnitude() == 0) {
        this->speed = 0;
        this->dribbler_on = false;
    }
}

float ShingGetBehindBall::find_move_angle(Vector goal_vec, float tolerance, float ball_angle, float ball_magnitude) {
    float angle_diff = M_PI / 2 - goal_vec.heading();
    if (ball_magnitude < 40) {
        return ball_angle;
    }
    if (ball_angle > goal_vec.heading() - tolerance && ball_angle < goal_vec.heading() + tolerance) {
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