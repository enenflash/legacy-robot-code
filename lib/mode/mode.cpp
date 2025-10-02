#include "mode.hpp"

PID linear_pid;

float Mode::get_rotation(float target_angle, float heading) {
    float rotation = target_angle - heading - PI/2;
    while (rotation > PI) rotation -= 2*PI;
    while (rotation < -PI) rotation += 2*PI;
    return rotation;
}

OutputData OneRobot::update(BotData &self_data, BotData &other_data, float loop_time) {
    Vector opp_goal_vector = opp_goal_pos_vector.relative_to(self_data.pos_vector);
    this->rotation = opp_goal_vector.heading() - self_data.heading - PI/2;
    while (this->rotation > PI) this->rotation -= 2*PI;
    while (this->rotation < -PI) this->rotation += 2*PI;

    this->angle = this->find_move_angle(opp_goal_vector, self_data.ball_angle, self_data.ball_strength);
    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + PI;
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
    float angle_diff = PI / 2 - goal_vec.heading();
    if (ball_magnitude < 30) {
        return ball_angle;
    }
    if (ball_angle > goal_vec.heading() - FORWARD_TOLERANCE && ball_angle < goal_vec.heading() + FORWARD_TOLERANCE) {
        return goal_vec.heading(); // move forward
    }
    else if ((ball_angle > goal_vec.heading() + FORWARD_TOLERANCE) || (ball_angle < -PI / 2 + angle_diff)) {
        // Serial.println("Turning right");
        return ball_angle + PI / 18 * 6; // turn right
    }
    else if ((ball_angle < goal_vec.heading() - FORWARD_TOLERANCE)) {
        // Serial.println("Turning left");
        return ball_angle - PI / 18 * 6; // turn left
    }
    return 0.0;
}

BetterDefend::BetterDefend() {
    this->status = 0;
    this->target_posv = Vector(0, 0);
    this->target_vec = Vector(0, 0);
}

void BetterDefend::reset() {
    this->status = this->RETURNING;
}

OutputData BetterDefend::update(BotData &self_data, BotData &other_data, float loop_time) {
    // if in goal square and sees the ball, start defending
    if (self_data.pos_vector.i > -GOAL_WIDTH/2 && self_data.pos_vector.i < GOAL_WIDTH/2 && self_data.pos_vector.j <= -65 && self_data.ball_strength != 0) {
        this->status = this->DEFENDING;
    }
    else {
        this->status = this->RETURNING;
    }

    Vector goal_vec = own_goal_pos_vector.relative_to(self_data.pos_vector);

    // if defending go on the semi-circle
    if (this->status == this->DEFENDING) {
        // limit rotation
        if (self_data.ball_angle > 3*PI/2) {
            this->rotation = this->get_rotation(0, self_data.heading);
        }
        else if (self_data.ball_angle > PI) {
            this->rotation = this->get_rotation(PI, self_data.heading);
        }
        else {
            this->rotation = this->get_rotation(self_data.ball_angle, self_data.heading);
        }
        Vector ball_vector = Vector::from_heading(self_data.ball_angle, DEFEND_DIST);
        this->target_vec = Vector(goal_vec.i+ball_vector.i, goal_vec.j+ball_vector.j);
        this->angle = target_vec.heading();
    }
    // if returning go back to goal while avoiding the ball
    else if (this->status == this->RETURNING) {
        this->rotation = this->get_rotation(PI/2, self_data.heading);
        this->target_vec = Vector(goal_vec.i, goal_vec.j+15);
        this->angle = target_vec.heading();

        // difference in angle between ball angle and goal target vector
        float angle_diff = self_data.ball_angle - target_vec.heading();
        while (angle_diff > PI) angle_diff -= 2*PI;
        while (angle_diff < -PI) angle_diff += 2*PI;

        // if ball close and ball in the way off goal
        if (self_data.ball_strength >= 30 && angle_diff <= PI/2) {
            if (angle_diff != 0) this->angle += (abs(angle_diff)/angle_diff)*-PI/18*6;
            else this->angle += PI/18*6;
        }
        // if angle diff positive then ball is on the right therefore -60 degrees
        // if angle diff negative then ball is on the left therefore +60 degrees
    }

    // PID movement vector
    // Vector movement = linear_pid.get_movement(self_data.pos_vector, this->target_posv, MAX_SPEED, loop_time);
    // this->angle = movement.heading();
    // this->speed = movement.magnitude();

    this->dribbler_on = false;
    this->speed = 100;
    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + PI;
    }

    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=this->rotation, .dribbler_on=this->dribbler_on };
}

float BetterDefend::find_move_angle(Vector goal_vector, float ball_angle, float ball_strength) {

}

OutputData Defend::update(BotData &self_data, BotData &other_data, float loop_time) {
    Vector target_pos(0, 0);
    // If in goal square
    if (self_data.pos_vector.i > -GOAL_WIDTH/2 && self_data.pos_vector.i < GOAL_WIDTH/2 && self_data.pos_vector.j <= -65 && self_data.ball_strength != 0) {
        this->calib_and_return.step = 0;
        this->rotation = self_data.ball_angle - self_data.heading - PI/2;
        Vector ball_vector = Vector::from_heading(self_data.ball_angle, DEFEND_DIST);
        target_pos = Vector(own_goal_pos_vector.i+ball_vector.i, own_goal_pos_vector.j+ball_vector.j);
    }
    // calibration finished go to goal
    else if (this->calib_and_return.step == 2) {
        // this->rotation = -self_data.heading;
        // if (self_data.pos_vector.j > other_data.pos_vector.j - 20 && self_data.pos_vector.i > other_data.pos_vector.i) { // front right of attacking robot
        //     target_pos = Vector(other_data.pos_vector.i + 50, other_data.pos_vector.j - 20);
        // }
        // if (self_data.pos_vector.j > other_data.pos_vector.j - 20 && self_data.pos_vector.i < other_data.pos_vector.i) { // front left of attacking robot
        //     target_pos = Vector(other_data.pos_vector.i - 50, other_data.pos_vector.j - 20);
        // }
        // else {
        //     target_pos = Vector(own_goal_pos_vector.i, own_goal_pos_vector.j+15); // behind attacking robot
        // }
        target_pos = Vector(own_goal_pos_vector.i, own_goal_pos_vector.j+15); // behind attacking robot
    }
    else {
        return this->calib_and_return.update(self_data, other_data, loop_time);
    }

    // limit rotation to -180->180
    while (this->rotation > PI) this->rotation -= 2*PI;
    while (this->rotation < -PI) this->rotation += 2*PI;
    Vector movement = linear_pid.get_movement(self_data.pos_vector, target_pos, MAX_SPEED, loop_time);
    this->angle = movement.heading();
    this->speed = movement.magnitude();
    this->dribbler_on = false;
    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + PI;
    }
    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=this->rotation, .dribbler_on=this->dribbler_on };
}

OutputData CalibrateAndReturn::update(BotData &self_data, BotData &other_data, float loop_time) {
    if ((self_data.line_vector.magnitude() == 0 && this->previous_line_vec.magnitude() && this->previous_line_vec.heading() < PI / 4 && this->previous_line_vec.heading() > -PI / 4)) {
        this->step = 1;
        
    }
    if (self_data.line_vector.magnitude() == 0 && this->previous_line_vec.magnitude() && (this->previous_line_vec.heading() > 3 * PI / 4 || this->previous_line_vec.heading() < - 3 * PI / 4)) {
        this->step = 1;
        // Serial.println("just touched left line");
    }
    if (self_data.line_vector.magnitude() == 0 && this->previous_line_vec.magnitude() && this->previous_line_vec.heading() > -3 * PI / 4 && this->previous_line_vec.heading() < -PI / 4) {
        this->step = 2;
        // Serial.println("just touched back line");
    }
    Serial.printf("%d \n", this->step);
    
    this->speed = 80;
    if (this->step == 0 && self_data.pos_vector.i > 0) {
        this->angle = 0;
    }
    if (this->step == 0 && self_data.pos_vector.i < 0) {
        this->angle = PI;
    }
    if (this->step == 1) {
        this->angle = -PI/2;
    }
    this->previous_line_vec = self_data.line_vector;
    // return {}
    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + PI;
    }
    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=-self_data.heading, .dribbler_on=0 };
}

OutputData StayInLines::update(BotData &self_data, BotData &other_data, float loop_time) {
    this->speed = 80;

    // face goal
    Vector opp_goal_vector = opp_goal_pos_vector.relative_to(self_data.pos_vector);
    this->rotation = opp_goal_vector.heading() - self_data.heading - PI/2;
    while (this->rotation > PI) this->rotation -= 2*PI;
    while (this->rotation < -PI) this->rotation += 2*PI;

    // move opposite to line
    this->previous_line_vec = self_data.line_vector;
    if (self_data.line_vector.magnitude() != 0) {
        this->angle = self_data.line_vector.heading() + PI;
    }
    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=-self_data.heading, .dribbler_on=1 };
}

// Matches the other robot's heading and goes behind it (for testing bluetooth communication)
OutputData GoToRobot::update(BotData &self_data, BotData &other_data, float loop_time) {
    this->rotation = other_data.heading - self_data.heading;
    while (rotation > PI) rotation -= 2*PI;
    while (rotation < -PI) rotation += 2*PI;
    Vector target_pos = Vector(other_data.pos_vector.i, other_data.pos_vector.j-20);
    Vector movement = linear_pid.get_movement(self_data.pos_vector, target_pos, MAX_SPEED, loop_time);
    this->angle = movement.heading();
    this->speed = movement.magnitude();
    this->dribbler_on = false;
    return OutputData { .angle=this->angle, .speed=this->speed, .rotation=this->rotation, .dribbler_on=this->dribbler_on };
}