#include "ultrasonic_ps.hpp"

// check if left_dist + right_dist = the field width
bool UltrasonicPS::equals_total_width(float left_dist, float right_dist) {
    return abs((left_dist+right_dist)-FIELD_WIDTH) < ULTRASONIC_TOLERANCE;
}

// return true if use left ultrasonic
bool UltrasonicPS::choose_ult() {
    float delta_distances[3] = { 0, 0, 0 };
    for (int i = 0; i < 3; i++) {
        delta_distances[i] = abs(this->ult_distances[i]-this->previous_distances[i]);
    }

    // if both unreliable
    if (delta_distances[0] > 20 && delta_distances[1] > 20) {
        this->reliable = false;
        return true;
    }
    
    this->reliable = true;

    if (delta_distances[0] > 20) {
        return false; // use right
    }
    if (delta_distances[1] > 20) {
        return true; // use left
    }
    return true;
}

// uses centre of field as (0, 0)
// call every loop
void UltrasonicPS::update(float angle_offset) {
    ult_distances[0] = cos(angle_offset)*(ult_left.get_distance() + ULTRASONIC_TO_ROBOT);
    ult_distances[1] = cos(angle_offset)*(ult_right.get_distance() + ULTRASONIC_TO_ROBOT);
    ult_distances[2] = cos(angle_offset)*(ult_back.get_distance() + ULTRASONIC_TO_ROBOT);

    if (!this->equals_total_width(ult_distances[0], ult_distances[1])) {
        use_left = this->choose_ult();
    }

    if (use_left) {
        this->x = ult_distances[0] - FIELD_LENGTH/2;
    }
    else {
        this->x = ult_distances[1] - FIELD_LENGTH/2;
    }
    this->y = ult_distances[2] - FIELD_LENGTH/2;

    if (this->equals_total_width(ult_distances[0], ult_distances[1])) {
        for (int i = 0; i < 3; i++) {
            this->previous_distances[i] = ult_distances[i];
        }
    }
}

// returns pos vector of the robot
Vector UltrasonicPS::get_posv() {
    return Vector(this->x, this->y);
}