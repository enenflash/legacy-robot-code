#include "motor_controller.hpp"

// rotation constant is how much the rotation is scaled compared to the movement
MotorController::MotorController(float min_rotation_speed) {
    this->min_rotation_speed = min_rotation_speed;
    
    this->TL = Motor(TL_PWM, TL_DIR);
    this->TR = Motor(TR_PWM, TR_DIR);
    this->BL = Motor(BL_PWM, BL_DIR);
    this->BR = Motor(BR_PWM, BR_DIR);
}

std::array<float, 4> MotorController::scale_speeds(std::array<float, 4> speeds, float scale_to) {
    int index = 0;
    for (int i = 1; i < 4; i++) {
        if (abs(speeds[i]) > abs(speeds[index])) {
            index = i;
        }
    }
    float highest = abs(speeds[index]);
    std::array<float, 4> scaled_speeds = { 0, 0, 0, 0 };
    if (highest == 0) { // avoid zero division
        return scaled_speeds;
    }
    for (int i = 0; i < 4; i++) {
        scaled_speeds[i] = speeds[i]/highest*scale_to;
    }
    return scaled_speeds;
}

std::array<float, 4> MotorController::get_motor_speeds(float movement_speed, float angle, float rotation) {
    float rotation_speed = -50/pow(M_PI, 3)*rotation*(pow(rotation, 2)-3*pow(M_PI, 2));
    float speed = std::min((float)100, rotation_speed+movement_speed);
    Vector mv = Vector::from_heading(angle, 1);
    std::array<float, 4> motor_speeds = scale_speeds(
        { -mv.i - mv.j,-mv.i + mv.j, mv.i - mv.j, mv.i + mv.j }, 
        speed*movement_speed/(movement_speed+rotation_speed)
    );
    for (int i = 0; i < 4; i++) {
        motor_speeds[i] += speed*rotation_speed/(movement_speed+rotation_speed);
    }
    return motor_speeds;
}

// speed 0->100, angle and rotation in radians
void MotorController::run_motors(float speed, float angle, float rotation) {
    std::array<float, 4> motor_speeds = this->get_motor_speeds(speed, angle, rotation);

    this->TL.run(motor_speeds[0]);
    this->TR.run(motor_speeds[1]);
    this->BL.run(motor_speeds[2]);
    this->BR.run(motor_speeds[3]);
}

void MotorController::run_raw(float tl_raw, float tr_raw, float bl_raw, float br_raw) {
    this->TL.run(tl_raw);
    this->TR.run(tr_raw);
    this->BL.run(bl_raw);
    this->BR.run(br_raw);
}

void MotorController::stop_motors() {
    this->TL.stop();
    this->TR.stop();
    this->BL.stop();
    this->BR.stop();
}