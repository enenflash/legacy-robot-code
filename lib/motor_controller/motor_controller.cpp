#include "motor_controller.hpp"

MotorController::MotorController(float rotation_constant) {
    this->rotation_const = rotation_constant;
    
    this->TL = Motor(TL_PWM, TL_DIR);
    this->TR = Motor(TR_PWM, TR_DIR);
    this->BL = Motor(BL_PWM, BL_DIR);
    this->BR = Motor(BR_PWM, BR_DIR);
}

std::array<float, 4> MotorController::get_motor_speeds(float speed, float angle, float rotation) {
    Vector mv = Vector::from_heading(angle, speed);
    float motor_ratio[4] = { 
        -mv.i - mv.j + rotation*this->rotation_const, 
        -mv.i + mv.j + rotation*this->rotation_const, 
         mv.i - mv.j + rotation*this->rotation_const, 
         mv.i + mv.j + rotation*this->rotation_const 
    };
    // Rounding Motor Ratio to Prevent Errors
    for (int i = 0; i < 4; i++) {
        motor_ratio[i] = round(motor_ratio[i] * 1000) / 1000;
    }
    int index = 0;
    for (int i = 1; i < 4; i++) {
        if (abs(motor_ratio[i]) > abs(motor_ratio[index])) {
            index = i;
        }
    }
    float highest = abs(motor_ratio[index]);
    std::array<float, 4> motor_speeds = { 0, 0, 0, 0 };
    if (highest == 0) { // avoid zero division
        return motor_speeds;
    }
    for (int i = 0; i < 4; i++) {
        motor_speeds[i] = motor_ratio[i]/highest*speed;
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