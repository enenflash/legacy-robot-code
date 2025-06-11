#include "dribbler.hpp"

DribblerMotor::DribblerMotor(int dir_pin, int pwm_pin) {
    this->DIR_PIN = dir_pin;
    this->PWM_PIN = pwm_pin;
}

void DribblerMotor::run(float speed) {
    int pwmSpeed = speed / 100 * 255;
    digitalWrite(this->DIR_PIN, LOW);
    analogWrite(this->PWM_PIN, pwmSpeed);
}

void DribblerMotor::stop() {
    digitalWrite(this->DIR_PIN, LOW);
    analogWrite(this->PWM_PIN, 0);
}