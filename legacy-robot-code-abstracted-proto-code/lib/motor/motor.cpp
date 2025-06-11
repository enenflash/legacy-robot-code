#include "motor.hpp"

Motor::Motor(int pwm_pin, int dir_pin) {
  this->PWM_PIN = pwm_pin;
  this->DIR_PIN = dir_pin;
}

void Motor::run(float speed) {
  // get pwm from speed
  int pwmSpeed = (100 - abs(speed)) / 100 * 255;

  if (speed > 0) {
    digitalWrite(this->DIR_PIN, LOW);
  }
  else {
    digitalWrite(this->DIR_PIN, HIGH);
  }

  analogWrite(this->PWM_PIN, pwmSpeed);
}

void Motor::stop() {
  analogWrite(this->PWM_PIN, 255);
}