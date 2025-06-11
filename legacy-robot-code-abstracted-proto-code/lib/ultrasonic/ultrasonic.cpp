#include "ultrasonic.hpp"

Ultrasonic::Ultrasonic(int trigger_pin, int echo_pin) {
    this->TRIGGER_PIN = trigger_pin;
    this->ECHO_PIN = echo_pin;
}

int Ultrasonic::get_distance() {
    long duration, ult_distance;

    // clear trigger pin
    digitalWrite(this->TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(this->TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->TRIGGER_PIN, LOW);

    // get duration in miliseconds
    duration = pulseIn(this->ECHO_PIN, HIGH, 20000);
    
    // converet to cm
    ult_distance = (duration / 29) / 2;  // Calculating the ult_distance

    return ult_distance;
}