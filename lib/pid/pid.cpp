#include <pid.hpp>
#include <math.h>
#include "vector.hpp"

float PID::max(float var1, float var2) {
    if (var1 > var2) return var1;
    return var2;
}

float PID::min(float var1, float var2) {
    if (var1 < var2) return var1;
    return var2;
}

double PID::compute(double error, double dt) {
    this->integral += error * dt;
    this->derivitive = (error - this->previousError) / dt;
    this->previousError = error;
    return this->PROPORTIONAL_CONSTANT * error + this->INTEGRAL_CONSTANT * this->integral + this->DERIVETIVE_CONSTANT * this->derivitive; 
}

Vector PID::moveTo(float TARGET_X, float TARGET_Y, float x, float y, float maxSpeed, double dt) {
    float dx = TARGET_X - x;
    float dy = TARGET_Y - y;
    float distance = sqrt(pow(dx, 2) + pow(dy, 2));


    if (distance <= 0.3) {
        this->angle = 0;
        this->speed = 0;
        this->integral = 0;
        return Vector::from_heading(0, 0);
    }
    
    if (distance <= 2) this->MINIMUM_PID_SPEED = 10;
    else this->MINIMUM_PID_SPEED = 30;

    Vector target_vector = Vector(dx/distance, dy/distance);
    this->angle = target_vector.heading();
    

    this->speed = this->compute(distance, dt);
    speed = this->max(0, this->min(speed, maxSpeed));
    if (speed > 0 && speed < this->MINIMUM_PID_SPEED) {
      speed = MINIMUM_PID_SPEED;
    }

    return Vector::from_heading(this->angle, this->speed);
};