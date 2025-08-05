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
    this->derivitive = (error - this->previous_error) / dt;
    this->previous_error = error;
    return this->PROPORTIONAL_CONSTANT * error + this->INTEGRAL_CONSTANT * this->integral + this->DERIVETIVE_CONSTANT * this->derivitive; 
}

Vector PID::get_movement(Vector pos, Vector target_pos, float max_speed, double dt) {
    Vector target_pos_relative = Vector(target_pos.i-pos.i, target_pos.j-pos.j);
    float distance = target_pos_relative.magnitude();

    if (distance <= 0.3) {
        this->angle = 0;
        this->speed = 0;
        this->integral = 0;
        return Vector::from_heading(0, 0);
    }
    
    if (distance <= 2) this->MINIMUM_PID_SPEED = 10;
    else this->MINIMUM_PID_SPEED = 30;

    this->angle = target_pos_relative.heading();
    this->speed = this->compute(distance, dt);
    this->speed = this->max(0, this->min(this->speed, max_speed));
    if (this->speed > 0 && this->speed < this->MINIMUM_PID_SPEED) {
      this->speed = MINIMUM_PID_SPEED;
    }
    return Vector::from_heading(this->angle, this->speed);
};