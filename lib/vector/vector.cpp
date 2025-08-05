#include "vector.hpp"

Vector::Vector(float new_i, float new_j) {
    this->i = new_i;
    this->j = new_j;
}

std::string Vector::display() const {
    std::string display_i = std::to_string(this->i), display_j = std::to_string(this->j);
    if ((int)this->i == this->i) {
        display_i = std::to_string((int)this->i);
    }
    if ((int)this->j == this->j) {
        display_j = std::to_string((int)this->j);
    }
    return "<"+display_i+", "+display_j+">";
}

float Vector::magnitude() const {
    return pow(pow(this->i, 2) + pow(this->j, 2), 0.5);
}

float Vector::heading() const {
    return atan2(this->j, this->i);
}

Vector Vector::unit() const {
    float magnitude = this->magnitude();
    if (magnitude == 0) return Vector(0, 0);
    return Vector(this->i/magnitude, this->j/magnitude);
}

float Vector::dot(Vector vec2) const {
    return this->i*vec2.i + this->j*vec2.j;
}
Vector Vector::scale(float scalar) const {
    return Vector(this->i*scalar, this->j*scalar);
}

Vector Vector::relative_to(Vector vec2) const {
    return Vector(this->i-vec2.i, this->j-vec2.j);
}