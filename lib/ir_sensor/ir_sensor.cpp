#include "ir_sensor.hpp"

bool IRSensor::read_serial(float* result, int num_floats) {
    const size_t total_bytes = num_floats * sizeof(float);
    if (!Serial6.available()) {
        return false;
    }
    // Wait for 'e' header
    char header = Serial6.read();
    if (header != 'e') {
        return false;
    }
    // Wait until all float bytes are available
    unsigned long start = millis();
    while (Serial6.available() < total_bytes) {
        if (millis() - start > 100) return false; // timeout
    }
    byte* byte_ptr = (byte*)result;
    for (size_t i = 0; i < total_bytes; i++) {
        byte_ptr[i] = Serial6.read();
    }
    return true;
}

void IRSensor::update() {
    float data[4];
    this->read_success = this->read_serial(data, 4);
    if (this->read_success) {
        this->angle = data[0];
        this->magnitude = data[1];
    }
}

// heading in radians
void IRSensor::angle_correction(float heading) {
    this->angle = fmodf(this->angle + heading, 2*PI);
}

float IRSensor::get_magnitude() {
    return this->magnitude;
}

float IRSensor::get_angle() {
    return this->angle;
}