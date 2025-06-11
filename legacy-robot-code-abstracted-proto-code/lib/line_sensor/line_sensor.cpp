#include "line_sensor.hpp"

bool LineSensor::read_serial(float* result, int num_floats) {
    const size_t total_bytes = num_floats * sizeof(float);
    if (!Serial2.available()) {
        return false;
    }
    // Wait for 'e' header
    char header = Serial2.read();
    if (header != 'e') {
        return false;
    }
    // Wait until all float bytes are available
    unsigned long start = millis();
    while (Serial2.available() < total_bytes) {
        if (millis() - start > 100) return false; // timeout
    }

    byte* byte_ptr = (byte*)result;
    for (size_t i = 0; i < total_bytes; i++) {
        byte_ptr[i] = Serial2.read();
    }
    return true; // success
}

void LineSensor::update() {
    float data[2];
    this->read_success = this->read_serial(data, 2);
    if (this->read_success) {
        this->angle = data[0];
        this->distance = data[1];
    }
}

// heading in radians
void LineSensor::angle_correction(float heading) {
    this->angle = fmodf(this->angle + heading, 2*PI);
}

float LineSensor::get_distance() {
    return std::round(this->distance*1000)/1000;
}

float LineSensor::get_angle() {
    return std::round(this->angle*1000)/1000;
}