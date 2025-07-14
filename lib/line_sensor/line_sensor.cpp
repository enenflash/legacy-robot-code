#include "line_sensor.hpp"

bool LineSensor::read_serial(float* result, int num_floats) {
    const size_t total_bytes = num_floats * sizeof(float);
    const size_t max_buffer = 256;
    uint8_t buffer[max_buffer];
    int len = 0;

    // read all available bytes into a temp buffer
    while (Serial2.available() && len < max_buffer) {
        buffer[len++] = Serial2.read();
    }

    // find the last 'e' that has enough bytes after it
    int start_index = -1;
    for (int i = len - total_bytes - 1; i >= 0; i--) {
        if (buffer[i] == 'e' && (i + 1 + total_bytes <= len)) {
            start_index = i;
            break;
        }
    }

    if (start_index == -1) {
        // Serial.println("No full message found");
        return false;
    }

    // Step 3: Extract the float bytes
    const uint8_t* float_bytes = &buffer[start_index + 1];
    memcpy(result, float_bytes, total_bytes);

    return true;
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