#include "ir_sensor.hpp"

// Read data from IR sensor using UART
bool IRSensor::read_serial(float* result, int num_floats) {
    const size_t total_bytes = num_floats * sizeof(float);
    const size_t max_buffer = 256;
    uint8_t buffer[max_buffer];
    int len = 0;

    // read all available bytes into a temp buffer
    while (Serial6.available() && len < max_buffer) {
        buffer[len++] = Serial6.read();
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

    // Extract the float bytes
    const uint8_t* float_bytes = &buffer[start_index + 1];
    memcpy(result, float_bytes, total_bytes);

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

// Calculates the true angle of ball relative to the field not the robot (heading in radians)
void IRSensor::angle_correction(float heading) {
    this->angle = fmodf(this->angle + heading, 2*PI);
}

// Returns strength reading
float IRSensor::get_magnitude() {
    return this->magnitude;
}

// Returns angle in radians
float IRSensor::get_angle() {
    return this->angle;
}