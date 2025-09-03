#include "camera.hpp"

bool Camera::read_serial(int* result, int num_ints) {
    const size_t total_bytes = num_ints * sizeof(int);
    const size_t max_buffer = 256;
    uint8_t buffer[max_buffer];
    int len = 0;
    if (Serial8.available() < total_bytes + 1) {
        return false;
    }
    // read all available bytes into a temp buffer
    while (Serial8.available() && len < max_buffer) {
        buffer[len++] = Serial8.read();
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
        Serial.println("No full message found");
        return false;
    }

    // Extract the int bytes
    const uint8_t* int_bytes = &buffer[start_index + 1];
    memcpy(result, int_bytes, total_bytes);

    return true;
}

void Camera::update() {
    int data[4];
    this->read_success = this->read_serial(data, 4);
    if (this->read_success) {
        this->blue_x = data[0];
        this->blue_y = data[1];
        this->yellow_x = data[2];
        this->yellow_y = data[3];
    }
}