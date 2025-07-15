#include "line_sensor.hpp"

bool LineSensor::read_serial(float* result) {
    const size_t max_buffer = 256;
    uint8_t buffer[max_buffer];
    int len = 0;

    // read all available bytes into a temp buffer
    while (Serial2.available() && len < max_buffer) {
        buffer[len++] = Serial2.read();
    }

    // find the last 'e' that has enough bytes after it
    int start_index = -1;
    for (int i = len - TOTAL_BYTES - 1; i >= 0; i--) {
        if (buffer[i] == 'e' && (i + 1 + TOTAL_BYTES <= len)) {
            start_index = i;
            break;
        }
    }

    if (start_index == -1) {
        // Serial.println("No full message found");
        return false;
    }

    // Step 3: Extract the float bytes
    const uint8_t* received_bytes = &buffer[start_index + 1];
    memcpy(result, received_bytes, TOTAL_BYTES);

    return true;
}
void LineSensor::send_bot_data(BotData self_data) {
    uint8_t buffer[sizeof(BotData) + 1];
    buffer[0] = 'd';
    memcpy(&buffer[1], &self_data, sizeof(BotData));
    Serial2.write(buffer, sizeof(BotData) + 1);
}
void LineSensor::update() {
    float data[TOTAL_BYTES];
    this->read_success = this->read_serial(data);
    if (this->read_success) {
        this->angle = data[0];
        this->distance = data[1];
        memcpy(&this->other_data, &data[2], sizeof(BotData));
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