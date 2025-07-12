#include "bluetooth.hpp"
#include "bot_data.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(0, 1);

bool Bluetooth::check_bytes_valid(uint8_t* buffer) {
    // Check if the first three bytes match the start bytes
    return (buffer[0] == start_bytes[0] && 
            buffer[1] == start_bytes[1] && 
            buffer[2] == start_bytes[2]);
}

void Bluetooth::struct_to_bytes(BotData data, uint8_t* buffer) {
    buffer[0] = start_bytes[0];
    buffer[1] = start_bytes[1];
    buffer[2] = start_bytes[2];
    std::memcpy(buffer+3, &data, total_bytes - 3); // Copy BotData struct to buffer after start bytes
}

bool Bluetooth::bytes_to_struct(uint8_t* buffer, BotData& data) {
    if (!check_bytes_valid(buffer)) {
        return false; // Invalid start bytes
    }
    std::memcpy(&data, buffer + 3, total_bytes - 3); // Copy bytes from buffer to BotData struct
    return true; // Successfully converted bytes to struct
}

void Bluetooth::begin() {
    SoftwareSerial bluetooth2(0, 1);
    bluetooth2.begin(38400);
}

void Bluetooth::send_data(BotData self_data) {
    uint8_t buffer[total_bytes];
    struct_to_bytes(self_data, buffer);
    bluetooth.write(buffer, total_bytes);
    bluetooth.flush();
}

BotData Bluetooth::read_data() {
    if (!bluetooth.available()) {
        return BotData {
            .heading = 0, .pos_vector = Vector(0, 0),
            .ball_strength = 0, .ball_angle = 0, .line_vector = Vector(0, 0)
        };
    }

    // return other_data;
    BotData other_data;
    uint8_t buffer[total_bytes];
    buffer[0] = 0xAA;
    uint8_t dummy_buffer[total_bytes];
    bluetooth.readBytesUntil(0xAA, dummy_buffer, total_bytes);

    float start_time = millis();
    while (bluetooth.available() < total_bytes - 1) {
        if (millis() - start_time <= 500) { continue; }
        Serial.println("time out");
        break;
    }
    int bytes_read = bluetooth.readBytes(buffer+1, total_bytes-1);

    if (bytes_read != total_bytes - 1) {
        Serial.println("Error: Not enough bytes read from Bluetooth.");
    }
    bytes_to_struct(buffer, other_data);
    return other_data;
}