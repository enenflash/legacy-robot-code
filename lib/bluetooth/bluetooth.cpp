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



void Bluetooth::send_data(BotData self_data) {
    // String full_data =  String(self_data.possession) + "," + String(self_data.heading) + "," + \
    // String(self_data.pos_vector.i) + "," + String(self_data.pos_vector.j) + "," + \
    // String(self_data.ball_strength) + "," + String(self_data.ball_angle) + ";";
    // bluetooth.println(full_data);
    uint8_t buffer[total_bytes];
    struct_to_bytes(self_data, buffer);
    bluetooth.write(buffer, total_bytes);
    bluetooth.flush();
}

bool Bluetooth::receive_data() {
    return bluetooth.available();
}

BotData Bluetooth::read_data() {
    // int comma_index_1, comma_index_2, comma_index_3, comma_index_4, comma_index_5;
    // BotData other_data;
    
    // String data = bluetooth.readStringUntil(';');
    // comma_index_1 = data.indexOf(",");
    // comma_index_2 = data.indexOf(",", comma_index_1 + 1);
    // comma_index_3 = data.indexOf(",", comma_index_2 + 1);
    // comma_index_4 = data.indexOf(",", comma_index_3 + 1);
    // comma_index_5 = data.indexOf(",", comma_index_4 + 1);

    // other_data = {
    //     .possession = data.substring(0, comma_index_1).toInt(),
    //     .heading = data.substring(comma_index_1 + 1, comma_index_2).toFloat(),
    //     .pos_vector = Vector(data.substring(comma_index_2 + 1, comma_index_3).toFloat(), data.substring(comma_index_3 + 1, comma_index_4).toFloat()),
    //     .opp_goal_vector = Vector(0, 0),
    //     .ball_strength = data.substring(comma_index_4 + 1, comma_index_5).toFloat(),
    //     .ball_angle = data.substring(comma_index_5 + 1).toFloat(),
    //     .line_vector = Vector(0, 0)
    // };

    // return other_data;
    BotData other_data;
    uint8_t buffer[total_bytes];
    uint8_t dummy[total_bytes];
    bluetooth.readBytesUntil(0xAA, dummy, total_bytes);

    buffer[0] = 0xAA;
    bluetooth.readBytes(buffer + 1, total_bytes - 1);
    bytes_to_struct(buffer, other_data);
    return other_data;
}