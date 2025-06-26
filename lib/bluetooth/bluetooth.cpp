#include "bluetooth.hpp"
#include "bot_data.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(0, 1);

void Bluetooth::send_data(BotData self_data) {
    String full_data =  String(self_data.possession) + "," + String(self_data.heading) + "," + \
    String(self_data.pos_vector.i) + "," + String(self_data.pos_vector.j) + "," + \
    String(self_data.ball_strength) + "," + String(self_data.ball_angle) + ";";
    bluetooth.println(full_data);

}

bool Bluetooth::receive_data() {
    return bluetooth.available();
}

BotData Bluetooth::read_data() {
    int comma_index_1, comma_index_2, comma_index_3, comma_index_4, comma_index_5;
    BotData other_data;
    
    String data = bluetooth.readStringUntil(';');
    comma_index_1 = data.indexOf(",");
    comma_index_2 = data.indexOf(",", comma_index_1 + 1);
    comma_index_3 = data.indexOf(",", comma_index_2 + 1);
    comma_index_4 = data.indexOf(",", comma_index_3 + 1);
    comma_index_5 = data.indexOf(",", comma_index_4 + 1);

    other_data = {
        .possession = data.substring(0, comma_index_1).toInt(),
        .heading = data.substring(comma_index_1 + 1, comma_index_2).toFloat(),
        .pos_vector = Vector(data.substring(comma_index_2 + 1, comma_index_3).toFloat(), data.substring(comma_index_3 + 1, comma_index_4).toFloat()),
        .opp_goal_vector = Vector(0, 0),
        .ball_strength = data.substring(comma_index_4 + 1, comma_index_5).toFloat(),
        .ball_angle = data.substring(comma_index_5 + 1).toFloat(),
        .line_vector = Vector(0, 0)
    };

    return other_data;
}