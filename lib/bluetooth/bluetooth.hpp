#ifndef _BLUETOOTH_HPP_
#define _BLUETOOTH_HPP_

#include "bot_data.h"
#include <cstring>
const uint8_t start_bytes[3] = {0xAA, 0xBB, 0xCC};

constexpr size_t total_bytes = sizeof(BotData) + 3; // 3 bytes for start bytes
class Bluetooth {
    private:
        bool check_bytes_valid(uint8_t* buffer);
        void struct_to_bytes(BotData data, uint8_t* buffer);
        bool bytes_to_struct(uint8_t* buffer, BotData& data);
    public:
        void send_data(BotData self_data);
        bool receive_data();
        BotData read_data();
};

#endif