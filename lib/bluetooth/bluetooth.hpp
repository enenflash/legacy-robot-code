#ifndef _BLUETOOTH_HPP_
#define _BLUETOOTH_HPP_

#include "bot_data.h"

class Bluetooth {
    public:
        void send_data(BotData self_data);
        bool receive_data();
        BotData read_data();
};

#endif