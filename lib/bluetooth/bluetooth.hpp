#ifndef _BLUETOOTH_HPP_
#define _BLUETOOTH_HPP_

#include "bot_data.h"

class Bluetooth {
    void send_data(BotData self_data);
    BotData receive_data();
};

#endif