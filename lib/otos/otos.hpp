#ifndef _OTOS_HPP_
#define _OTOS_HPP_

#pragma once

#include <iostream>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include "vector.hpp"

class OTOS {
    private:
    QwiicOTOS sparkfun_otos;
    public:
    void set_up();
    void set_pos(float x, float y, float rotation);
    /* return position vector of robot */
    Vector get_posv();
};

#endif