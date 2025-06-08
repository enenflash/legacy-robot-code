#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include "bot_data.h"
#include "vector.hpp"
#include "motor_controller.hpp"
#include "position_system.hpp"
#include "ir_sensor.hpp"
#include "line_sensor.hpp"
#include "dribbler.hpp"

class Robot {
    public:
    bool angle_correction;
    PositionSystem pos_sys;
    MotorController motor_ctrl = MotorController(0.8);
    DribblerMotor dribbler = DribblerMotor(DR_DIR, DR_PWM);
    IRSensor ir_sensor;
    LineSensor line_sensor;
    BotData data;

    Robot(bool angle_correction=true);
    void update();
    BotData get_botdata();
};

#endif