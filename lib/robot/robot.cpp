#include "robot.hpp"

Robot::Robot(bool angle_correction) {
    this->angle_correction=angle_correction;
    MotorController motor_ctrl = MotorController(0.8);
    DribblerMotor dribbler = DribblerMotor(DR_DIR, DR_PWM);
}

void Robot::update() {
    // update sensors
    pos_sys.update();
    ir_sensor.update();
    line_sensor.update();

    float heading = pos_sys.get_heading();
    if (angle_correction) {
        ir_sensor.angle_correction(heading);
        line_sensor.angle_correction(heading);
    }

    // get sensor values
    Vector posv = pos_sys.get_posv(); // note this is a custom class (uppercase) the cpp vector is lowercase

    this->data = BotData { 
        .possession=false, .heading=heading, .pos_vector=posv, .opp_goal_vector=pos_sys.get_opp_goal_vec(),
        .ball_strength=ir_sensor.get_magnitude(), .ball_angle=ir_sensor.get_angle(), 
        .line_vector=Vector::from_heading(line_sensor.get_angle(), line_sensor.get_distance())
    };
}