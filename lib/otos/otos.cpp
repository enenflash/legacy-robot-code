#include "otos.hpp"

void OTOS::set_up() {
    if (this->sparkfun_otos.begin()) { Serial.println("OTOS Initialized"); }
    else { Serial.println("No OTOs detected");}
    this->sparkfun_otos.calibrateImu();
    this->sparkfun_otos.setLinearUnit(sfe_otos_linear_unit_t(0));
    sfe_otos_pose2d_t offset = { 0.04, 0.04, 0 };
    this->sparkfun_otos.setOffset(offset);
    this->sparkfun_otos.resetTracking();
}

/* receives x and y in cm */
void OTOS::set_pos(float x, float y, float rotation) {
    sfe_otos_pose2d_t pos = {x/100, y/100, rotation};
    this->sparkfun_otos.setPosition(pos);
}

/* returns pos vector in cm */
Vector OTOS::get_posv() {
    sfe_otos_pose2d_t position;
    this->sparkfun_otos.getPosition(position);
    return Vector(position.x * 100, position.y * 100);
}