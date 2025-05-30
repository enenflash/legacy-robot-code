#include "position_system.hpp"

PositionSystem::PositionSystem(bool use_otos, bool use_ult) {
    this->bno = Adafruit_BNO055(55);
    this->bno_ok = false;

    this->use_otos = use_otos;
    this->use_ult = use_ult;
    
    this->opp_goal_posv = Vector(0, 91.5);
    this->own_goal_posv = Vector(0, -91.5);
    this->posv = Vector(0, 0);
    this->heading = 0;
}

bool PositionSystem::check_bno_ok() {
    return this->bno_ok;
}

Vector PositionSystem::get_posv() {
    return this->posv;
}
float PositionSystem::get_heading() {
    return this->heading;
}

Vector PositionSystem::get_relative_to(Vector other_posv) {
    return Vector(other_posv.i-this->posv.i, other_posv.j-this->posv.j);
}

Vector PositionSystem::get_opp_goal_vec() {
    return this->get_relative_to(this->opp_goal_posv);
}
Vector PositionSystem::get_own_goal_vec() {
    return this->get_relative_to(this->own_goal_posv);
}

void PositionSystem::set_pos(Vector posv, float heading) {
    this->otos.set_pos(posv.i, posv.j, heading);
}

void PositionSystem::setup() {
    this->bno_ok = this->bno.begin();
    this->bno.setExtCrystalUse(true);
    this->otos.set_up();
}

void PositionSystem::update() {
    // get tilt from BNO or get tilt from parameter
    sensors_event_t event;
    this->bno.getEvent(&event);
    this->heading = (360-event.orientation.x)*PI/180;

    // could potentially implement a method that uses both ultrasonics and otos
    if (this->use_ult) {
        this->ult_ps.update(this->heading);
        this->posv = this->ult_ps.get_posv();
    }
    if (this->use_otos) {
        this->posv = this->otos.get_posv();
    }
}