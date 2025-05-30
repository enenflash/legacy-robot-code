#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <cmath>
#include <iostream>

#include "pins.h"
#include "constants.h"
#include "bot_data.h"

#include "vector.hpp"
#include "motor_controller.hpp"
#include "position_system.hpp"
#include "ir_sensor.hpp"
#include "line_sensor.hpp"
#include "dribbler.hpp"
#include "mode.hpp"

// declarations here
bool check_move();

PositionSystem pos_sys;
MotorController motor_ctrl(0.8);
DribblerMotor dribbler = DribblerMotor(DR_DIR, DR_PWM);

IRSensor ir_sensor;
LineSensor line_sensor;

ShingGetBehindBall current_mode;

bool robot_move = false;
bool angle_correction = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(115200); // Line Sensor
  Serial6.begin(115200); // IR Sensor

  pinMode(DEBUG_LED, OUTPUT);

  // Motors
  pinMode(TL_PWM, OUTPUT);
  pinMode(TR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);
  analogWriteFrequency(TL_PWM, 20000);
  analogWriteFrequency(TR_PWM, 20000);
  analogWriteFrequency(BL_PWM, 20000);
  analogWriteFrequency(BR_PWM, 20000);
  pinMode(TL_DIR, OUTPUT);
  pinMode(TR_DIR, OUTPUT);
  pinMode(BL_DIR, OUTPUT);
  pinMode(BR_DIR, OUTPUT);
  motor_ctrl.stop_motors();

  // Ultrasonics (default no power)
  pinMode(UL_TRIG, OUTPUT);
  pinMode(UR_TRIG, OUTPUT);
  pinMode(UB_TRIG, OUTPUT);
  pinMode(UL_ECHO, INPUT);
  pinMode(UR_ECHO, INPUT);
  pinMode(UB_ECHO, INPUT);

  // Buttons
  pinMode(BTN_1, INPUT_PULLDOWN);
  pinMode(BTN_2, INPUT_PULLDOWN);
  pinMode(BTN_3, INPUT_PULLDOWN);
  pinMode(BTN_4, INPUT_PULLDOWN);
  pinMode(BTN_5, INPUT_PULLDOWN);

  // Dribbler
  pinMode(DR_PWM, OUTPUT);
  pinMode(DR_DIR, OUTPUT);

  pos_sys.setup(); // bno055
}

void loop() {
  // wait for button press
  if (!robot_move) {
    motor_ctrl.stop_motors();
    dribbler.stop();
    robot_move = check_move();
    return;
  }

  Serial.println(".");

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
  String posv_str = String(posv.display().c_str()); // must convert from std::string to String (arduino)

  BotData self_data = BotData { 
    .possession=false, .heading=heading, .pos_vector=posv, .opp_goal_vector=pos_sys.get_opp_goal_vec(),
    .ball_strength=ir_sensor.get_magnitude(), .ball_angle=ir_sensor.get_angle(), 
    .line_vector=Vector::from_heading(line_sensor.get_angle(), line_sensor.get_distance())
  };

  current_mode.update(self_data);
  
  float speed = current_mode.get_speed();
  float rotation = current_mode.get_rotation();
  float mv_angle = current_mode.get_angle();
  bool dribbler_on = current_mode.get_dribbler_on();

  if (dribbler_on) dribbler.run();
  else dribbler.stop();

  if (angle_correction) mv_angle -= heading*PI/180;
  Serial.println(rotation);
  motor_ctrl.run_motors(speed, mv_angle, rotation*180/PI);

  digitalWrite(DEBUG_LED, HIGH);
}

bool check_move() {
  if (digitalRead(BTN_1) == HIGH) {
    pos_sys.set_pos(Vector(91, 110), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_2) == HIGH) {
    pos_sys.set_pos(Vector(48.5, 57.5), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_3) == HIGH) {
    pos_sys.set_pos(Vector(91, 57.5), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_4) == HIGH) {
    pos_sys.set_pos(Vector(133.5, 57.5), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_5) == HIGH) {
    pos_sys.set_pos(Vector(91, 37.5), 0); // set position of otos
    return true;
  }
  return false;
}