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

bool robot_move = false;
bool angle_correction = true;

int mode_select;
OrbitBall orbit_ball;
TargetGoalOTOS target_goal_otos;
Mode* mode_list[2] = {
  orbit_ball.get_pointer(),
  target_goal_otos.get_pointer()
};

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

  Serial.println("Awaiting button press");
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

  // get heading
  float heading = pos_sys.get_heading();

  // correct sensor angles
  if (angle_correction) {
    ir_sensor.angle_correction(heading);
    line_sensor.angle_correction(heading);
  }

  // get position vector
  Vector posv = pos_sys.get_posv(); // note this is a custom class (uppercase) the cpp vector is lowercase
  String posv_str = String(posv.display().c_str()); // must convert from std::string to String (arduino)

  // create bot data
  BotData self_data = BotData { 
    .possession=false, .heading=heading, .pos_vector=posv, .opp_goal_vector=pos_sys.get_opp_goal_vec(),
    .ball_strength=ir_sensor.get_magnitude(), .ball_angle=ir_sensor.get_angle(), 
    .line_vector=Vector::from_heading(line_sensor.get_angle(), line_sensor.get_distance())
  };

  // select the mode
  if (angle_correction) {
    // if ball directly in front
    if (self_data.ball_angle > heading-FORWARD_TOLERANCE && self_data.ball_angle < heading+FORWARD_TOLERANCE) {
      mode_select = TARGET_GOAL_OTOS;
    }
    // else get behind the ball
    else {
      mode_select = ORBIT_BALL;
    }
  }
  else {
    // if ball directly in front
    if (self_data.ball_angle > PI/2-FORWARD_TOLERANCE && self_data.ball_angle < PI/2+FORWARD_TOLERANCE) {
      mode_select = TARGET_GOAL_OTOS;
    }
    // else get behind the ball
    else {
      mode_select = ORBIT_BALL;
    }
  }

  // update mode
  mode_list[mode_select]->update(self_data);

  // get speed, rotation, movement angle and dribbler status
  float speed = mode_list[mode_select]->get_speed();
  float rotation = mode_list[mode_select]->get_rotation();
  float mv_angle = mode_list[mode_select]->get_angle();
  bool dribbler_on = mode_list[mode_select]->get_dribbler_on();

  // for debugging purposes
  Serial.print("Mode: "); Serial.print(mode_select);
  Serial.print(" speed: "); Serial.print(speed);
  Serial.print(" rotation: "); Serial.print(rotation*180/PI);
  Serial.print(" mv_angle: "); Serial.print(mv_angle*180/PI);
  Serial.print(" dribbler_on: "); Serial.println(dribbler_on);

  // run/stop dribbler
  if (dribbler_on) dribbler.run();
  else dribbler.stop();

  // run motors
  if (angle_correction) mv_angle -= heading;
  motor_ctrl.run_motors(speed, mv_angle, rotation);

  digitalWrite(DEBUG_LED, HIGH);
}

bool check_move() {
  if (digitalRead(BTN_1) == HIGH) {
    pos_sys.set_pos(Vector(0, -11.5), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_2) == HIGH) {
    pos_sys.set_pos(Vector(-42.5, -64), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_3) == HIGH) {
    pos_sys.set_pos(Vector(0, -64), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_4) == HIGH) {
    pos_sys.set_pos(Vector(42.5, -64), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_5) == HIGH) {
    pos_sys.set_pos(Vector(0, -84), 0); // set position of otos
    return true;
  }
  return false;
}