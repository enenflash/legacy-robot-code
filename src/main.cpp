#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <cmath>
#include <iostream>

#include "pins.h"
#include "vector.hpp"
#include "motor_controller.hpp"
#include "position_system.hpp"
#include "ir_sensor.hpp"
#include "line_sensor.hpp"
#include "dribbler.hpp"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

DribblerMotor dribbler = DribblerMotor(DR_DIR, DR_PWM);

bool check_robot_start();
float find_robot_start_angle(PositionSystem posv, Vector goal_pos, float tolerance, float ball_angle, float ball_magnitude);

Adafruit_SSD1306 display(128, 32, &Wire, -1);
PositionSystem pos_sys;

// 0.5 is how much the rotation is scaled compared to the robot_startment
MotorController motor_ctrl(0.8);

IRSensor ir_sensor;
LineSensor line_sensor;

bool angle_correction = true;
bool robot_start = false;

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
  pinMode(BTN_1, INPUT_PULLDOWN);
  pinMode(BTN_2, INPUT_PULLDOWN);
  pinMode(BTN_3, INPUT_PULLDOWN);
  pinMode(BTN_4, INPUT_PULLDOWN);
  pinMode(BTN_5, INPUT_PULLDOWN);

  pinMode(DR_PWM, OUTPUT);
  pinMode(DR_DIR, OUTPUT);

  pos_sys.setup(); // bno055

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(2);
  display.setTextSize(3);     
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);  

  display.setCursor(0, 0);   
  display.clearDisplay();
  display.println("Ready");
  display.display();

  Serial.println("Awaiting button press");
}

void loop() {
  Serial.println(".");
  pos_sys.update(); // call every loop (it reads all the sensors)
  // look at lib/position_system/position_system.hpp for all methods
  ir_sensor.update();
  line_sensor.update();

  float heading = pos_sys.get_heading(); // returns unit circle heading
  Vector posv = pos_sys.get_posv(); // note this is a custom class (uppercase) the cpp vector is lowercase
  // .display() returns std::string
  String posv_str = String(posv.display().c_str()); // must convert from std::string to String (arduino)

  if (!robot_start) {
    robot_start = check_robot_start();
  }

  float ball_angle = fmodf(PI + ir_sensor.get_angle() + heading, 2 * PI) - PI;
  float line_angle = fmodf(PI + line_sensor.get_angle() + heading, 2 * PI) - PI;

  Vector goal_vec = pos_sys.get_relative_to((Vector){91, 200});

  // convert unit circle heading to rotation
  float rotation = goal_vec.heading() - heading - PI/2; // convert to degrees
  while (rotation > PI) rotation -= 2*PI;
  while (rotation < -PI) rotation += 2*PI;

  // angle_correction is 'rotation matrix'
  float mv_angle = 0;
  mv_angle = find_robot_start_angle(pos_sys, (Vector){91, 180}, FORWARD_TOLERANCE, ball_angle, ir_sensor.get_magnitude());

  if (line_sensor.get_distance() != 0) {
    mv_angle = (line_angle) + PI;
  }

  float speed = 100;

  if ((ir_sensor.get_magnitude() == 0 && line_sensor.get_distance() == 0) || !robot_start) {
    speed = 0;
    dribbler.stop();
  }
  else {
    dribbler.run();
  }

  // Serial.println(mv_angle*180/PI);
  if (angle_correction) mv_angle -= heading;

  Serial.print(line_sensor.get_distance());
  Serial.print(" ");
  Serial.print(line_sensor.get_angle() * 180 / PI);
  Serial.print(" ");
  Serial.print(ir_sensor.get_angle() * 180 / PI);
  Serial.print(" ");
  Serial.print(ir_sensor.get_magnitude());
  Serial.print(" ");
  Serial.print(posv.i);
  Serial.print(" ");
  Serial.print(posv.j); 
  Serial.print(" ");
  Serial.print(mv_angle * 180 / PI);
  Serial.print(" ");
  Serial.print(rotation * 180 / PI);

  motor_ctrl.run_motors(speed, mv_angle, rotation); // run motors 50 speed, angle (radians), rotation

  digitalWrite(DEBUG_LED, HIGH);
}

bool check_robot_start() {
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

float find_robot_start_angle(PositionSystem posv, Vector goal_pos, float tolerance, float ball_angle, float ball_magnitude) {
  Vector goal_vec = posv.get_relative_to(goal_pos);
  float angle_diff = PI / 2 - goal_vec.heading();
  if (ball_magnitude < 40) {
    dribbler.stop();
    return ball_angle;
  }
  if (ball_angle > goal_vec.heading() - tolerance && ball_angle < goal_vec.heading() + tolerance) {
    // return goal_vec.heading();
    // float current_i = posv.get_posv().i;
    // float current_j = posv.get_posv().j;
    // if (current_i > goal_pos.i - 20 && current_i < goal_pos.i + 20 && current_j> goal_pos.j - 20) {
    //   dribbler.stop(); // stop dribbler if close to goal
    //   return 0; // robot_start forward
    // }
    dribbler.run(); // run dribbler
    return goal_vec.heading(); // robot_start forward
  }
  else if ((ball_angle > goal_vec.heading() + tolerance) || (ball_angle < -PI / 2 + angle_diff)) {
    dribbler.stop();
    return ball_angle + PI / 18 * 6; // turn right
  }
  else if ((ball_angle < goal_vec.heading() - tolerance)) {
    dribbler.stop();
    return ball_angle - PI / 18 * 6; // turn left
  }
}