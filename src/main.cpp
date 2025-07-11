#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <cmath>
#include <iostream>
#include <SoftwareSerial.h>

#include "pins.h"
#include "vector.hpp"
#include "motor_controller.hpp"
#include "position_system.hpp"
#include "ir_sensor.hpp"
#include "line_sensor.hpp"
#include "dribbler.hpp"
#include "mode.hpp"
#include "bluetooth.hpp"
#include "pid.hpp"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

bool check_robot_start();
float find_move_angle(Vector goal_vec, float ball_angle, float ball_magnitude);

IRSensor ir_sensor;
LineSensor line_sensor;
PositionSystem pos_sys;

MotorController motor_ctrl(0.8);
DribblerMotor dribbler(DR_DIR, DR_PWM);
Adafruit_SSD1306 display(128, 32, &Wire, -1);

PID linear_pid;
Bluetooth bluetooth_comm;

ShingGetBehindBall shing_mode;

int loop_time = 0;
bool angle_correction = true;
bool robot_start = false;

float time_start = millis();
float time_end = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200); // Line Sensor
  Serial6.begin(115200); // IR Sensor
  bluetooth_comm.begin();

  pinMode(DEBUG_LED, OUTPUT);

  // Drive motors
  pinMode(TL_PWM, OUTPUT); pinMode(TR_PWM, OUTPUT); pinMode(BL_PWM, OUTPUT); pinMode(BR_PWM, OUTPUT);
  analogWriteFrequency(TL_PWM, 20000); analogWriteFrequency(TR_PWM, 20000);
  analogWriteFrequency(BL_PWM, 20000); analogWriteFrequency(BR_PWM, 20000);
  pinMode(TL_DIR, OUTPUT); pinMode(TR_DIR, OUTPUT); pinMode(BL_DIR, OUTPUT); pinMode(BR_DIR, OUTPUT);

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

  // Display Setup
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(2);     
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);  

  // Displaying Ready and compile date/time
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);   
  display.println("Ready");

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(__DATE__); display.print(" "); display.println(__TIME__);
  display.display();

  Serial.println("Awaiting button press");
}

void loop() {
  pos_sys.update();
  ir_sensor.update();
  line_sensor.update();

  float heading = pos_sys.get_heading();
  Vector posv = pos_sys.get_posv();

  // check for button press
  if (!robot_start) {
    robot_start = check_robot_start();
  }

  // angle correction
  float ball_angle = fmodf(PI + ir_sensor.get_angle() + heading, 2 * PI) - PI;
  float line_angle = fmodf(PI + line_sensor.get_angle() + heading, 2 * PI) - PI;

  Vector goal_vec = pos_sys.get_relative_to(Vector(0, 78.5));

  // convert unit circle heading to rotation
  float rotation = goal_vec.heading() - heading - PI/2; // convert to degrees
  while (rotation > PI) rotation -= 2*PI;
  while (rotation < -PI) rotation += 2*PI;

  // construct self_data
  BotData self_data = {
    .possession=false, .heading=heading, .pos_vector=posv, .opp_goal_vector=goal_vec, 
    .ball_strength=ir_sensor.get_magnitude(), .ball_angle=ball_angle, 
    .line_vector=Vector::from_heading(line_angle, line_sensor.get_distance())
  };

  // bluetooth communication
  // bluetooth_comm.send_data(self_data);
  // BotData other_data = bluetooth_comm.read_data();
  
  // GET STUFF
  // find movement angle
  float mv_angle = 0;
  mv_angle = find_move_angle(pos_sys.get_relative_to(Vector(0, 58.5)), ball_angle, ir_sensor.get_magnitude());

  if (line_sensor.get_distance() != 0) {
    mv_angle = (line_angle) + PI;
  }

  float speed = MAX_SPEED;

  if ((ir_sensor.get_magnitude() == 0 && line_sensor.get_distance() == 0) || !robot_start) {
    speed = 0;
    dribbler.stop();
  }
  else {
    dribbler.run();
  }
  // END GET STUFF

  // loop time
  time_end=millis();
  loop_time = time_end - time_start;
  time_start=millis();

  // print data to serial
  // Serial.print(line_sensor.get_distance());
  // Serial.print(" ");
  // Serial.print(line_sensor.get_angle() * 180 / PI);
  // Serial.print(" ");
  // Serial.print(ir_sensor.get_angle() * 180 / PI);
  // Serial.print(" ");
  // Serial.print(ir_sensor.get_magnitude());
  // Serial.print(" ");
  // Serial.print(posv.i);
  // Serial.print(" ");
  // Serial.print(posv.j); 
  // Serial.print(" ");
  // Serial.print(mv_angle * 180 / PI);
  // Serial.print(" ");
  // Serial.println(rotation * 180 / PI);

  // display values on oled
  // if (robot_start) {
  //   display.clearDisplay();
  //   display.setCursor(0, 0);
  //   // display.print("posv: ");
  //   // display.print(other_data.pos_vector.i);
  //   // display.print(" ");
  //   // display.print(other_data.pos_vector.j);
  //   // display.println();
  //   // display.print(loop_time);
  //   display.print("x: "); display.println(pos_sys.get_posv().i);
  //   display.print("y: "); display.println(pos_sys.get_posv().j);
  //   display.display();
  // }
  
  // run motors
  if (angle_correction) mv_angle -= heading;
  motor_ctrl.run_motors(speed, mv_angle, rotation);
  digitalWrite(DEBUG_LED, HIGH);
}

bool check_robot_start() {
  if (digitalRead(BTN_1) == HIGH) {
    pos_sys.set_pos(Vector(0, -11.5), 0); // set position of otos (kick off)
    return true;
  }
  if (digitalRead(BTN_2) == HIGH) {
    pos_sys.set_pos(Vector(-35, -73), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_3) == HIGH) {
    pos_sys.set_pos(Vector(0, -61.5), 0); // set position of otos (center front)
    return true;
  }
  if (digitalRead(BTN_4) == HIGH) {
    pos_sys.set_pos(Vector(35, -73), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_5) == HIGH) {
    pos_sys.set_pos(Vector(0, -81.5), 0); // set position of otos (center back)
    return true;
  }
  return false;
}

float find_move_angle(Vector goal_vec, float ball_angle, float ball_magnitude) {
  float angle_diff = PI / 2 - goal_vec.heading();
  if (ball_magnitude < 40) {
    dribbler.stop();
    return ball_angle;
  }
  if (ball_angle > goal_vec.heading() - FORWARD_TOLERANCE && ball_angle < goal_vec.heading() + FORWARD_TOLERANCE) {
    dribbler.run(); // run dribbler
    return goal_vec.heading(); // robot move forward
  }
  else if ((ball_angle > goal_vec.heading() + FORWARD_TOLERANCE) || (ball_angle < -PI / 2 + angle_diff)) {
    dribbler.stop();
    return ball_angle + PI / 18 * 6; // turn right
  }
  else if ((ball_angle < goal_vec.heading() - FORWARD_TOLERANCE)) {
    dribbler.stop();
    return ball_angle - PI / 18 * 6; // turn left
  }
}