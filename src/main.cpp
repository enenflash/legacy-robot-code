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
#include "camera.hpp"
#include "compute.hpp"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

bool check_robot_start();
void print_botdata(BotData &bot_data, String message);

IRSensor ir_sensor;
LineSensor line_sensor;
Camera camera;
PositionSystem pos_sys;

MotorController motor_ctrl(20);
DribblerMotor dribbler(DR_DIR, DR_PWM);
Adafruit_SSD1306 display(128, 32, &Wire, -1);

// Modes
OneRobot one_robot_mode;
BetterDefend better_defend_mode;
uint8_t previous_mode = 0;

#define ATTACKER 0
#define DEFENDER 1
#define YELLOW 0
#define BLUE 1

int current_goal = YELLOW;
int loop_time = 0;
bool robot_start = false;
Vector velocity(0, 0);

float time_start = millis();
float time_end = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);
  Serial2.begin(921600); // Line Sensor
  Serial6.begin(921600); // IR Sensor
  Serial8.begin(230400); // Camera

  pinMode(DEBUG_LED, OUTPUT);

  // Drive motors
  pinMode(TL_PWM, OUTPUT); pinMode(TR_PWM, OUTPUT); pinMode(BL_PWM, OUTPUT); pinMode(BR_PWM, OUTPUT);
  analogWriteFrequency(TL_PWM, 20000); analogWriteFrequency(TR_PWM, 20000);
  analogWriteFrequency(BL_PWM, 20000); analogWriteFrequency(BR_PWM, 20000);
  pinMode(TL_DIR, OUTPUT); pinMode(TR_DIR, OUTPUT); pinMode(BL_DIR, OUTPUT); pinMode(BR_DIR, OUTPUT);

  motor_ctrl.stop_motors();

  // Start buttons
  pinMode(BTN_1, INPUT_PULLDOWN);
  pinMode(BTN_2, INPUT_PULLDOWN);
  pinMode(BTN_3, INPUT_PULLDOWN);
  pinMode(BTN_4, INPUT_PULLDOWN);
  pinMode(BTN_5, INPUT_PULLDOWN);

  // Dribbler pins
  pinMode(DR_PWM, OUTPUT);
  pinMode(DR_DIR, OUTPUT);

  pos_sys.setup(); // initialise bno055

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
  /* -------------------------------------------------------------------------- */
  /*                              GET SENSOR VALUES                             */
  /* -------------------------------------------------------------------------- */

  pos_sys.update();
  ir_sensor.update();
  line_sensor.update();
  camera.update();

  float heading = pos_sys.get_heading();
  Vector posv = pos_sys.get_posv();

  // angle correction (0-PI)
  ir_sensor.angle_correction(heading);
  line_sensor.angle_correction(heading);

  // Constructs robot data from sensor values
  BotData self_data = {
    .heading=heading, .pos_vector=posv,
    .ball_strength=ir_sensor.get_magnitude(), .ball_angle=ir_sensor.get_angle(), 
    .line_vector=Vector::from_heading(line_sensor.get_angle(), line_sensor.get_distance()),
    .velocity=velocity, .goal_x=-1,
  };

  if (current_goal == YELLOW) self_data.goal_x=camera.yellow_x;
  else self_data.goal_x=camera.blue_x;

  if (self_data.ball_strength == 0) {
    self_data.ball_angle = 0;
  }

  /* -------------------------------------------------------------------------- */
  /*                           BLUETOOTH COMMUNICATION                          */
  /* -------------------------------------------------------------------------- */
  
  line_sensor.send_bot_data(self_data); // send values
  BotData other_data = line_sensor.other_data; // receive values

  /* -------------------------------------------------------------------------- */
  /*                            LINE CORRECTION TEST                            */
  /* -------------------------------------------------------------------------- */

  // if (self_data.line_vector.magnitude() == 0 && previous_line_vec.magnitude() && previous_line_vec.heading() < M_PI / 4 && previous_line_vec.heading() > -M_PI / 4) {
  //   pos_sys.set_pos((Vector){40, self_data.pos_vector.j}, self_data.heading * 180 / M_PI);
  //   Serial.printf("set right \n");
  // }
  // if (self_data.line_vector.magnitude() == 0 && previous_line_vec.magnitude() && (previous_line_vec.heading() > 3 * M_PI / 4 || previous_line_vec.heading() < - 3 * M_PI / 4)) {
  //   pos_sys.set_pos((Vector){-40, self_data.pos_vector.j}, self_data.heading * 180 / M_PI);
  //   Serial.printf("set left \n");
  // }
  // if (self_data.line_vector.magnitude() == 0 && previous_line_vec.magnitude() && previous_line_vec.heading() > -3 * M_PI / 4 && previous_line_vec.heading() < -M_PI / 4) {
  //   pos_sys.set_pos((Vector){self_data.pos_vector.i, -85}, self_data.heading * 180 / M_PI);
  //   Serial.printf("set back \n");
  // }
  // if (self_data.line_vector.magnitude() == 0 && previous_line_vec.magnitude() && previous_line_vec.heading() < 3 * M_PI / 4 && previous_line_vec.heading() > 1 * M_PI / 4) {
  //   pos_sys.set_pos((Vector){self_data.pos_vector.i, 70}, self_data.heading * 180 / M_PI);
  //   Serial.printf("set forward \n");
  // }

  /* -------------------------------------------------------------------------- */
  /*                            OTOS CALIBRATION CODE                           */
  /* -------------------------------------------------------------------------- */

  // if (self_data.line_vector.magnitude() != 0) {
    /* ------------------------ Calibrating X-Coordinate ------------------------ */

    // using 0-PI rangegf
    // Left
    // if (self_data.line_vector.heading() >= 3*PI/4 || self_data.line_vector.heading() <= 5*PI/4) {
    //   pos_sys.set_pos(Vector(-(FIELD_WIDTH/2 - 25 - 2.5 - self_data.line_vector.magnitude()), self_data.pos_vector.j), Compute::unit_angle_to_bearing(heading));
    // }

    // // Right
    // if (self_data.line_vector.heading() <= PI/4 || self_data.line_vector.heading() >= 7*PI/4) {
    //   pos_sys.set_pos(Vector(FIELD_WIDTH/2 - 2.5 - 25 - self_data.line_vector.magnitude(), self_data.pos_vector.j), Compute::unit_angle_to_bearing(heading));
    // }
    // /* ------------------------ Calibrating Y-Coordinate ------------------------ */

    // Front
    // if (self_data.line_vector.heading() >= PI/4 && self_data.line_vector.heading() <= 3*PI/4) {
    //   pos_sys.set_pos(Vector(self_data.pos_vector.i, FIELD_LENGTH/2 - 25 - 2.5 - self_data.line_vector.magnitude()), Compute::unit_angle_to_bearing(heading));
    // }

    // // Back
    // if (self_data.line_vector.heading() <= 7*PI/4 && self_data.line_vector.heading() >= 5*PI/4) {
    //   pos_sys.set_pos(Vector(self_data.pos_vector.i, -(FIELD_LENGTH/2 - 25 -2.5 - self_data.line_vector.magnitude())), Compute::unit_angle_to_bearing(heading));
    // }
  // }

  /* -------------------------------------------------------------------------- */
  /*                       CHOOSE MODE AND GET OUTPUT DATA                      */
  /* -------------------------------------------------------------------------- */

  OutputData output;

  // if closer to the ball or if other robot doesn't detect the ball then become the attacker
  if (self_data.ball_strength > other_data.ball_strength && other_data.ball_strength != 0 || other_data.ball_strength == 0) {
    output = one_robot_mode.update(self_data, other_data, loop_time);
    previous_mode = ATTACKER;
  }
  // else become the defender
  else {
    if (previous_mode != DEFENDER) {
      better_defend_mode.reset();
    }
    output = better_defend_mode.update(self_data, other_data, loop_time);
    previous_mode = DEFENDER;
  }
  // output = better_defend_mode.update(self_data, other_data, loop_time);
  
  float mv_angle = output.angle;
  float speed = output.speed;
  float rotation = output.rotation;
  bool dribbler_on = output.dribbler_on;
  velocity = Vector::from_heading(mv_angle, speed);

  /* -------------------------------------------------------------------------- */
  
  // check for button press
  if (!robot_start) {
    speed = 0;
    dribbler_on = false;
    robot_start = check_robot_start();
  }

  // determine loop time
  time_end=millis();
  loop_time = time_end - time_start;
  time_start=millis();

  /* -------------------------------------------------------------------------- */
  /*                       OUTPUT TO SERIAL MONITOR / OLED                      */
  /* -------------------------------------------------------------------------- */
  
  // SERIAL PRINTING
  // print_botdata(self_data, "");
  // print_botdata(other_data, "");
  // Serial.printf("received: %.2f loop_time: %d\n", line_sensor.angle, loop_time);
  Serial.println(self_data.pos_vector.j);
  // Serial.print(" ");
  // Serial.printf("coordinates: %.2f, %.2f \n", self_data.pos_vector.i, self_data.pos_vector.j);
  // Serial.printf("rotation: %.2f \n", rotation * 180 / PI);
  // Serial.printf("ir_strength: %.2f", self_data.ball_strength);
  
  // OLED PRINTING
  // if (robot_start) {
  //   display.clearDisplay();
  //   display.setCursor(0, 0);
  //   display.print("Hello, World!");
  //   display.display();
  // }
  
  /* -------------------------------------------------------------------------- */
  /*                                 RUN MOTORS                                 */
  /* -------------------------------------------------------------------------- */

  if (dribbler_on && self_data.pos_vector.j < SLOW_DOWN_DIST-5) {
    dribbler.run();
  }
  else if (dribbler_on && self_data.pos_vector.j >= SLOW_DOWN_DIST-5 && mv_angle > 0 && mv_angle < PI && RELEASE_BALL) {
    dribbler.run_reverse();
    if (self_data.pos_vector.j >= SLOW_DOWN_DIST) speed = RELEASE_SPEED;
  }
  else {
    dribbler.stop();
  }

  mv_angle -= heading;
  motor_ctrl.run_motors(speed, mv_angle, rotation);
  if (!robot_start) digitalWrite(DEBUG_LED, HIGH);
}

bool check_robot_start() {
  if (digitalRead(BTN_1) == HIGH) {
    pos_sys.set_pos(Vector(0, -11.5), 0); // set position of otos (kick off)
    return true;
  }
  if (digitalRead(BTN_2) == HIGH) {
    pos_sys.set_pos(Vector(-41, -69.5), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_3) == HIGH) {
    pos_sys.set_pos(Vector(0, -64.0), 0); // set position of otos (center front)
    return true;
  }
  if (digitalRead(BTN_4) == HIGH) {
    pos_sys.set_pos(Vector(41, -69.5), 0); // set position of otos
    return true;
  }
  if (digitalRead(BTN_5) == HIGH) {
    pos_sys.set_pos(Vector(0, -87.5), 0); // set position of otos (center back)
    return true;
  }
  return false;
}

void print_botdata(BotData &bot_data, String message) {
  Serial.print(message + " | heading:");
  Serial.print(bot_data.heading*180/PI);
  Serial.print(" posv:<");
  Serial.print(bot_data.pos_vector.i);
  Serial.print(",");
  Serial.print(bot_data.pos_vector.j);
  Serial.print("> strength:");
  Serial.print(bot_data.ball_strength);
  Serial.print(" IR angle:");
  Serial.print(bot_data.ball_angle*180/PI);
  Serial.print(" line angle:");
  Serial.print(bot_data.line_vector.heading()*180/PI);
  Serial.print(" line distance:");
  Serial.println(bot_data.line_vector.magnitude());
}