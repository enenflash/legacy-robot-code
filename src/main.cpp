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
void print_botdata(BotData &bot_data, String message);

IRSensor ir_sensor;
LineSensor line_sensor;
PositionSystem pos_sys;

MotorController motor_ctrl(20);
DribblerMotor dribbler(DR_DIR, DR_PWM);
Adafruit_SSD1306 display(128, 32, &Wire, -1);
// Bluetooth bluetooth_comm;

PID movement_pid;

OneRobot one_robot_mode;
Defend defend_mode;
StayInLines stay_in_lines_mode;
GoToRobot go_to_robot;

uint8_t previous_mode = 0;
const int ATTACKER = 0;
const int DEFENDER = 1;

int loop_time = 0;
bool angle_correction = true;
bool robot_start = false;
Vector previous_line_vec;
Vector velocity(0, 0);

float time_start = millis();
float time_end = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(921600); // Line Sensor
  Serial6.begin(921600); // IR Sensor
  // bluetooth_comm.begin();

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
  /* -------------------------------------------------------------------------- */
  /*                              GET SENSOR VALUES                             */
  /* -------------------------------------------------------------------------- */

  pos_sys.update();
  ir_sensor.update();
  line_sensor.update();

  float heading = pos_sys.get_heading();
  Vector posv = pos_sys.get_posv();

  // angle correction
  float ball_angle = fmodf(PI + ir_sensor.get_angle() + heading, 2 * PI) - PI;
  float line_angle = fmodf(PI + line_sensor.get_angle() + heading, 2 * PI) - PI;

  BotData self_data = {
    .heading=heading, .pos_vector=posv,
    .ball_strength=ir_sensor.get_magnitude(), .ball_angle=ball_angle, 
    .line_vector=Vector::from_heading(line_angle, line_sensor.get_distance()),
    .velocity=velocity
  };

  if (self_data.ball_strength == 0) {
    self_data.ball_angle = 0;
  }

  /* -------------------------------------------------------------------------- */
  /*                           BLUETOOTH COMMUNICATION                          */
  /* -------------------------------------------------------------------------- */
  
  // send values
  // bluetooth_comm.send_data(self_data);
  line_sensor.send_bot_data(self_data);
  
  // receive values
  // BotData other_data = bluetooth_comm.read_data();
  BotData other_data = line_sensor.other_data;
  OutputData output;

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
  /*                       CHOOSE MODE AND GET OUTPUT DATA                      */
  /* -------------------------------------------------------------------------- */

  // if closer to the ball or if other robot doesn't detect the ball
  if (self_data.ball_strength > other_data.ball_strength && other_data.ball_strength != 0 || other_data.ball_strength == 0) {
    output = one_robot_mode.update(self_data, other_data, loop_time);
    previous_mode = ATTACKER;
  }
  else {
    if (previous_mode != DEFENDER) {
      defend_mode.calib_and_return.step = 0;
    }
    output = defend_mode.update(self_data, other_data, loop_time);
    previous_mode = DEFENDER;
  }

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

  // print data to serial
  // print_botdata(self_data);
  // print_botdata(other_data);
  // Serial.printf("received: %.2f loop_time: %d\n", line_sensor.angle, loop_time);
  Serial.println(line_angle * 180 / PI);
  // Serial.println(self_data.pos_vector.i)
  // Serial.print(" ");
  Serial.printf("coordinates: %.2f, %.2f \n", self_data.pos_vector.i, self_data.pos_vector.j);
  Serial.printf("rotation: %.2f \n", rotation * 180 / PI);

  // if (robot_start) {
  //   display.clearDisplay();
  //   display.setCursor(0, 0);
  //   // display.print("posv: ");
  //   // display.print(other_data.pos_vector.i);
  //   // display.print(" ");
  //   // display.print(other_data.pos_vector.j);
  //   // display.println();
  //   // display.print(loop_time);
  //   // display.print("x: "); display.println(pos_sys.get_posv().i);
  //   // display.print("y: "); display.println(pos_sys.get_posv().j);
  //   display.println(mv_angle * 180 / M_PI);
  //   display.display();
  // }

  previous_line_vec = self_data.line_vector;
  
  /* -------------------------------------------------------------------------- */
  /*                                 RUN MOTORS                                 */
  /* -------------------------------------------------------------------------- */

  if (dribbler_on) {
    dribbler.run();
  }
  else {
    dribbler.stop();
  }

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

void print_botdata(BotData &bot_data, String message) {
  Serial.print(message + " | heading:");
  Serial.print(bot_data.heading*180/M_PI);
  Serial.print(" posv:<");
  Serial.print(bot_data.pos_vector.i);
  Serial.print(",");
  Serial.print(bot_data.pos_vector.j);
  Serial.print("> strength:");
  Serial.print(bot_data.ball_strength);
  Serial.print(" IR angle:");
  Serial.print(bot_data.ball_angle*180/M_PI);
  Serial.print(" line angle:");
  Serial.print(bot_data.line_vector.heading()*180/M_PI);
  Serial.print(" line distance:");
  Serial.println(bot_data.line_vector.magnitude());
}