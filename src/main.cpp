#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <iostream>
#include <cmath>
#include "pins.h"
#include "vector.hpp"
#include "motor_controller.hpp"
#include "position_system.hpp"
#include "ir_sensor.hpp"
#include "line_sensor.hpp"

// declarations here
void blinkLED();
float get_rotation(float heading);

PositionSystem pos_sys;

// 0.5 is how much the rotation is scaled compared to the movement
MotorController motor_ctrl(0.5);

IRSensor ir_sensor;
LineSensor line_sensor;

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

  // set up the buttons
  pinMode(BTN_1, INPUT_PULLDOWN);  // has to pull down
  pinMode(BTN_2, INPUT_PULLDOWN);
  pinMode(BTN_3, INPUT_PULLDOWN);
  pinMode(BTN_4, INPUT_PULLDOWN);
  pinMode(BTN_5, INPUT_PULLDOWN);

  pos_sys.setup(); // bno055
}

void loop() {
  Serial.println(".");
  pos_sys.update(); // call every loop (it reads all the sensors)
  // look at lib/position_system/position_system.hpp for all methods
  ir_sensor.update();
  line_sensor.update();

  float heading = pos_sys.get_heading(); // returns unit circle heading in radians

  if (angle_correction) {
    ir_sensor.angle_correction(heading);
    line_sensor.angle_correction(heading);
  }

  Vector posv = pos_sys.get_posv(); // note this is a custom class (uppercase) the cpp vector is lowercase
  // .display() returns std::string
  String posv_str = String(posv.display().c_str()); // must convert from std::string to String (arduino)
  
  // Serial.print(heading);
  // Serial.print(" ");
  // Serial.println(posv_str);

  Serial.print(ir_sensor.get_angle());
  Serial.print(" ");
  Serial.println(ir_sensor.get_magnitude());

  // Serial.print(line_sensor.get_angle());
  // Serial.print(" ");
  // Serial.println(line_sensor.get_distance());

  // angle_correction is 'rotation matrix'
  float mv_angle = PI/2;
  if (angle_correction) mv_angle -= heading*PI/180;

  motor_ctrl.run_motors(50, mv_angle, get_rotation(heading)); // run motors 50 speed, angle (radians), rotation
  // motor_ctrl.run_raw(-100, -100, 100, 100); // run motors raw
  // motor_ctrl.stop_motors(); // stop all motors
  digitalWrite(DEBUG_LED, HIGH);
}

// function definitions here
void blinkLED() { 
  digitalWrite(DEBUG_LED, HIGH);
  delay(100);
  digitalWrite(DEBUG_LED, LOW);
  delay(100);
}

// corrects robot heading
// heading in radians
float get_rotation(float heading) {
  float rotation = heading;
  if (rotation > PI) {
    rotation -= 2*PI;
  }
  rotation *= -1;
  return rotation;
}