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

const int DRIBBLER_SPEED = 100;
class DribblerMotor {
public:
	void run() {
    int pwmSpeed = DRIBBLER_SPEED / 100 * 255;
    digitalWrite(DR_DIR, LOW);
    analogWrite(DR_PWM, pwmSpeed);
	};

	void stop() {
    digitalWrite(DR_DIR, LOW);
    analogWrite(DR_PWM, 0);
	};
};

DribblerMotor DR = DribblerMotor();


#define FORWARD_TOLERANCE PI / 10
// declarations here
void blinkLED();
float find_move_angle(PositionSystem posv, Vector goal_pos, float tolerance, float ball_angle, float ball_magnitude);

PositionSystem pos_sys;
// 0.5 is how much the rotation is scaled compared to the movement
MotorController motor_ctrl(0.8);

IRSensor ir_sensor;
LineSensor line_sensor;

bool headless = true;
bool move = false;

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
}

void loop() {
  Serial.println(".");
  pos_sys.update(); // call every loop (it reads all the sensors)
  ir_sensor.update();
  line_sensor.update();

  float heading = pos_sys.get_heading(); // returns unit circle heading
  Vector posv = pos_sys.get_posv(); // note this is a custom class (uppercase) the cpp vector is lowercase
  String posv_str = String(posv.display().c_str()); // must convert from std::string to String (arduino)
  
  Vector mv_vec = pos_sys.get_relative_to(Vector(91, 180));
  
  if (digitalRead(BTN_1) == HIGH) {
    pos_sys.set_pos(Vector(91, 110), 0); // set position of otos
  }
  if (digitalRead(BTN_2) == HIGH) {
    pos_sys.set_pos(Vector(48.5, 57.5), 0); // set position of otos
  }
  if (digitalRead(BTN_3) == HIGH) {
    pos_sys.set_pos(Vector(91, 57.5), 0); // set position of otos
  }
  if (digitalRead(BTN_4) == HIGH) {
    pos_sys.set_pos(Vector(133.5, 57.5), 0); // set position of otos
  }
  if (digitalRead(BTN_5) == HIGH) {
    pos_sys.set_pos(Vector(91, 37.5), 0); // set position of otos
  }
  if (digitalRead(BTN_1) || digitalRead(BTN_2) || digitalRead(BTN_3) || digitalRead(BTN_4) || digitalRead(BTN_5)) {
    move = true;
  }

  Serial.print(ir_sensor.get_magnitude());
  float ball_angle = fmodf(ir_sensor.get_angle() + heading * PI / 180, 2 * PI);
  float line_angle = fmodf(line_sensor.get_angle() + heading * PI / 180, 2 * PI);

  Vector goal_vec = pos_sys.get_relative_to(Vector(91, 180));

  // convert unit circle heading to rotation
  float rotation = goal_vec.heading() * 180 / PI - heading - 90; // convert to degrees
  while (rotation > 180) rotation -= 360;
  while (rotation < -180) rotation += 360;

  float mv_angle = 0;
  mv_angle = find_move_angle(pos_sys, Vector(91, 180), FORWARD_TOLERANCE, ball_angle, ir_sensor.get_magnitude());

  if (line_sensor.get_distance() != 0) {
    mv_angle = (line_angle) + PI;
  }

  float speed = 100;
  
  if ((ir_sensor.get_magnitude() == 0 && line_sensor.get_distance() == 0) || !move) {
    speed = 0;
    DR.stop();
  }
  else {
    DR.run();
  }

  if (headless) mv_angle -= heading*PI/180;
  Serial.println(rotation);
  motor_ctrl.run_motors(speed, mv_angle, rotation);

  digitalWrite(DEBUG_LED, HIGH);
}

float find_move_angle(PositionSystem posv, Vector goal_pos, float tolerance, float ball_angle, float ball_magnitude) {
  Vector goal_vec = posv.get_relative_to(goal_pos);
  float angle_diff = PI / 2 - goal_vec.heading();
  if (ball_magnitude < 40) {
    return ball_angle;
  }
  if (ball_angle > goal_vec.heading() - tolerance && ball_angle < goal_vec.heading() + tolerance) {
    // return goal_vec.heading();
    return goal_vec.heading(); // move forward
  }
  else if ((ball_angle > goal_vec.heading() + tolerance) || (ball_angle < -PI / 2 + angle_diff)) {
    return ball_angle + PI / 18 * 6; // turn right
  }
  else if ((ball_angle < goal_vec.heading() - tolerance)) {
    return ball_angle - PI / 18 * 6; // turn left
  }
}