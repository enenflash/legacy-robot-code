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

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int DRIBBLER_SPEED = 100;
class DribblerMotor {
public:

	void run() {
    int pwmSpeed = DRIBBLER_SPEED / 100 * 255;
    digitalWrite(DF_DIR1, LOW);
    analogWrite(DF_PWM, pwmSpeed);
	};

	void stop() {
    digitalWrite(DF_DIR1, LOW);
    analogWrite(DF_PWM, 0);
	};
};

DribblerMotor DF = DribblerMotor();


#define FORWARD_TOLLERANCE PI / 10
// declarations here
void blinkLED();
float find_move_angle(PositionSystem posv, Vector goal_pos, float tollerance, float ball_angle, float ball_magnitude) {
  Vector goal_vec = posv.get_relative_to(goal_pos);
  float angle_diff = PI / 2 - goal_vec.heading();
  if (ball_magnitude < 40) {
    DF.stop();
    return ball_angle;
  }
  if (ball_angle > goal_vec.heading() - tollerance && ball_angle < goal_vec.heading() + tollerance) {
    // return goal_vec.heading();
    // float current_i = posv.get_posv().i;
    // float current_j = posv.get_posv().j;
    // if (current_i > goal_pos.i - 20 && current_i < goal_pos.i + 20 && current_j> goal_pos.j - 20) {
    //   DF.stop(); // stop dribbler if close to goal
    //   return 0; // move forward
    // }
    DF.run(); // run dribbler
    return goal_vec.heading(); // move forward
  }
  else if ((ball_angle > goal_vec.heading() + tollerance) || (ball_angle < -PI / 2 + angle_diff)) {
    DF.stop();
    return ball_angle + PI / 18 * 6; // turn right
  }
  else if ((ball_angle < goal_vec.heading() - tollerance)) {
    DF.stop();
    return ball_angle - PI / 18 * 6; // turn left
  }
}
Adafruit_SSD1306 display(128, 32, &Wire, -1);
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

  pinMode(DF_PWM, OUTPUT);
  pinMode(DF_DIR1, OUTPUT);

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
  
    Vector mv_vec = pos_sys.get_relative_to((Vector){91, 200});

  
  if (digitalRead(BTN_1) == HIGH) {
    pos_sys.otos.set_pos(91, 110, 0); // set position of otos
  }
  if (digitalRead(BTN_2) == HIGH) {
    pos_sys.otos.set_pos(48.5, 57.5, 0); // set position of otos
  }
  if (digitalRead(BTN_3) == HIGH) {
    pos_sys.otos.set_pos(91, 57.5, 0); // set position of otos
  }
  if (digitalRead(BTN_4) == HIGH) {
    pos_sys.otos.set_pos(133.5, 57.5, 0); // set position of otos
  }
  if (digitalRead(BTN_5) == HIGH) {
    pos_sys.otos.set_pos(91, 37.5, 0); // set position of otos
  }
  if (digitalRead(BTN_1) || digitalRead(BTN_2) || digitalRead(BTN_3) || digitalRead(BTN_4) || digitalRead(BTN_5)) {
    move = true;
  }

  // if (ir_sensor.get_magnitude() != 0) ir_sensor.angle -= heading*PI/180;
  // ir_sensor.angle = fmod((ir_sensor.angle + PI), 2 * PI) - PI;
  // line_sensor.angle -= heading*PI/180;
  // Serial.print(heading);
  // Serial.print(" ");
  // Serial.println(posv_str);

  // Serial.print(ir_sensor.get_angle());
  // Serial.print(" ");
  // Serial.print(ir_sensor.get_magnitude());
  // Serial.print(" ");
  // Serial.print(heading*PI/180);
  // Serial.print(" ");
  // Serial.print(remainder(ir_sensor.get_angle() + heading * PI / 180, 2 * PI));
  float ball_angle = fmodf(PI + ir_sensor.get_angle() + heading * PI / 180, 2 * PI) - PI;
  float line_angle = fmodf(PI + line_sensor.get_angle() + heading * PI / 180, 2 * PI) - PI;
  // Serial.print(ball_angle);
  // Serial.print(" ");
  // Serial.println(ir_sensor.read_success);

  Vector goal_vec = pos_sys.get_relative_to((Vector){91, 200});

  // convert unit circle heading to rotation
  float rotation = goal_vec.heading() * 180 / PI - heading - 90; // convert to degrees
  // rotation = fmodf(rotation + 180.0f, 360.0f) - 180.0f; // convert to range [-180, 180]
  // rotation %= 360; // convert to range [0, 360]
  while (rotation > 180) rotation -= 360;
  while (rotation < -180) rotation += 360;
  // rotation *= -1;
  // idk where to put this code so it is here for now

  // headless is 'rotation matrix'
  float mv_angle = 0;
  mv_angle = find_move_angle(pos_sys, (Vector){91, 180}, FORWARD_TOLLERANCE, ball_angle, ir_sensor.get_magnitude());
  // if (ball_angle < PI/2 - FORWARD_TOLLERANCE && ball_angle > -PI/2) {
  //   mv_angle = ball_angle - PI/18 * 7;
  //   // Serial.print(" ");
  //   // Serial.print("right");
  // }
  // else if (ball_angle > PI/2 + FORWARD_TOLLERANCE || ball_angle < -PI/2) {
  //   mv_angle = ball_angle + PI/18 * 7;
  //   // Serial.print(" ");
  //   // Serial.print("left");
  // }
  // else {
  //   mv_angle = PI/2;
  // }

  if (line_sensor.get_distance() != 0) {
    mv_angle = (line_angle) + PI;
  }




  float speed = 100;

  
  if ((ir_sensor.get_magnitude() == 0 && line_sensor.get_distance() == 0) || !move) {
    speed = 0;
    DF.stop();
  }
  else {
    DF.run();
  }

  // Serial.println(mv_angle*180/PI);
  if (headless) mv_angle -= heading*PI/180;
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
  motor_ctrl.run_motors(speed, mv_angle, rotation); // run motors 50 speed, angle (radians), rotation
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