#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define Max_Pulse 2000
#define Min_Pulse 1000
#define PWM_Frequency 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int claw_left_channel = 11;
int claw_right_channel = 12; 

void move_servo_angle(int channel, double angle) {
  long angle_pulse = map(angle, 0.0, 180.0, Min_Pulse, Max_Pulse); 
  pwm.writeMicroseconds(channel, angle_pulse);
}

void setup(){
  pwm.begin();
  pwm.setPWMFreq(PWM_Frequency); 
  return_claw();
}

void loop() {
  
}

void close_claw() {
  move_servo_angle(claw_left_channel, 90);
  move_servo_angle(claw_right_channel, 90);
}

void open_claw() {
  move_servo_angle(claw_left_channel, 20);
  move_servo_angle(claw_right_channel, 160);
}

void return_claw() {
  move_servo_angle(claw_left_channel, 45);
  move_servo_angle(claw_right_channel, 135);
}
