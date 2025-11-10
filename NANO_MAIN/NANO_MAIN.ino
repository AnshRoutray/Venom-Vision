#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define Max_Pulse 2000
#define Min_Pulse 1000
#define PWM_Frequency 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define claw_left_channel 12
#define claw_right_channel 13
#define potPin A0

// Servo channels on PCA9685 (0-15)
#define FL_HIP 0
#define FL_KNEE 1
#define FR_HIP 2
#define FR_KNEE 3
#define ML_HIP 4
#define ML_KNEE 5
#define MR_HIP 6
#define MR_KNEE 7
#define BL_HIP 8
#define BL_KNEE 9
#define BR_HIP 10
#define BR_KNEE 11
#define ESP32_CAM A1;

// Servo pulse lengths (tune these!)
#define SERVOMIN 150   // Min pulse length (0 degrees)
#define SERVOMAX 600   // Max pulse length (180 degrees)
#define SERVOSTOP 375

// Gait parameters - SPEED OPTIMIZED
#define HIP_CENTER 90
#define HIP_FORWARD 65      // More aggressive swing
#define HIP_BACKWARD 115

#define KNEE_UP 40          // Higher lift = faster
#define KNEE_DOWN 95

// State machine timing (FAST MODE)
#define LIFT_TIME 80        // ms to lift legs (was 100)
#define SWING_TIME 120      // ms to swing (was 200)
#define LOWER_TIME 80       // ms to lower legs (was 100)

int targetAngle = 60; 

// Walking state machine
enum WalkState {
  IDLE,
  LIFT_A,
  SWING_A,
  LOWER_A,
  LIFT_B,
  SWING_B,
  LOWER_B
};

WalkState currentState = IDLE;
unsigned long lastStateChange = 0;
bool isWalking = false;

// Movement commands from camera
enum Command {
  CMD_STOP,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT
};

void setServoAngle(int channel, double angle) {
  long angle_pulse = map(angle, 0.0, 180.0, Min_Pulse, Max_Pulse); 
  pwm.writeMicroseconds(channel, angle_pulse);
}
void moveServo(int servoChannel, int targetAngle) {
    int currentAngle = map(analogRead(potPin), 0, 675, 0, 180);
  while (currentAngle != targetAngle) {
    int error = targetAngle - currentAngle;
    int speed = constrain(error * 2, -50, 50); // Proportional control

    if (abs(error) > 3) { // Deadband
      int pulseWidth;
      if (speed > 0) {
        pulseWidth = map(speed, 0, 50, SERVOSTOP, SERVOMAX);
      } else {
        pulseWidth = map(speed, 0, -50, SERVOSTOP, SERVOMIN);
      }
      pwm.setPWM(servoChannel, 0, pulseWidth);
    } else {
      pwm.setPWM(servoChannel, 0, SERVOSTOP); // Stop
    }
    currentAngle = map(analogRead(potPin), 0, 675, 0, 180);
    Serial.print("Target: "); Serial.print(targetAngle);
    Serial.print(" Current: "); Serial.print(currentAngle);
    Serial.print(" Error: "); Serial.println(error);
    int raw = analogRead(potPin);  // 0–1023
    Serial.println(raw);
    delay(20);
  }
}
Command currentCommand = CMD_STOP;

void setup(){
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(PWM_Frequency); 
  delay(10);
  return_claw();
  // Stand neutral
  standNeutral();
}

void loop() {
    // Check for commands from ESP32-CAM
  readCameraCommands();
  
  // Update walking state machine
  updateWalkingStateMachine();
  
  // Other sensors, LED updates, etc. can go here
  // All non-blocking!
}

// Standing position
void standNeutral() {
  setServoAngle(FL_HIP, HIP_CENTER);
  setServoAngle(FR_HIP, HIP_CENTER);
  setServoAngle(ML_HIP, HIP_CENTER);
  setServoAngle(MR_HIP, HIP_CENTER);
  setServoAngle(BL_HIP, HIP_CENTER);
  setServoAngle(BR_HIP, HIP_CENTER);
  
  setServoAngle(FL_KNEE, KNEE_DOWN);
  setServoAngle(FR_KNEE, KNEE_DOWN);
  setServoAngle(ML_KNEE, KNEE_DOWN);
  setServoAngle(MR_KNEE, KNEE_DOWN);
  setServoAngle(BL_KNEE, KNEE_DOWN);
  setServoAngle(BR_KNEE, KNEE_DOWN);
}

// NON-BLOCKING state machine
void updateWalkingStateMachine() {
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - lastStateChange;
  
  // Handle command changes
  if (currentCommand == CMD_STOP && currentState != IDLE) {
    currentState = IDLE;
    standNeutral();
    return;
  }
  
  if (currentCommand == CMD_FORWARD && currentState == IDLE) {
    currentState = LIFT_A;
    lastStateChange = currentTime;
  }
  
  // State machine for forward walking
  switch (currentState) {
    case IDLE:
      // Do nothing, waiting for command
      break;
      
    case LIFT_A:
      if (elapsed == 0) {  // Just entered this state
        // Lift Group A (FL, MR, BL)
        setServoAngle(FL_KNEE, KNEE_UP);
        setServoAngle(MR_KNEE, KNEE_UP);
        setServoAngle(BL_KNEE, KNEE_UP);
      }
      if (elapsed >= LIFT_TIME) {
        currentState = SWING_A;
        lastStateChange = currentTime;
      }
      break;
      
    case SWING_A:
      if (elapsed == 0) {
        // Swing Group A forward, Group B back
        setServoAngle(FL_HIP, HIP_FORWARD);
        setServoAngle(MR_HIP, HIP_FORWARD);
        setServoAngle(BL_HIP, HIP_FORWARD);
        
        setServoAngle(FR_HIP, HIP_BACKWARD);
        setServoAngle(ML_HIP, HIP_BACKWARD);
        setServoAngle(BR_HIP, HIP_BACKWARD);
      }
      if (elapsed >= SWING_TIME) {
        currentState = LOWER_A;
        lastStateChange = currentTime;
      }
      break;
      
    case LOWER_A:
      if (elapsed == 0) {
        // Lower Group A
        setServoAngle(FL_KNEE, KNEE_DOWN);
        setServoAngle(MR_KNEE, KNEE_DOWN);
        setServoAngle(BL_KNEE, KNEE_DOWN);
      }
      if (elapsed >= LOWER_TIME) {
        currentState = LIFT_B;
        lastStateChange = currentTime;
      }
      break;
      
    case LIFT_B:
      if (elapsed == 0) {
        // Lift Group B (FR, ML, BR)
        setServoAngle(FR_KNEE, KNEE_UP);
        setServoAngle(ML_KNEE, KNEE_UP);
        setServoAngle(BR_KNEE, KNEE_UP);
      }
      if (elapsed >= LIFT_TIME) {
        currentState = SWING_B;
        lastStateChange = currentTime;
      }
      break;
      
    case SWING_B:
      if (elapsed == 0) {
        // Swing Group B forward, Group A back
        setServoAngle(FR_HIP, HIP_FORWARD);
        setServoAngle(ML_HIP, HIP_FORWARD);
        setServoAngle(BR_HIP, HIP_FORWARD);
        
        setServoAngle(FL_HIP, HIP_BACKWARD);
        setServoAngle(MR_HIP, HIP_BACKWARD);
        setServoAngle(BL_HIP, HIP_BACKWARD);
      }
      if (elapsed >= SWING_TIME) {
        currentState = LOWER_B;
        lastStateChange = currentTime;
      }
      break;
      
    case LOWER_B:
      if (elapsed == 0) {
        // Lower Group B
        setServoAngle(FR_KNEE, KNEE_DOWN);
        setServoAngle(ML_KNEE, KNEE_DOWN);
        setServoAngle(BR_KNEE, KNEE_DOWN);
      }
      if (elapsed >= LOWER_TIME) {
        // Complete cycle - check if should continue
        if (currentCommand == CMD_FORWARD) {
          currentState = LIFT_A;  // Loop back
        } else {
          currentState = IDLE;
        }
        lastStateChange = currentTime;
      }
      break;
  }
}

// Read commands from ESP32-CAM
void readCameraCommands() {
  if(Serial.available() > 0){
    uint8_t cmd = Serial.read();
    switch(cmd) {
      case 1: 
      currentCommand = CMD_STOP;
      break;
      case 2:
      currentCommand = CMD_FORWARD;
      break;
      case 3:
      currentCommand = CMD_RIGHT;
      break;
      case 4:
      currentCommand = CMD_LEFT;
      break;
    }
  }
}

void close_claw() {
  setServoAngle(claw_left_channel, 90);
  setServoAngle(claw_right_channel, 90);
}

void open_claw() {
  setServoAngle(claw_left_channel, 20);
  setServoAngle(claw_right_channel, 160);
}

void return_claw() {
  setServoAngle(claw_left_channel, 45);
  setServoAngle(claw_right_channel, 135);
}
