#include "Servo.h"
#include "RF24.h"

// Pin Definitions
#define PIN_SERVO           2
#define MOTOR_DIRECTION     0
#define PIN_DIRECTION_LEFT  4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT  6
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_SONIC_TRIG      7
#define PIN_SONIC_ECHO      8
#define PIN_TRACKING_LEFT   A1
#define PIN_TRACKING_CENTER A2
#define PIN_TRACKING_RIGHT  A3
#define PIN_BATTERY         A0

// Constants
#define OBSTACLE_DISTANCE       40
#define OBSTACLE_DISTANCE_LOW   15
#define LINE_TRACKING_THRESHOLD 7
#define TK_FORWARD_SPEED        50
#define TK_TURN_SPEED           180
#define TK_STOP_SPEED           0
#define OA_SCAN_ANGLE_MIN       30
#define OA_SCAN_ANGLE_MAX       150
#define OA_WAITTING_SERVO_TIME  130

// RF24 Remote Control
#define NRF_CE_PIN              9
#define NRF_CSN_PIN             10
#define MODE_COMBINED           1

// Objects
Servo servo;
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// Variables
int currentMode = MODE_COMBINED;
float batteryVoltage = 0;
int tk_VoltageCompensationToSpeed = 0;
int servoAngle = 90;
bool scanningLeftToRight = true; // Servo scanning direction flag

int distance[3]; // To store obstacle distances from sonar

// Function Declarations
void setup();
void loop();
void combinedMode();
void updateObstacleAvoidance();
void updateLineTracking();
void updateServoScan();
float getSonar();
u8 getTrackingSensorVal();
void motorRun(int speedl, int speedr);
void calculateVoltageCompensation();

void setup() {
  // Initialize pins
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  pinMode(PIN_SONIC_ECHO, INPUT);
  pinMode(PIN_TRACKING_LEFT, INPUT);
  pinMode(PIN_TRACKING_CENTER, INPUT);
  pinMode(PIN_TRACKING_RIGHT, INPUT);

  // Initialize servo
  servo.attach(PIN_SERVO);
  servo.write(90);

  // Calculate speed compensation based on battery voltage
  calculateVoltageCompensation();
}

void loop() {
  if (currentMode == MODE_COMBINED) {
    combinedMode();
  }
}

// Combines line tracking and obstacle avoidance
void combinedMode() {
  // Update obstacle distances
  updateObstacleAvoidance();

  // If no obstacle is detected ahead, perform line tracking
  if (distance[1] >= OBSTACLE_DISTANCE) {
    updateLineTracking();
  }
}

// Updates obstacle avoidance logic
void updateObstacleAvoidance() {
  int tempDistance[3] = {0};
  for (int i = 0; i < 3; i++) {
    updateServoScan(); // Ensure servo scans left and right
    delay(OA_WAITTING_SERVO_TIME);
    tempDistance[i] = getSonar();
  }

  distance[0] = tempDistance[0]; // Left
  distance[1] = tempDistance[1]; // Center
  distance[2] = tempDistance[2]; // Right

  if (distance[1] < OBSTACLE_DISTANCE) { // Obstacle detected ahead
    if (distance[0] > distance[2]) {
      motorRun(-150, 150); // Turn left
    } else {
      motorRun(150, -150); // Turn right
    }
    delay(500);
  }
}

// Updates line tracking logic
void updateLineTracking() {
  u8 trackingSensorVal = getTrackingSensorVal();
  switch (trackingSensorVal) { //1 is non they are backwards
    case 7:   // 111 - Stop
      motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);
    break;
    case 0:
      motorRun(-TK_FORWARD_SPEED, -TK_FORWARD_SPEED);
    break;
    case 3:   // 011 - Turn Right
      motorRun(-TK_TURN_SPEED, TK_TURN_SPEED);
      break;
    case 1:   // 001 - Turn Right
      motorRun(-TK_TURN_SPEED, TK_TURN_SPEED);
      break;
    case 6:   // 110 - Turn Left
      motorRun(TK_TURN_SPEED, -TK_TURN_SPEED);
      break;
    case 4:   // 100 - Turn Left
      motorRun(TK_TURN_SPEED, -TK_TURN_SPEED);
      break;
    case 2 :
      motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);
    default:  // Default to Forward
      motorRun(TK_STOP_SPEED, TK_STOP_SPEED);
      break;
  }
}

// Servo scanning logic for obstacle avoidance
void updateServoScan() {
  if (scanningLeftToRight) {
    servoAngle += 30; // Increment angle to move right
    if (servoAngle >= OA_SCAN_ANGLE_MAX) {
      servoAngle = OA_SCAN_ANGLE_MAX; // Clamp to max angle
      scanningLeftToRight = false;   // Switch direction
    }
  } else {
    servoAngle -= 30; // Decrement angle to move left
    if (servoAngle <= OA_SCAN_ANGLE_MIN) {
      servoAngle = OA_SCAN_ANGLE_MIN; // Clamp to min angle
      scanningLeftToRight = true;    // Switch direction
    }
  }
  servo.write(servoAngle); // Update servo position
}

// Gets the distance from the ultrasonic sensor
float getSonar() {
  digitalWrite(PIN_SONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_SONIC_ECHO, HIGH);
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

// Reads the tracking sensor values
u8 getTrackingSensorVal() {
  u8 val = 0;
  val |= digitalRead(PIN_TRACKING_LEFT) << 2;
  val |= digitalRead(PIN_TRACKING_CENTER) << 1;
  val |= digitalRead(PIN_TRACKING_RIGHT);
  return val;
}

// Runs the motors with given speeds
void motorRun(int speedl, int speedr) {
  int dirL = speedl > 0 ? 0 : 1;
  int dirR = speedr > 0 ? 1 : 0;

  speedl = abs(speedl);
  speedr = abs(speedr);

  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

// Calculates voltage compensation for consistent speeds
void calculateVoltageCompensation() {
  int batteryADC = analogRead(PIN_BATTERY);
  batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
  tk_VoltageCompensationToSpeed = (7.0 - batteryVoltage) * 30;
}