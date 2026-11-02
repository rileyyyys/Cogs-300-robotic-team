
/*
  COGS 300 - AUTONOMOUS WALL FOLLOWER
  Arduino UNO R4 WiFi

  Automatically starts 3 seconds after power-on.
  No Serial Monitor required.

  MOTOR A = RIGHT WHEELS
  MOTOR B = LEFT WHEELS

  TARGET LEFT WALL DISTANCE = 15 cm
  FRONT CORNER THRESHOLD = 15 cm
*/

#include <Arduino.h>

// =====================================
// 1. CONFIRMED WIRING
// =====================================

// Left ultrasonic
const int LEFT_TRIG = 13;
const int LEFT_ECHO = 12;

// Front ultrasonic
const int FRONT_TRIG = 11;
const int FRONT_ECHO = 10;

// Right motor: Channel A
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Left motor: Channel B
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

// Set to true if a side runs backward
const bool REVERSE_LEFT = false;
const bool REVERSE_RIGHT = false;

// =====================================
// 2. ROBOT SETTINGS
// =====================================

const float TARGET_WALL = 15.0;
const float FRONT_STOP = 15.0;
const float FRONT_CLEAR = 25.0;

const float WALL_LOST = 35.0;
const float WALL_FOUND = 24.0;

const float WALL_TOLERANCE = 1.5;

// Speeds: 0-255
const int BASE_SPEED = 115;
const int MIN_SPEED = 75;
const int MAX_SPEED = 175;
const int TURN_SPEED = 120;

// Steering strength
const float KP = 4.0;

// Maximum steering correction
const int MAX_CORRECTION = 65;

// Motor acceleration limit per cycle
const int ACCEL_STEP = 12;

// Timing
const unsigned long START_DELAY = 3000;
const unsigned long RIGHT_TURN_MAX = 2200;
const unsigned long LEFT_TURN_MAX = 2200;
const unsigned long CORNER_FORWARD_MS = 320;
const unsigned long MIN_TURN_MS = 250;

// =====================================
// 3. ROBOT STATE
// =====================================

enum State {
  FOLLOW_WALL,
  RIGHT_CORNER,
  OUTSIDE_FORWARD,
  LEFT_CORNER,
  ERROR_STOP
};

State state = FOLLOW_WALL;

unsigned long stateStart = 0;

float leftCM = -1;
float frontCM = -1;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

int lostCount = 0;
int leftInvalidCount = 0;
int frontInvalidCount = 0;
int clearCount = 0;
int foundCount = 0;

// =====================================
// 4. ULTRASONIC FUNCTIONS
// =====================================

float readDistance(int trig, int echo) {

  digitalWrite(trig, LOW);
  delayMicroseconds(3);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long duration =
    pulseIn(echo, HIGH, 25000UL);

  if (duration == 0) {
    return -1;
  }

  float cm = duration * 0.0343 / 2.0;

  if (cm < 2 || cm > 350) {
    return -1;
  }

  return cm;
}

// Exponential smoothing for valid readings
float smoothReading(float previous, float current) {

  if (current < 0) {
    return -1;
  }

  if (previous < 0) {
    return current;
  }

  return previous * 0.55 + current * 0.45;
}

void updateSensors() {

  float newLeft =
    readDistance(LEFT_TRIG, LEFT_ECHO);

  // Separate ultrasonic triggers
  delay(35);

  float newFront =
    readDistance(FRONT_TRIG, FRONT_ECHO);

  leftCM = smoothReading(leftCM, newLeft);
  frontCM = smoothReading(frontCM, newFront);
}

// =====================================
// 5. MOTOR FUNCTIONS
// =====================================

void motorOutput(
  int speedValue,
  int enablePin,
  int pin1,
  int pin2,
  bool reverse
) {

  speedValue = constrain(speedValue, -255, 255);

  if (reverse) {
    speedValue = -speedValue;
  }

  if (speedValue > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(enablePin, -speedValue);
  }
  else {
    analogWrite(enablePin, 0);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

// Smoothly approach desired speed
int approachSpeed(int current, int target) {

  if (current < target) {
    return min(current + ACCEL_STEP, target);
  }

  if (current > target) {
    return max(current - ACCEL_STEP, target);
  }

  return target;
}

void driveSmooth(int leftTarget, int rightTarget) {

  leftTarget = constrain(leftTarget, -255, 255);
  rightTarget = constrain(rightTarget, -255, 255);

  leftMotorSpeed =
    approachSpeed(leftMotorSpeed, leftTarget);

  rightMotorSpeed =
    approachSpeed(rightMotorSpeed, rightTarget);

  // LEFT = Channel B
  motorOutput(
    leftMotorSpeed, ENB, IN3, IN4, REVERSE_LEFT
  );

  // RIGHT = Channel A
  motorOutput(
    rightMotorSpeed, ENA, IN1, IN2, REVERSE_RIGHT
  );
}

void stopNow() {

  leftMotorSpeed = 0;
  rightMotorSpeed = 0;

  motorOutput(0, ENB, IN3, IN4, REVERSE_LEFT);
  motorOutput(0, ENA, IN1, IN2, REVERSE_RIGHT);
}

void enterState(State nextState) {

  state = nextState;
  stateStart = millis();

  lostCount = 0;
  clearCount = 0;
  foundCount = 0;
  leftInvalidCount = 0;
  frontInvalidCount = 0;
}

// =====================================
// 6. NORMAL WALL FOLLOWING
// =====================================

void followWall() {

  // Front sensor must be valid
  if (frontCM < 0) {
    stopNow();
    return;
  }

  // Obstacle directly in front
  if (frontCM <= FRONT_STOP) {
    enterState(RIGHT_CORNER);
    driveSmooth(TURN_SPEED, -TURN_SPEED);
    return;
  }

  if (leftCM < 0) {

    leftInvalidCount++;
    stopNow();

    if (leftInvalidCount >= 5) {
      enterState(ERROR_STOP);
    }

    return;
  }

  leftInvalidCount = 0;

  // Outside corner detection
  if (leftCM >= WALL_LOST) {
    lostCount++;
  }
  else {
    lostCount = 0;
  }

  if (lostCount >= 3) {
    enterState(OUTSIDE_FORWARD);
    driveSmooth(BASE_SPEED, BASE_SPEED);
    return;
  }

  // Proportional controller
  float error = leftCM - TARGET_WALL;

  float correction = KP * error;

  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );

  if (abs(error) <= WALL_TOLERANCE) {
    correction = 0;
  }

  // Far from left wall -> turn left
  // Close to left wall -> turn right
  int leftTarget = BASE_SPEED - correction;
  int rightTarget = BASE_SPEED + correction;

  leftTarget = constrain(
    leftTarget, MIN_SPEED, MAX_SPEED
  );

  rightTarget = constrain(
    rightTarget, MIN_SPEED, MAX_SPEED
  );

  // Slow down if approaching front obstacle
  if (frontCM < FRONT_CLEAR) {

    float factor = (frontCM - FRONT_STOP) /
                   (FRONT_CLEAR - FRONT_STOP);

    factor = constrain(factor, 0.4f, 1.0f);

    leftTarget = (int)(leftTarget * factor);
    rightTarget = (int)(rightTarget * factor);
  }

  driveSmooth(leftTarget, rightTarget);
}

// =====================================
// 7. INSIDE CORNER - TURN RIGHT
// =====================================

void handleRightCorner() {

  unsigned long elapsed = millis() - stateStart;

  if (elapsed > RIGHT_TURN_MAX) {
    enterState(ERROR_STOP);
    stopNow();
    return;
  }

  if (frontCM < 0) {
    frontInvalidCount++;
  }
  else {
    frontInvalidCount = 0;
  }

  // Stop if readings remain invalid
  if (frontInvalidCount >= 3) {
    enterState(ERROR_STOP);
    stopNow();
    return;
  }

  // Require two clear measurements
  if (frontCM >= FRONT_CLEAR) {
    clearCount++;
  }
  else {
    clearCount = 0;
  }

  if (elapsed >= MIN_TURN_MS &&
      clearCount >= 2) {

    enterState(FOLLOW_WALL);
    return;
  }

  // Keep rotating right without
  // stopping between sensor readings
  driveSmooth(TURN_SPEED, -TURN_SPEED);
}

// =====================================
// 8. OUTSIDE CORNER - MOVE FORWARD
// =====================================

void handleOutsideForward() {

  unsigned long elapsed = millis() - stateStart;

  if (frontCM < 0) {
    enterState(ERROR_STOP);
    stopNow();
    return;
  }

  if (frontCM <= FRONT_STOP) {
    enterState(RIGHT_CORNER);
    return;
  }

  // Move forward to position the robot
  // before rotating around the corner.
  if (elapsed < CORNER_FORWARD_MS) {
    driveSmooth(BASE_SPEED, BASE_SPEED);
    return;
  }

  enterState(LEFT_CORNER);
}

// =====================================
// 9. OUTSIDE CORNER - TURN LEFT
// =====================================

void handleLeftCorner() {

  unsigned long elapsed = millis() - stateStart;

  if (elapsed > LEFT_TURN_MAX) {
    enterState(ERROR_STOP);
    stopNow();
    return;
  }

  if (frontCM < 0) {
    frontInvalidCount++;
  }
  else {
    frontInvalidCount = 0;
  }

  if (frontInvalidCount >= 3) {
    enterState(ERROR_STOP);
    stopNow();
    return;
  }

  if (frontCM > 0 && frontCM <= FRONT_STOP) {
    enterState(RIGHT_CORNER);
    return;
  }

  if (leftCM > 0 && leftCM <= WALL_FOUND) {
    foundCount++;
  }
  else {
    foundCount = 0;
  }

  if (elapsed >= MIN_TURN_MS &&
      foundCount >= 2) {

    enterState(FOLLOW_WALL);
    return;
  }

  driveSmooth(-TURN_SPEED, TURN_SPEED);
}

// =====================================
// 10. SETUP
// =====================================

void setup() {

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(LEFT_TRIG, LOW);
  digitalWrite(FRONT_TRIG, LOW);

  stopNow();

  // Give user time to set robot down
  delay(START_DELAY);

  // Initially validate both sensors
  updateSensors();

  if (leftCM < 0 || frontCM < 0 ||
      frontCM <= FRONT_STOP) {

    enterState(ERROR_STOP);
    stopNow();
    return;
  }

  enterState(FOLLOW_WALL);
}

// =====================================
// 11. AUTONOMOUS MAIN LOOP
// =====================================

void loop() {

  if (state == ERROR_STOP) {
    stopNow();
    delay(100);
    return;
  }

  updateSensors();

  switch (state) {

    case FOLLOW_WALL:
      followWall();
      break;

    case RIGHT_CORNER:
      handleRightCorner();
      break;

    case OUTSIDE_FORWARD:
      handleOutsideForward();
      break;

    case LEFT_CORNER:
      handleLeftCorner();
      break;

    case ERROR_STOP:
      stopNow();
      break;
  }

  delay(20);
}
