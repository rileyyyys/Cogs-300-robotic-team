
/*
  COGS 300 - LAB 7
  WHITE TAPE LINE FOLLOWER
  Arduino UNO R4 WiFi

  3 semi-spread IR sensors
  Independent left/right baseline speeds
  NO automatic motor boost

  ORIGINAL MOTOR WIRING:
  RIGHT: ENA 9, IN1 8, IN2 7
  LEFT:  ENB 3, IN3 5, IN4 4

  IR SENSORS:
  LEFT   A0
  CENTER A1
  RIGHT  A2
*/

#include <Arduino.h>

// =====================================
// 1. MOTOR PINS
// =====================================

const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

// Original motor orientation
const bool REVERSE_LEFT = true;
const bool REVERSE_RIGHT = false;

// =====================================
// 2. IR SENSORS
// =====================================

const int IR_LEFT = A0;
const int IR_CENTER = A1;
const int IR_RIGHT = A2;

const bool IR_ACTIVE_LOW = true;

// =====================================
// 3. SPEED SETTINGS
// =====================================

// Independent motor baseline speeds
const int BASE_SPEED = 70;

// Proportional steering
const float KP = 12;
const int MAX_CORRECTION = 18;

// Strong corner steering
const int CORNER_FAST = 115;
const int CORNER_SLOW = 25;

// Lost-line recovery
const int SEARCH_SPEED = 75;

const int LEFT_MOTOR_BOOST = 35;

// Motor acceleration smoothing
const int ACCEL_STEP = 10;

// Timing
const unsigned long START_DELAY = 3000;
const unsigned long LOOP_DELAY = 10;
const unsigned long LOST_GRACE_MS = 50;
const unsigned long MAX_SEARCH_MS = 1200;

// =====================================
// 4. ROBOT STATE
// =====================================

bool irL = false;
bool irC = false;
bool irR = false;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// -1 = left
//  0 = unknown
// +1 = right
int lastSide = 0;

float filteredError = 0;

unsigned long lostSince = 0;

int previousLeft = 0;
int previousRight = 0;

bool stoppedForLostLine = false;

// =====================================
// 5. IR SENSOR FUNCTIONS
// =====================================

bool readIR(int pin) {
  int value = digitalRead(pin);

  return IR_ACTIVE_LOW
    ? (value == LOW)
    : (value == HIGH);
}

void updateIR() {
  irL = readIR(IR_LEFT);
  irC = readIR(IR_CENTER);
  irR = readIR(IR_RIGHT);
}

// =====================================
// 6. MOTOR OUTPUT
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

// =====================================
// 7. SMOOTH MOTOR CONTROL
// =====================================

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

  // Compensate for weaker left motor
  if (leftTarget > 0) {
    leftTarget += LEFT_MOTOR_BOOST;
  }

  leftTarget = constrain(leftTarget, 0, 255);
  rightTarget = constrain(rightTarget, 0, 255);

  leftMotorSpeed =
      approachSpeed(leftMotorSpeed, leftTarget);

  rightMotorSpeed =
      approachSpeed(rightMotorSpeed, rightTarget);

  motorOutput(
      leftMotorSpeed,
      ENB, IN3, IN4, REVERSE_LEFT
  );

  motorOutput(
      rightMotorSpeed,
      ENA, IN1, IN2, REVERSE_RIGHT
  );
}

// =====================================
// 8. LINE FOLLOWING
// =====================================

void followLine() {

  if (stoppedForLostLine) {
    stopNow();
    return;
  }

  int leftCmd = 0;
  int rightCmd = 0;

  bool seesTape = irL || irC || irR;

  if (seesTape) {
    lostSince = 0;
  }

  // CENTER ONLY: STRAIGHT
  if (!irL && irC && !irR) {

    filteredError *= 0.65;

    int correction = constrain(
      (int)(KP * filteredError),
      -MAX_CORRECTION,
      MAX_CORRECTION
    );

    leftCmd = BASE_SPEED;
    rightCmd = BASE_SPEED;
  }

  // LEFT + CENTER: GENTLE LEFT
  else if (irL && irC && !irR) {

    lastSide = -1;

    filteredError =
      0.6 * filteredError - 0.4;

    int correction = constrain(
      (int)(KP * filteredError),
      -MAX_CORRECTION,
      MAX_CORRECTION
    );

    leftCmd = BASE_SPEED + correction;
    rightCmd = BASE_SPEED - correction;
  }

  // CENTER + RIGHT: GENTLE RIGHT
  else if (!irL && irC && irR) {

    lastSide = 1;

    filteredError =
      0.6 * filteredError + 0.4;

    int correction = constrain(
      (int)(KP * filteredError),
      -MAX_CORRECTION,
      MAX_CORRECTION
    );

    leftCmd = BASE_SPEED + correction;
    rightCmd = BASE_SPEED - correction;
  }

  // LEFT ONLY: SHARP LEFT
  else if (irL && !irC && !irR) {

    lastSide = -1;
    filteredError = -2.0;

    leftCmd = CORNER_SLOW;
    rightCmd = CORNER_FAST;
  }

  // RIGHT ONLY: SHARP RIGHT
  else if (!irL && !irC && irR) {

    lastSide = 1;
    filteredError = 2.0;

    leftCmd = CORNER_FAST;
    rightCmd = CORNER_SLOW;
  }

  // ALL THREE: STRAIGHT
  else if (irL && irC && irR) {

    filteredError *= 0.5;

    leftCmd = BASE_SPEED;
    rightCmd = BASE_SPEED;
  }

  // LEFT + RIGHT BUT NOT CENTER
  // Ambiguous line/distractor detection
  else if (irL && !irC && irR) {

    filteredError *= 0.5;

    leftCmd = 75;
    rightCmd = 55;
  }

  // NO SENSORS DETECT TAPE
  else {

    if (lostSince == 0) {
      lostSince = millis();
    }

    unsigned long lostTime =
      millis() - lostSince;

    // Briefly continue with reduced speeds
    if (lostTime < LOST_GRACE_MS) {

      leftCmd = min(previousLeft, 65);
      rightCmd = min(previousRight, 55);
    }

    // Stop if line is lost too long
    else if (lostTime >= MAX_SEARCH_MS) {

      stoppedForLostLine = true;
      stopNow();
      return;
    }

    // Search toward last known left
    else if (lastSide == -1) {

      leftCmd = 0;
      rightCmd = SEARCH_SPEED;
    }

    // Search toward last known right
    else if (lastSide == 1) {

      leftCmd = SEARCH_SPEED;
      rightCmd = 0;
    }

    // Unknown direction: stop
    else {

      leftCmd = 0;
      rightCmd = 0;
    }
  }

  previousLeft = leftCmd;
  previousRight = rightCmd;

  driveSmooth(leftCmd, rightCmd);
}

// =====================================
// 9. DEBUGGING
// =====================================

void printDebug() {

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 100) {
    return;
  }

  lastPrint = millis();

  Serial.print("L:");
  Serial.print(irL);

  Serial.print(" C:");
  Serial.print(irC);

  Serial.print(" R:");
  Serial.print(irR);

  Serial.print(" | LM:");
  Serial.print(leftMotorSpeed);

  Serial.print(" RM:");
  Serial.print(rightMotorSpeed);

  Serial.print(" | Side:");
  Serial.println(lastSide);

  if (stoppedForLostLine) {
    Serial.println("STOPPED: LINE LOST");
  }
}

void stopNow() {
  // Reset motor speeds
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;

  // Stop left motor
  motorOutput(0, ENB, IN3, IN4, REVERSE_LEFT);

  // Stop right motor
  motorOutput(0, ENA, IN1, IN2, REVERSE_RIGHT);
}

// =====================================
// 10. SETUP
// =====================================

void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);

  stopNow();

  Serial.println("COGS 300 LAB 7");
  Serial.println("NO MOTOR BOOST");
  Serial.println("LEFT BASE: 100");
  Serial.println("RIGHT BASE: 70");
  Serial.println("Starting in 3 seconds...");

  delay(START_DELAY);
}

// =====================================
// 11. MAIN LOOP
// =====================================

void loop() {

  updateIR();

  followLine();

  printDebug();

  delay(LOOP_DELAY);
}
