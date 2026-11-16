
/*
  COGS 300 - LAB 7
  SMOOTH WHITE-TAPE LINE FOLLOWER
  Arduino UNO R4 WiFi

  3 IR sensors, moderately spaced apart.
  Slow driving, gentle steering, corner recovery.
  No reversing motors.

  IR:
    Left   A0
    Center A1
    Right  A2

  Motors:
    Right: ENA 9, IN1 8, IN2 7
    Left:  ENB 3, IN3 5, IN4 4
*/

#include <Arduino.h>

// ========================================
// PINS
// ========================================

// Right motor
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Left motor
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

// IR sensors
const int IR_LEFT   = A0;
const int IR_CENTER = A1;
const int IR_RIGHT  = A2;

// Original confirmed motor orientation
const bool REVERSE_LEFT  = true;
const bool REVERSE_RIGHT = false;

// Set false ONLY if readings are inverted
const bool IR_ACTIVE_LOW = true;

// ========================================
// SPEED TUNING
// ========================================

const int BASE_SPEED = 60;

// Gentle correction
const int GENTLE_SLOW = 45;
const int GENTLE_FAST = 75;

// Strong correction
const int SHARP_SLOW = 0;
const int SHARP_FAST = 100;

// Lost-line recovery
const int SEARCH_SPEED = 85;

// Left motor compensation
const int LEFT_MOTOR_BOOST = 35;

// Smoothing
const int ACCEL_STEP = 12;

// Timings
const unsigned long START_DELAY = 3000;
const unsigned long LOOP_DELAY = 10;

// Short allowance for missed sensor readings
const unsigned long LOST_GRACE_MS = 30;

// Maximum time spent searching
const unsigned long MAX_SEARCH_MS = 1200;

// ========================================
// STATE
// ========================================

bool irL = false;
bool irC = false;
bool irR = false;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// -1 = line last seen left
//  0 = no direction known
// +1 = line last seen right
int lastSide = 0;

// Used to keep track of line loss
unsigned long lostSince = 0;

// Last valid driving commands
int previousLeft = 0;
int previousRight = 0;

// ========================================
// SENSOR FUNCTIONS
// ========================================

bool readIR(int pin) {
  int reading = digitalRead(pin);

  return IR_ACTIVE_LOW
    ? (reading == LOW)
    : (reading == HIGH);
}

void updateIR() {
  irL = readIR(IR_LEFT);
  irC = readIR(IR_CENTER);
  irR = readIR(IR_RIGHT);
}

// ========================================
// MOTOR OUTPUT
// ========================================

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

// ========================================
// SMOOTH SPEED CONTROL
// ========================================

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

  // Compensation for weaker left motor
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
    ENB,
    IN3,
    IN4,
    REVERSE_LEFT
  );

  motorOutput(
    rightMotorSpeed,
    ENA,
    IN1,
    IN2,
    REVERSE_RIGHT
  );
}

void stopNow() {
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;

  motorOutput(0, ENB, IN3, IN4, REVERSE_LEFT);
  motorOutput(0, ENA, IN1, IN2, REVERSE_RIGHT);
}

// ========================================
// LINE FOLLOWING
// ========================================

void followLine() {

  int leftCmd = 0;
  int rightCmd = 0;

  bool seesTape = irL || irC || irR;

  // Reset line-loss timer when tape is seen
  if (seesTape) {
    lostSince = 0;
  }

  // --------------------------------------
  // CENTER ONLY: STRAIGHT
  // --------------------------------------

  if (!irL && irC && !irR) {

    leftCmd = BASE_SPEED;
    rightCmd = BASE_SPEED;
  }

  // --------------------------------------
  // LEFT + CENTER: GENTLE LEFT
  // --------------------------------------

  else if (irL && irC && !irR) {

    lastSide = -1;

    leftCmd = GENTLE_SLOW;
    rightCmd = GENTLE_FAST;
  }

  // --------------------------------------
  // CENTER + RIGHT: GENTLE RIGHT
  // --------------------------------------

  else if (!irL && irC && irR) {

    lastSide = 1;

    leftCmd = GENTLE_FAST;
    rightCmd = GENTLE_SLOW;
  }

  // --------------------------------------
  // LEFT ONLY: STRONG LEFT
  // --------------------------------------

  else if (irL && !irC && !irR) {

    lastSide = -1;

    leftCmd = SHARP_SLOW;
    rightCmd = SHARP_FAST;
  }

  // --------------------------------------
  // RIGHT ONLY: STRONG RIGHT
  // --------------------------------------

  else if (!irL && !irC && irR) {

    lastSide = 1;

    leftCmd = SHARP_FAST;
    rightCmd = SHARP_SLOW;
  }

  // --------------------------------------
  // ALL THREE: GO STRAIGHT
  // --------------------------------------

  else if (irL && irC && irR) {

    leftCmd = BASE_SPEED;
    rightCmd = BASE_SPEED;
  }

  // --------------------------------------
  // LEFT + RIGHT, CENTER OFF
  // Ambiguous: possible junction/distractor
  // --------------------------------------

  else if (irL && !irC && irR) {

    // Don't suddenly select a different line
    leftCmd = 45;
    rightCmd = 45;
  }

  // --------------------------------------
  // NO SENSORS: LOST LINE
  // --------------------------------------

  else {

    if (lostSince == 0) {
      lostSince = millis();
    }

    unsigned long lostTime = millis() - lostSince;

    // Briefly continue the previous direction
    if (lostTime < LOST_GRACE_MS) {

      leftCmd = min(previousLeft, 45);
      rightCmd = min(previousRight, 45);
    }

    // Give up after prolonged line loss
    else if (lostTime > MAX_SEARCH_MS) {

      leftCmd = 0;
      rightCmd = 0;
    }

    // Search LEFT without reversing
    else if (lastSide == -1) {

      leftCmd = 0;
      rightCmd = SEARCH_SPEED;
    }

    // Search RIGHT without reversing
    else if (lastSide == 1) {

      leftCmd = SEARCH_SPEED;
      rightCmd = 0;
    }

    // No previous direction: stop
    else {

      leftCmd = 0;
      rightCmd = 0;
    }
  }

  previousLeft = leftCmd;
  previousRight = rightCmd;

  driveSmooth(leftCmd, rightCmd);
}

// ========================================
// DEBUGGING
// ========================================

void printDebug() {

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 150) {
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

  Serial.print(" | LastSide:");
  Serial.println(lastSide);
}

// ========================================
// SETUP
// ========================================

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
  Serial.println("SMOOTH LINE FOLLOWER");
  Serial.println("Starting in 3 seconds");

  delay(START_DELAY);
}

// ========================================
// MAIN LOOP
// ========================================

void loop() {

  updateIR();

  followLine();

  printDebug();

  delay(LOOP_DELAY);
}
