
/*
  COGS 300 - LAB 7
  SLOW THREE-SENSOR LINE FOLLOWER
  Arduino UNO R4 WiFi

  Follows WHITE tape.
  Tuned for slower and more controlled movement.
*/

#include <Arduino.h>

// =====================================
// MOTOR PINS
// =====================================

// Right motor
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Left motor
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

// Motor orientation
const bool REVERSE_LEFT = true;
const bool REVERSE_RIGHT = false;

// =====================================
// IR SENSORS
// =====================================

const int IR_LEFT = A0;
const int IR_CENTER = A1;
const int IR_RIGHT = A2;

// Confirmed sensor setting
const bool IR_ACTIVE_LOW = true;

// =====================================
// SPEED SETTINGS
// =====================================

// Slower than previous code
const int BASE_SPEED = 55;

// Gentle steering
const int TURN_SLOW = 30;
const int TURN_FAST = 75;

// Stronger steering
const int SHARP_SLOW = 0;
const int SHARP_FAST = 85;

// Slow recovery when tape is lost
const int SEARCH_SPEED = 65;

// Compensation for weaker left motor
const int LEFT_MOTOR_BOOST = 35;

// Smooth acceleration
const int ACCEL_STEP = 8;

// How often to update steering
const int LOOP_DELAY = 10;

// =====================================
// ROBOT STATE
// =====================================

bool irL = false;
bool irC = false;
bool irR = false;

// -1 = tape last detected left
//  1 = tape last detected right
int lastSide = 0;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

unsigned long lostSince = 0;

// =====================================
// SENSOR READING
// =====================================

bool readIR(int pin) {
  int value = digitalRead(pin);

  if (IR_ACTIVE_LOW) {
    return value == LOW;
  }

  return value == HIGH;
}

void updateIR() {
  irL = readIR(IR_LEFT);
  irC = readIR(IR_CENTER);
  irR = readIR(IR_RIGHT);
}

// =====================================
// MOTOR OUTPUT
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
// SMOOTH SPEED CONTROL
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

  // Boost weaker left motor
  if (leftTarget > 0) {
    leftTarget += LEFT_MOTOR_BOOST;
  }

  leftTarget = constrain(leftTarget, -255, 255);
  rightTarget = constrain(rightTarget, -255, 255);

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

// =====================================
// LINE FOLLOWING ALGORITHM
// =====================================

void followLine() {

  int leftCmd = 0;
  int rightCmd = 0;

  bool seesTape = irL || irC || irR;

  if (seesTape) {
    lostSince = 0;
  }

  // CENTER ONLY
  // Robot is aligned: drive straight
  if (!irL && irC && !irR) {

    leftCmd = BASE_SPEED;
    rightCmd = BASE_SPEED;
  }

  // LEFT + CENTER
  // Tape moving left: gently steer LEFT
  else if (irL && irC && !irR) {

    lastSide = -1;

    leftCmd = TURN_SLOW;
    rightCmd = TURN_FAST;
  }

  // CENTER + RIGHT
  // Tape moving right: gently steer RIGHT
  else if (!irL && irC && irR) {

    lastSide = 1;

    leftCmd = TURN_FAST;
    rightCmd = TURN_SLOW;
  }

  // LEFT ONLY
  // Stronger LEFT correction
  else if (irL && !irC && !irR) {

    lastSide = -1;

    leftCmd = SHARP_SLOW;
    rightCmd = SHARP_FAST;
  }

  // RIGHT ONLY
  // Stronger RIGHT correction
  else if (!irL && !irC && irR) {

    lastSide = 1;

    leftCmd = SHARP_FAST;
    rightCmd = SHARP_SLOW;
  }

  // ALL THREE
  // Wide white tape or intersection
  else if (irL && irC && irR) {

    leftCmd = BASE_SPEED;
    rightCmd = BASE_SPEED;
  }

  // LEFT + RIGHT, NO CENTER
  else if (irL && !irC && irR) {

    leftCmd = 45;
    rightCmd = 45;
  }

  // NO TAPE DETECTED
  else {

    if (lostSince == 0) {
      lostSince = millis();
    }

    // Very brief loss: continue slowly
    if (millis() - lostSince < 60) {

      leftCmd = 40;
      rightCmd = 40;
    }

    // Search in last known direction
    else if (lastSide == -1) {

      leftCmd = -SEARCH_SPEED;
      rightCmd = SEARCH_SPEED;
    }

    else if (lastSide == 1) {

      leftCmd = SEARCH_SPEED;
      rightCmd = -SEARCH_SPEED;
    }

    // Don't drive off if no line found yet
    else {

      leftCmd = 0;
      rightCmd = 0;
    }
  }

  driveSmooth(leftCmd, rightCmd);
}

// =====================================
// DEBUG OUTPUT
// =====================================

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

  Serial.print(" | Side:");
  Serial.println(lastSide);
}

// =====================================
// SETUP
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
  Serial.println("SLOW LINE FOLLOWER");
  Serial.println("Starting in 3 seconds");

  delay(3000);
}

// =====================================
// MAIN LOOP
// =====================================

void loop() {

  updateIR();

  followLine();

  printDebug();

  delay(LOOP_DELAY);
}
