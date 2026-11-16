/*
  COGS 300 - LAB 7
  LINE FOLLOWER v3 (fixes swinging + circling in sketch_linefollowing2)
  Arduino UNO R4 WiFi

  Same pins, same motor orientation, still never reverses a motor.

  WHAT CHANGED FROM v2, AND WHY
  1. LEFT_MOTOR_BOOST (+35) removed. Adding a fixed +35 to the left wheel
     broke the left turns: "gentle left" asked for L45 / R75, but after the
     boost that became L80 / R75, which steers RIGHT. Only the sharp left
     (where the left wheel is 0 and gets no boost) actually turned left, so
     the robot overshot right, then snapped hard left, then repeated.
     Now each motor gets a percentage trim (LEFT_TRIM / RIGHT_TRIM).

  2. Dead zone fixed. DC motors don't turn at all below some PWM (often
     50-70). v2 drove at 45-75, right at that edge, so "slow down a bit"
     often meant "stop that wheel", turning every gentle correction into a
     sharp one. Now speeds are 0-100 % and get mapped onto MIN_PWM..255.

  3. Steering is a PD controller instead of 7 fixed cases.
       P: steer harder the further the line is off-centre.
       D: when the line is moving back toward the centre, ease off before
          it gets there. This is the part that stops the swinging.

  4. ACCEL_STEP 12 -> 40. The ramp took ~100 ms to change speed, so every
     correction arrived late and overshot.

  5. Lost-line search that can't loop forever:
     - Lost while centred = a gap or the end of the tape: drive straight
       briefly instead of spinning toward a stale side.
     - Lost off one side = sharp corner: turn toward the side the line
       most recently left from, for at most MAX_SEARCH_MS, then stop.
       At a 90-degree corner the last reading is often centre-only, so it
       also remembers a side seen in the last SIDE_MEMORY_MS.

  IR:      Left A0, Center A1, Right A2
  Motors:  Right ENA 9 / IN1 8 / IN2 7     Left ENB 3 / IN3 5 / IN4 4

  CALIBRATE IN THIS ORDER (MODE at the top of the code):
    MODE_CALIBRATE_STRAIGHT: robot drives straight at CRUISE % for 3 s.
       Curves left  -> left wheel is weak:  lower RIGHT_TRIM (e.g. 0.90)
       Curves right -> right wheel is weak: lower LEFT_TRIM
    Find MIN_PWM: raise it until both wheels just start turning on the floor
       at the smallest speeds (try MODE_CALIBRATE_STRAIGHT with CRUISE 10).
    MODE_FOLLOW: tune KP first (D = 0), then add KD until the swinging stops.
*/

#include <Arduino.h>

// ========================================
// MODE
// ========================================

enum Mode { MODE_FOLLOW, MODE_CALIBRATE_STRAIGHT, MODE_SENSOR_TEST };
const Mode MODE = MODE_FOLLOW;

// ========================================
// PINS (unchanged from v2)
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

const bool IR_ACTIVE_LOW = true;

// ========================================
// MOTOR CALIBRATION
// ========================================

// Lowest PWM at which a wheel actually turns with the robot on the floor.
const int MIN_PWM = 55;

// Multiply each wheel's power. Keep both <= 1.0: slow the STRONG wheel
// down rather than trying to boost the weak one past 255.
const float LEFT_TRIM  = 1.00;
const float RIGHT_TRIM = 1.00;

// ========================================
// SPEED / STEERING TUNING (speeds in %, 0-100)
// ========================================

const float CRUISE = 20;          // straight-line speed
const float MIN_CRUISE = 18;      // slowest it slows to in a hard turn
const float SLOWDOWN_PER_ERR = 6; // % slower per unit of error

// Error is -2 .. +2 (see readError). Steering is in % of wheel difference.
const float KP = 8;      // steer per unit of error
const float KD = 12;     // damping: push back while the error is shrinking
const float D_DECAY = 0.85;  // how long the damping kick lasts (0.85 ~ 60 ms)

// Lost-line search
const float SEARCH_FAST = 32;     // outer wheel while searching
const float SEARCH_SLOW = 0;      // inner wheel (0 = pivot on that wheel)
const unsigned long GAP_MS = 250;          // straight ahead if lost while centred
const unsigned long SIDE_MEMORY_MS = 400;  // "recently saw it on a side" window
const unsigned long MAX_SEARCH_MS = 1500;  // then give up and stop

// Ramp limit per loop (PWM units). Bigger = snappier.
const int ACCEL_STEP = 40;

const unsigned long START_DELAY = 3000;
const unsigned long LOOP_DELAY = 5;

// ========================================
// STATE
// ========================================

bool irL = false, irC = false, irR = false;

int leftPwm = 0;
int rightPwm = 0;

float lastError = 0;      // most recent error while the line was visible
float dTerm = 0;          // decaying damping term
int lastSide = 0;         // -1 / +1: side the line most recently left from
unsigned long lastSideTime = 0;   // when a side sensor last saw the line

unsigned long lostSince = 0;
bool lost = false;
bool done = false;

// ========================================
// SENSORS
// ========================================

bool readIR(int pin) {
  int reading = digitalRead(pin);
  return IR_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
}

void updateIR() {
  irL = readIR(IR_LEFT);
  irC = readIR(IR_CENTER);
  irR = readIR(IR_RIGHT);
}

// Where is the line, relative to the robot?
//   -2 = far left (only L sees it)   -1 = left (L+C)   0 = centred (C)
//   +1 = right (C+R)                 +2 = far right (only R)
// Returns false if no sensor sees tape.
bool readError(float &error) {
  if (!irL && !irC && !irR) return false;

  if      ( irL && !irC && !irR) error = -2;
  else if ( irL &&  irC && !irR) error = -1;
  else if (!irL &&  irC && !irR) error =  0;
  else if (!irL &&  irC &&  irR) error = +1;
  else if (!irL && !irC &&  irR) error = +2;
  else                           error = 0;  // all three, or L+R: junction -> straight
  return true;
}

// ========================================
// MOTORS
// ========================================

void motorOutput(int speedValue, int enablePin, int pin1, int pin2, bool reverse) {
  speedValue = constrain(speedValue, -255, 255);
  if (reverse) speedValue = -speedValue;

  if (speedValue > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, speedValue);
  } else if (speedValue < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(enablePin, -speedValue);
  } else {
    analogWrite(enablePin, 0);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

// 0 % -> stopped.  1-100 % -> MIN_PWM..255, so every non-zero speed
// actually turns the wheel and the speed changes evenly.
int percentToPwm(float pct, float trim) {
  if (pct <= 0.5) return 0;
  pct = constrain(pct, 0, 100);
  return (int)((MIN_PWM + (255 - MIN_PWM) * pct / 100.0) * trim);
}

int approach(int current, int target) {
  if (current < target) return min(current + ACCEL_STEP, target);
  if (current > target) return max(current - ACCEL_STEP, target);
  return target;
}

void drive(float leftPct, float rightPct) {
  int leftTarget  = percentToPwm(leftPct, LEFT_TRIM);
  int rightTarget = percentToPwm(rightPct, RIGHT_TRIM);

  // Starting from 0, jump straight to MIN_PWM; below that nothing moves anyway.
  if (leftPwm == 0 && leftTarget > 0)   leftPwm = min(leftTarget, MIN_PWM);
  if (rightPwm == 0 && rightTarget > 0) rightPwm = min(rightTarget, MIN_PWM);

  leftPwm = approach(leftPwm, leftTarget);
  rightPwm = approach(rightPwm, rightTarget);

  // A ramp passing through the dead zone just stalls the wheel; cut to 0.
  if (leftTarget == 0 && leftPwm < MIN_PWM)   leftPwm = 0;
  if (rightTarget == 0 && rightPwm < MIN_PWM) rightPwm = 0;

  motorOutput(leftPwm, ENB, IN3, IN4, REVERSE_LEFT);
  motorOutput(rightPwm, ENA, IN1, IN2, REVERSE_RIGHT);
}

void stopNow() {
  leftPwm = 0;
  rightPwm = 0;
  motorOutput(0, ENB, IN3, IN4, REVERSE_LEFT);
  motorOutput(0, ENA, IN1, IN2, REVERSE_RIGHT);
}

// ========================================
// LINE FOLLOWING
// ========================================

void followLine() {
  float error;

  if (readError(error)) {
    // ---- Line visible: PD control ----
    lost = false;

    // D: change in error, kept for a few loops so it actually has an effect
    // (with 3 on/off sensors the error only changes now and then).
    dTerm = dTerm * D_DECAY + KD * (error - lastError);

    float steer = KP * error + dTerm;      // + = turn right

    float speed = CRUISE - SLOWDOWN_PER_ERR * fabs(error);
    speed = max(speed, MIN_CRUISE);

    // Never reverse: clamp each wheel at 0.
    drive(speed + steer, speed - steer);

    lastError = error;
    if (error != 0) {
      lastSide = error < 0 ? -1 : +1;
      lastSideTime = millis();
    }
    return;
  }

  // ---- No sensor sees tape ----
  if (!lost) {
    lost = true;
    lostSince = millis();
    dTerm = 0;
  }
  unsigned long lostTime = millis() - lostSince;

  if (lostTime > MAX_SEARCH_MS) {
    stopNow();
    done = true;
    Serial.println("Line lost: stopped. Press reset.");
    return;
  }

  // Did a side sensor see the line just before we lost it? At a 90-degree
  // corner the last reading is often centre-only for an instant, even
  // though the line was clearly heading off to one side.
  bool sawSideRecently = lastSide != 0 && (lostSince - lastSideTime) < SIDE_MEMORY_MS;

  if (!sawSideRecently && lostTime < GAP_MS) {
    // Lost while centred: probably a gap in the tape. Go straight briefly.
    drive(MIN_CRUISE, MIN_CRUISE);
    return;
  }

  if (lastSide == 0) {           // never seen a side at all: nothing to search
    stopNow();
    done = true;
    Serial.println("End of line.");
    return;
  }

  // Sharp corner (or the gap didn't come back): turn toward where it went.
  if (lastSide < 0) drive(SEARCH_SLOW, SEARCH_FAST);   // left
  else              drive(SEARCH_FAST, SEARCH_SLOW);   // right
}

// ========================================
// DEBUG (Serial Monitor / Plotter at 115200)
// ========================================

void printDebug() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  Serial.print("L:");  Serial.print(irL);
  Serial.print(" C:"); Serial.print(irC);
  Serial.print(" R:"); Serial.print(irR);
  Serial.print(" err:"); Serial.print(lastError);
  Serial.print(" d:");   Serial.print(dTerm);
  Serial.print(" LM:");  Serial.print(leftPwm);
  Serial.print(" RM:");  Serial.print(rightPwm);
  Serial.print(" lost:"); Serial.println(lost);
}

// ========================================
// SETUP / LOOP
// ========================================

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);

  stopNow();
  Serial.println("COGS 300 LAB 7 - line follower v3");
  if (MODE != MODE_SENSOR_TEST) delay(START_DELAY);
}

void loop() {
  if (done) { stopNow(); delay(100); return; }

  updateIR();

  if (MODE == MODE_SENSOR_TEST) {
    stopNow();
  } else if (MODE == MODE_CALIBRATE_STRAIGHT) {
    drive(CRUISE, CRUISE);
    if (millis() > START_DELAY + 3000) { stopNow(); done = true; }
  } else {
    followLine();
  }

  printDebug();
  delay(LOOP_DELAY);
}
