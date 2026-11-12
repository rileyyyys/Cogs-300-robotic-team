/*
  COGS 300 - LAB 07: LINE FOLLOWING (Bunker Bots)
  Arduino UNO R4 WiFi

  Built on docs/sketch_wallfollowing.ino: same confirmed motor wiring,
  same motorOutput()/driveSmooth() helpers, same state-machine style.
  NEW: 2-3 IR line sensors (digital output, tuned with the on-board pot).

  Set MODE below to choose what the robot does:

    MODE_SENSOR_TEST  No driving. Prints what each IR sensor sees.
                      Use this while tuning the pots with a screwdriver:
                      the sensor should read 1 on white tape, 0 on the floor.

    MODE_START_STOP   Lab step 2.
                      START_ON_LINE = false: drive from open floor, stop on the line.
                      START_ON_LINE = true : start on the tape, stop where it ends.

    MODE_PING_PONG    Lab step 3. Bang-bang follower using LEFT + RIGHT sensors:
                      drift forward-left until a sensor leaves the tape, then
                      turn back, "ping-ponging" along the edges of the tape.

    MODE_BAYES        Lab step 4 (maze). Discrete Bayes filter over where the
                      line is under the robot, combined with a P-controller.
                      Faster and smoother than ping-pong, and it remembers
                      which side the line went when every sensor loses it,
                      so it can recover around sharp corners.

  The robot starts START_DELAY ms after power-on (time to put it down).
*/

#include <Arduino.h>

// =====================================
// 0. MODE SELECT
// =====================================

enum Mode { MODE_SENSOR_TEST, MODE_START_STOP, MODE_PING_PONG, MODE_BAYES };

const Mode MODE = MODE_SENSOR_TEST;   // <-- change me

const bool START_ON_LINE = false;     // only used in MODE_START_STOP

// =====================================
// 1. WIRING
// =====================================

// Motors: unchanged from the wall follower ("CONFIRMED WIRING").
// Right motor: Channel A
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Left motor: Channel B
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

const bool REVERSE_LEFT = false;
const bool REVERSE_RIGHT = false;

// Ultrasonics still use 13/12 (left) and 11/10 (front), so the IR
// sensors go on the analog header (read as digital pins).
// IR module wiring: VCC -> 5V, GND -> GND, OUT/DO -> pin below.
const int IR_LEFT = A0;
const int IR_CENTER = A1;   // optional third sensor
const int IR_RIGHT = A2;

const bool HAS_CENTER_SENSOR = true;  // false if you only mounted two

// Most IR modules (TCRT5000 / FC-51 with LM393 comparator) pull OUT LOW
// when they see a reflection (white tape) and the little LED turns on.
// If MODE_SENSOR_TEST shows 1 on the floor and 0 on tape, flip this.
const bool IR_ACTIVE_LOW = true;

// =====================================
// 2. SENSOR GEOMETRY (measure your robot!)
// =====================================

// Sideways distance from the robot's centre line to the left/right sensors.
const float SENSOR_SPACING_CM = 1.5;

// Width of the tape on the course.
const float TAPE_WIDTH_CM = 4.8;

// Ping-pong layout:
//   true  = both L and R sit over the tape when centred (tape wider than gap)
//   false = L and R straddle the tape, just outside its edges
// "Only LEFT sees tape -> turn left" is the same in both layouts; this flag
// only changes what "both on" and "both off" mean.
const bool SENSORS_INSIDE_TAPE = (TAPE_WIDTH_CM > 2 * SENSOR_SPACING_CM);

// =====================================
// 3. SPEEDS / TUNING
// =====================================

const int BASE_SPEED = 110;     // straight-line speed (0-255)
const int MIN_SPEED = 0;
const int MAX_SPEED = 170;

// Ping-pong
const int DRIFT_FAST = 115;     // outer wheel while drifting
const int DRIFT_SLOW = 85;      // inner wheel while drifting
const int CORRECT_SPEED = 110;  // turning back toward the tape

// Spin in place (lost line / sharp corner)
const int PIVOT_SPEED = 115;

// Bayes P-controller: PWM of steering per cm of estimated line offset
const float KP_LINE = 22.0;
const int MAX_CORRECTION = 90;

// Slow down when the line is far off-centre (helps on curves)
const float SLOWDOWN_PER_CM = 0.10;   // 10% slower per cm of offset
const float MIN_SPEED_FACTOR = 0.55;

// Larger than the wall follower: line following needs quick reactions.
const int ACCEL_STEP = 40;

// Timing
const unsigned long START_DELAY = 3000;
const unsigned long LOOP_MS = 10;
const unsigned long LOST_STOP_MS = 2500;   // no tape this long -> finished / give up
const unsigned long FINISH_PAD_MS = 400;   // every sensor on tape this long -> finish pad
const bool STOP_ON_FINISH_PAD = false;     // true if the maze ends on a wide white square

const bool DEBUG_PRINT = true;             // Serial Plotter output (115200 baud)

// =====================================
// 4. STATE
// =====================================

enum State { RUNNING, DONE };
State state = RUNNING;

bool irL = false, irC = false, irR = false;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Which side was the line on last time we knew? -1 = left, +1 = right
int lastSide = 0;

unsigned long allOffSince = 0;
unsigned long allOnSince = 0;
bool startStopArmed = false;

// =====================================
// 5. IR SENSORS
// =====================================

bool readIR(int pin) {
  int v = digitalRead(pin);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void updateIR() {
  irL = readIR(IR_LEFT);
  irR = readIR(IR_RIGHT);
  irC = HAS_CENTER_SENSOR ? readIR(IR_CENTER) : false;
}

bool anyOnTape() { return irL || irR || irC; }

bool allOnTape() {
  return irL && irR && (HAS_CENTER_SENSOR ? irC : true);
}

// Track how long the sensors have been all-off / all-on.
void updateTimers() {
  unsigned long now = millis();
  if (anyOnTape()) allOffSince = now;
  if (!allOnTape()) allOnSince = now;
}

// =====================================
// 6. MOTOR FUNCTIONS (from wall follower)
// =====================================

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

int approachSpeed(int current, int target) {
  if (current < target) return min(current + ACCEL_STEP, target);
  if (current > target) return max(current - ACCEL_STEP, target);
  return target;
}

void driveSmooth(int leftTarget, int rightTarget) {
  leftTarget = constrain(leftTarget, -255, 255);
  rightTarget = constrain(rightTarget, -255, 255);

  leftMotorSpeed = approachSpeed(leftMotorSpeed, leftTarget);
  rightMotorSpeed = approachSpeed(rightMotorSpeed, rightTarget);

  motorOutput(leftMotorSpeed, ENB, IN3, IN4, REVERSE_LEFT);    // LEFT = Channel B
  motorOutput(rightMotorSpeed, ENA, IN1, IN2, REVERSE_RIGHT);  // RIGHT = Channel A
}

void stopNow() {
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
  motorOutput(0, ENB, IN3, IN4, REVERSE_LEFT);
  motorOutput(0, ENA, IN1, IN2, REVERSE_RIGHT);
}

void pivotToward(int side) {
  // side < 0: spin left, side > 0: spin right
  if (side < 0) driveSmooth(-PIVOT_SPEED, PIVOT_SPEED);
  else          driveSmooth(PIVOT_SPEED, -PIVOT_SPEED);
}

void finish(const char* why) {
  stopNow();
  state = DONE;
  Serial.print("DONE: ");
  Serial.println(why);
}

// =====================================
// 7. MODE: SENSOR TEST
// =====================================

void runSensorTest() {
  stopNow();
  Serial.print("L:");   Serial.print(irL);
  Serial.print("\tC:"); Serial.print(HAS_CENTER_SENSOR ? irC : -1);
  Serial.print("\tR:"); Serial.println(irR);
  delay(100);
}

// =====================================
// 8. MODE: START / STOP (lab step 2)
// =====================================

void runStartStop() {
  if (!START_ON_LINE) {
    // Drive across open floor; stop the moment any sensor sees tape.
    if (anyOnTape()) { finish("found line"); return; }
    driveSmooth(BASE_SPEED, BASE_SPEED);
  } else {
    // Start on the tape; drive until it ends.
    if (anyOnTape()) startStopArmed = true;
    if (startStopArmed && millis() - allOffSince > 60) { finish("tape ended"); return; }
    driveSmooth(BASE_SPEED, BASE_SPEED);
  }
}

// =====================================
// 9. MODE: PING-PONG (lab step 3)
// =====================================
//
// Inside-tape layout (both sensors on tape when centred):
//   both on          -> drift forward-LEFT (lab's "forwards and a little bit left")
//   L off, R on      -> line is to the right -> turn RIGHT
//   L on,  R off     -> line is to the left  -> turn LEFT
//   both off         -> lost: spin toward the side we last saw it
//
// Straddle layout (sensors just outside the tape edges):
//   both off         -> centred, drive straight
//   L on / R on      -> turn toward the sensor that sees tape
//   both on          -> junction: go straight

void runPingPong() {
  if (millis() - allOffSince > LOST_STOP_MS) { finish("line lost"); return; }

  bool lineLeft = irL && !irR;
  bool lineRight = irR && !irL;

  if (lineLeft) {
    lastSide = -1;
    driveSmooth(0, CORRECT_SPEED);                 // turn left
  } else if (lineRight) {
    lastSide = +1;
    driveSmooth(CORRECT_SPEED, 0);                 // turn right
  } else if (irL && irR) {
    if (SENSORS_INSIDE_TAPE) driveSmooth(DRIFT_SLOW, DRIFT_FAST);  // drift left
    else                     driveSmooth(BASE_SPEED, BASE_SPEED);  // junction
  } else {
    // both off
    if (SENSORS_INSIDE_TAPE) pivotToward(lastSide == 0 ? +1 : lastSide);
    else                     driveSmooth(BASE_SPEED, BASE_SPEED);
  }
}

// =====================================
// 10. MODE: BAYES FILTER LINE FOLLOWER (lab step 4)
// =====================================
//
// Hidden state x = sideways offset of the line from the robot's centre, in cm
// (+ = line is to the RIGHT of the robot). We keep a probability for each of
// NUM_BINS possible offsets and update it every loop:
//
//   PREDICT: our own steering moves the line relative to us (turning right
//            slides the line left), plus some blur because the track curves
//            and the wheels slip.
//   UPDATE:  multiply by P(sensor readings | x). A sensor at position s should
//            see tape when |x - s| <= TAPE_WIDTH/2, and is right ~90% of the time.
//
// Steering uses the mean of the belief. When every sensor is off the tape,
// the belief keeps drifting out the side the line left from, so the robot
// knows which way to spin to find it again.

const int NUM_BINS = 17;            // offsets -8 .. +8 cm
const float BIN_MIN_CM = -8.0;
const float BIN_STEP_CM = 1.0;

const float P_SENSOR_CORRECT = 0.90;   // how often an IR reading is right
const float BLUR = 0.15;               // spread to each neighbour per step

// How far (cm) the line slides sideways per loop at full wheel difference
// (|left - right| = 255). Measure/tune: spin the robot and watch the belief.
const float SHIFT_CM_PER_LOOP = 0.6;

// Beyond this offset no sensor can see the line -> pivot instead of steering.
const float SENSOR_REACH_CM = SENSOR_SPACING_CM + TAPE_WIDTH_CM / 2.0;

float belief[NUM_BINS];
float tmp[NUM_BINS];

float binCm(int i) { return BIN_MIN_CM + i * BIN_STEP_CM; }

void resetBelief() {
  for (int i = 0; i < NUM_BINS; i++) belief[i] = 1.0 / NUM_BINS;
}

void normalize(float* p) {
  float total = 0;
  for (int i = 0; i < NUM_BINS; i++) total += p[i];
  if (total < 1e-9) { resetBelief(); return; }
  for (int i = 0; i < NUM_BINS; i++) p[i] /= total;
}

// PREDICT step: shift by our own turning, then blur.
void predict(int leftCmd, int rightCmd) {
  // Turning right (left wheel faster) moves the line to the LEFT (negative).
  float shiftBins = -SHIFT_CM_PER_LOOP * (leftCmd - rightCmd) / 255.0 / BIN_STEP_CM;

  // Fractional shift: split each bin between the two bins it lands between.
  for (int i = 0; i < NUM_BINS; i++) tmp[i] = 0;
  for (int i = 0; i < NUM_BINS; i++) {
    float target = i + shiftBins;
    int lo = (int)floor(target);
    float frac = target - lo;
    int a = constrain(lo, 0, NUM_BINS - 1);
    int b = constrain(lo + 1, 0, NUM_BINS - 1);
    tmp[a] += belief[i] * (1.0 - frac);
    tmp[b] += belief[i] * frac;
  }

  // Blur (motion noise / track curvature). Edge bins reflect back in.
  for (int i = 0; i < NUM_BINS; i++) {
    float left = tmp[max(i - 1, 0)];
    float right = tmp[min(i + 1, NUM_BINS - 1)];
    belief[i] = (1.0 - 2 * BLUR) * tmp[i] + BLUR * (left + right);
  }
  normalize(belief);
}

float sensorLikelihood(bool reading, float sensorPosCm, float lineCm) {
  bool shouldSeeTape = fabs(lineCm - sensorPosCm) <= TAPE_WIDTH_CM / 2.0;
  return (reading == shouldSeeTape) ? P_SENSOR_CORRECT : (1.0 - P_SENSOR_CORRECT);
}

// UPDATE step: Bayes rule with each sensor's reading.
void updateBelief() {
  for (int i = 0; i < NUM_BINS; i++) {
    float x = binCm(i);
    float like = sensorLikelihood(irL, -SENSOR_SPACING_CM, x) *
                 sensorLikelihood(irR, +SENSOR_SPACING_CM, x);
    if (HAS_CENTER_SENSOR) like *= sensorLikelihood(irC, 0.0, x);
    belief[i] *= like;
  }
  normalize(belief);
}

float beliefMean() {
  float m = 0;
  for (int i = 0; i < NUM_BINS; i++) m += belief[i] * binCm(i);
  return m;
}

float beliefMaxP() {
  float best = 0;
  for (int i = 0; i < NUM_BINS; i++) best = max(best, belief[i]);
  return best;
}

int lastLeftCmd = 0, lastRightCmd = 0;

void runBayes() {
  if (millis() - allOffSince > LOST_STOP_MS) { finish("line lost / end of course"); return; }
  if (STOP_ON_FINISH_PAD && millis() - allOnSince > FINISH_PAD_MS) { finish("finish pad"); return; }

  predict(lastLeftCmd, lastRightCmd);
  updateBelief();

  float offset = beliefMean();
  int leftCmd, rightCmd;

  if (!anyOnTape() && fabs(offset) > SENSOR_REACH_CM * 0.8) {
    // Line is out past the sensors: spin toward where we believe it went.
    int side = offset > 0 ? +1 : -1;
    leftCmd = side > 0 ? PIVOT_SPEED : -PIVOT_SPEED;
    rightCmd = -leftCmd;
  } else {
    // P-controller on the estimated offset. Line to the right -> turn right.
    int correction = constrain((int)(KP_LINE * offset), -MAX_CORRECTION, MAX_CORRECTION);

    float factor = 1.0 - SLOWDOWN_PER_CM * fabs(offset);
    factor = constrain(factor, MIN_SPEED_FACTOR, 1.0f);
    int base = (int)(BASE_SPEED * factor);

    leftCmd = constrain(base + correction, MIN_SPEED, MAX_SPEED);
    rightCmd = constrain(base - correction, MIN_SPEED, MAX_SPEED);
  }

  driveSmooth(leftCmd, rightCmd);
  // Predict with what the wheels are actually doing (after ramping).
  lastLeftCmd = leftMotorSpeed;
  lastRightCmd = rightMotorSpeed;

  if (DEBUG_PRINT) {
    static int n = 0;
    if (++n >= 5) {   // every 5th loop so Serial doesn't slow the robot
      n = 0;
      Serial.print("offset:");   Serial.print(offset);
      Serial.print("\tconf:");   Serial.print(beliefMaxP() * 10);   // x10 to see on plot
      Serial.print("\tL:");      Serial.print(irL * 5);
      Serial.print("\tC:");      Serial.print(irC * 5);
      Serial.print("\tR:");      Serial.print(irR * 5);
      Serial.print("\tlm:");     Serial.print(leftMotorSpeed / 25.0);
      Serial.print("\trm:");     Serial.println(rightMotorSpeed / 25.0);
    }
  }
}

// =====================================
// 11. SETUP
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
  pinMode(IR_RIGHT, INPUT);
  if (HAS_CENTER_SENSOR) pinMode(IR_CENTER, INPUT);

  stopNow();
  resetBelief();

  if (MODE != MODE_SENSOR_TEST) delay(START_DELAY);

  allOffSince = millis();
  allOnSince = millis();
  Serial.println("Lab 07 line follower ready");
}

// =====================================
// 12. MAIN LOOP
// =====================================

void loop() {
  if (state == DONE) {
    stopNow();
    delay(100);
    return;
  }

  updateIR();
  updateTimers();

  switch (MODE) {
    case MODE_SENSOR_TEST: runSensorTest(); break;
    case MODE_START_STOP:  runStartStop();  break;
    case MODE_PING_PONG:   runPingPong();   break;
    case MODE_BAYES:       runBayes();      break;
  }

  delay(LOOP_MS);
}
