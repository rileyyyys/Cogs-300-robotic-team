/*
  COGS 300 - WALL MAZE (left-hand rule)
  WAVGAT (LGT8F328) + L298N + front ultrasonic on a servo

  Built on your PD wall-following sketch: same pins, same setMotors(),
  same PD gains, speeds and thresholds you tuned. It keeps the LEFT wall
  at TARGET_DISTANCE:
    - wall ahead          -> turn right (until the way is clear)
    - left wall disappears -> drive past the corner, turn left, find the wall
  Following one wall like this gets through any maze whose walls connect.

  CHANGES FROM THE WALL-FOLLOWING SKETCH
  1. Servo held pointing straight ahead (front sensor rides on it).
     Hand-made pulses, no Servo library, so pin 9 PWM still works.
  2. Left turns drive forward PASS_CORNER_MS first, so the back of the
     robot clears the wall end instead of clipping it.
  3. After a left turn it drives forward looking for the new wall. If none
     appears (going round the end of a thin wall) it turns left again.
  4. Checks the front sensor DURING each move and stops early if a wall
     comes up, instead of driving blind for 550 ms.
  5. A missing left echo counts as "opening on the left" instead of
     stopping the robot.
  6. A sudden jump in the left reading is treated as a corner (drive
     straight and confirm) instead of steering hard toward it.
  7. Angled walls: an ultrasonic echo bounces away from a wall hit at a
     steep angle, so the sensor reads nothing (-1). When straight ahead is
     empty or far, the servo glances 25 degrees left and right too.

  Wiring: unchanged. Servo orange -> pin 2, red -> 5V, brown -> GND.

  MODES:
    MODE_SENSOR_TEST  No driving. Servo centred. Prints left + front.
                      Check the front sensor points straight ahead.
    MODE_MAZE         Solve the maze.
*/

// ==========================================
// MODE
// ==========================================

enum Mode { MODE_SENSOR_TEST, MODE_MAZE };
const Mode MODE = MODE_SENSOR_TEST;   // <-- change to MODE_MAZE

// ---------- MOTOR PINS ----------
const int ENA = 9;   // right motor
const int IN1 = 8;
const int IN2 = 7;
const int ENB = 3;   // left motor
const int IN3 = 5;
const int IN4 = 4;

// ---------- ULTRASONIC PINS ----------
const int LEFT_TRIG = 13;
const int LEFT_ECHO = 12;
const int FRONT_TRIG = 11;   // on the servo
const int FRONT_ECHO = 10;

// ---------- SERVO ----------
const int SERVO_PIN = 2;
const int SERVO_CENTER = 90;   // same value that worked in the sweep sketch
const int GLANCE_DEG = 25;          // look this far left/right for angled walls
const float GLANCE_IF_OVER_CM = 40; // only glance when straight ahead is empty or further than this
const int GLANCE_SETTLE_MS = 120;   // servo travel + settle time per glance

// ---------- WALL FOLLOWING (your values) ----------
const float TARGET_DISTANCE = 10.0;   // raise to 12-15 if the front corner scrapes the wall
const int BASE_SPEED = 70;
const int MAX_CORRECTION = 20;
const int MAX_SPEED = 130;
const int LEFT_MOTOR_BOOST = 17;
const float KP = 1.5;
const float KD = 3.0;

// ---------- RIGHT TURN (your values) ----------
const float TURN_THRESHOLD = 15.0;   // wall this close ahead -> turn right
const int TURN_SPEED = 75;
const int TURN_PAUSE = 300;
const int EXTRA_TURN_TIME = 350;
const unsigned long MAX_RIGHT_TURN_MS = 4000;   // safety: never spin forever

// ---------- LEFT TURN ----------
const float LEFT_TURN_THRESHOLD = 20.0;  // left wall further than this = opening (keep ~10 above TARGET)
const int LEFT_TURN_CONFIRMATIONS = 3;
const float LEFT_JUMP_CM = 5.0;          // left reading jumped this much in one step = corner
const int PASS_CORNER_MS = 450;          // drive past the wall end before turning
const int LEFT_TURN_DURATION = 450;      // your value (about 90 degrees)
const int FIND_WALL_MS = 900;            // after turning, drive this long looking for a wall
const int MAX_LEFT_TURNS_IN_ROW = 2;     // 2 = can go round the end of a thin wall

// ---------- MOVEMENT TIMING ----------
const int MOVE_TIME = 550;      // your value: drive time between measurements
const int MEASURE_DELAY = 30;
const int FRONT_CHECK_MS = 70;  // how often the front is checked while moving

// ---------- VARIABLES ----------
float previousDistance = TARGET_DISTANCE;
bool firstReading = true;
int leftOpeningCount = 0;

// ==========================================
// SERVO
// ==========================================

int servoOffsetNow = 0;   // where the sensor points: 0 ahead, + right, - left

// Point the sensor offsetDeg from straight ahead (+ right, - left),
// sending pulses for holdMs. Flip the sign here if it glances the wrong way.
void servoPoint(int offsetDeg, int holdMs) {
  servoOffsetNow = offsetDeg;
  int angle = constrain(SERVO_CENTER - offsetDeg, 0, 180);
  int pulseUs = 544 + (long)angle * (2400 - 544) / 180;
  unsigned long start = millis();
  do {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(SERVO_PIN, LOW);
    delay(20 - pulseUs / 1000);
  } while (millis() - start < (unsigned long)holdMs);
}

void servoCenter(int holdMs) {
  servoPoint(0, holdMs);
}

// ==========================================
// MOTORS (from your wall-following code)
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
  bool leftForward = !(leftSpeed >= 0);   // left motor wired reversed
  digitalWrite(IN3, leftForward ? HIGH : LOW);
  digitalWrite(IN4, leftForward ? LOW : HIGH);

  bool rightForward = (rightSpeed >= 0);
  digitalWrite(IN1, rightForward ? HIGH : LOW);
  digitalWrite(IN2, rightForward ? LOW : HIGH);

  analogWrite(ENB, constrain(abs(leftSpeed), 0, 255));
  analogWrite(ENA, constrain(abs(rightSpeed), 0, 255));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void driveStraight() {
  setMotors(BASE_SPEED + LEFT_MOTOR_BOOST, BASE_SPEED);
}

// ==========================================
// ULTRASONIC
// ==========================================

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 12000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

bool wallAhead() {
  float f = readDistance(FRONT_TRIG, FRONT_ECHO);
  return f > 0 && f <= TURN_THRESHOLD;
}

// Front distance that also catches ANGLED walls. An ultrasonic echo bounces
// away from a wall hit at a steep angle, so the sensor sees nothing (-1).
// If straight ahead is empty or far, glance left and right with the servo
// and take the closest wall any of the three directions sees.
float frontScan() {
  float best = readDistance(FRONT_TRIG, FRONT_ECHO);
  if (best > 0 && best <= GLANCE_IF_OVER_CM) return best;

  int offsets[2] = { -GLANCE_DEG, GLANCE_DEG };
  for (int i = 0; i < 2; i++) {
    servoPoint(offsets[i], GLANCE_SETTLE_MS);
    float d = readDistance(FRONT_TRIG, FRONT_ECHO);
    // a wall seen at an angle is that far along the beam; how far ahead is it?
    if (d > 0) d = d * cos(GLANCE_DEG * PI / 180.0);
    if (d > 0 && (best < 0 || d < best)) best = d;
  }
  servoCenter(GLANCE_SETTLE_MS);
  return best;
}

// Keep the motors as they are for ms, checking the front as we go.
// Returns true if a wall came up (and stops the motors).
bool moveWatchingFront(int ms) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) {
    delay(FRONT_CHECK_MS);
    if (wallAhead()) {
      stopMotors();
      return true;
    }
  }
  return false;
}

// ==========================================
// TURNS
// ==========================================

void turnRight() {
  Serial.println("Wall ahead: turning right");
  stopMotors();
  delay(TURN_PAUSE);

  setMotors(TURN_SPEED + LEFT_MOTOR_BOOST, -TURN_SPEED);
  unsigned long start = millis();
  while (millis() - start < MAX_RIGHT_TURN_MS) {
    delay(50);
    float f = readDistance(FRONT_TRIG, FRONT_ECHO);
    if (f < 0 || f > TURN_THRESHOLD) break;   // clear (no echo = open space)
  }
  delay(EXTRA_TURN_TIME);
  stopMotors();
  delay(TURN_PAUSE);
  firstReading = true;
}

// Opening on the left: pass the corner, turn left, find the new wall.
void turnLeftAroundCorner() {
  for (int turns = 1; turns <= MAX_LEFT_TURNS_IN_ROW; turns++) {
    Serial.println("Opening on the left: passing corner");

    // 1. Drive past the wall end so the back of the robot clears it.
    //    If a wall comes up ahead, just stop: the opening on the left still
    //    wins (left-hand rule), so we turn left rather than right.
    driveStraight();
    moveWatchingFront(PASS_CORNER_MS);
    stopMotors();
    delay(TURN_PAUSE);

    // 2. Pivot left (your timing)
    Serial.println("Turning left");
    setMotors(-(TURN_SPEED + LEFT_MOTOR_BOOST), TURN_SPEED);
    delay(LEFT_TURN_DURATION);
    stopMotors();
    delay(TURN_PAUSE);

    // 3. Drive forward until the left wall shows up again
    unsigned long start = millis();
    while (millis() - start < (unsigned long)FIND_WALL_MS) {
      driveStraight();
      if (moveWatchingFront(150)) { turnRight(); return; }
      stopMotors();
      delay(MEASURE_DELAY);
      float l = readDistance(LEFT_TRIG, LEFT_ECHO);
      if (l > 0 && l <= LEFT_TURN_THRESHOLD) {
        Serial.println("Found the new wall");
        firstReading = true;
        return;
      }
    }
    // No wall yet: we're probably going round the end of a thin wall.
    // Loop round and turn left again.
  }
  Serial.println("Still no wall - carrying on straight");
  firstReading = true;
}

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LEFT_TRIG, OUTPUT);  pinMode(LEFT_ECHO, INPUT);
  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(SERVO_PIN, OUTPUT);

  stopMotors();
  servoCenter(600);   // point the front sensor straight ahead
  Serial.println("Wall maze ready");
  if (MODE == MODE_MAZE) delay(2000);
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  // Keep the servo pointing forward (also gives the motors a stop to measure)
  stopMotors();
  servoCenter(MEASURE_DELAY + 10);

  float leftDistance = readDistance(LEFT_TRIG, LEFT_ECHO);
  delay(30);
  float frontDistance = frontScan();   // also catches angled walls

  if (MODE == MODE_SENSOR_TEST) {
    Serial.print("Left: ");  Serial.print(leftDistance);
    Serial.print(" cm | Front: "); Serial.print(frontDistance);
    Serial.println(" cm");
    delay(300);
    return;
  }

  // ---------- 1. Wall ahead: turn right ----------
  if (frontDistance > 0 && frontDistance <= TURN_THRESHOLD) {
    turnRight();
    leftOpeningCount = 0;
    return;
  }

  // ---------- 2. Opening on the left (or no echo): turn left ----------
  // A sudden jump in the left reading also counts: at a wall end the sensor
  // often catches the edge and reads just under the threshold, and steering
  // toward that "far wall" drives the robot into the corner.
  bool jumped = !firstReading && leftDistance > 0 &&
                leftDistance - previousDistance > LEFT_JUMP_CM;
  if (leftDistance < 0 || leftDistance > LEFT_TURN_THRESHOLD || jumped) {
    leftOpeningCount++;
    if (leftOpeningCount >= LEFT_TURN_CONFIRMATIONS) {
      leftOpeningCount = 0;
      turnLeftAroundCorner();
      return;
    }
    // Not confirmed yet: creep forward a little and look again
    driveStraight();
    if (moveWatchingFront(150)) turnRight();
    return;
  }
  leftOpeningCount = 0;

  // ---------- 3. PD wall following (your controller) ----------
  float error = leftDistance - TARGET_DISTANCE;
  float distanceChange = firstReading ? 0 : leftDistance - previousDistance;
  float correction = constrain(KP * error + KD * distanceChange,
                               -MAX_CORRECTION, MAX_CORRECTION);
  previousDistance = leftDistance;
  firstReading = false;

  int leftSpeed = constrain(BASE_SPEED - correction + LEFT_MOTOR_BOOST, 0, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED + correction, 0, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
  if (moveWatchingFront(MOVE_TIME)) {
    turnRight();
    return;
  }
  stopMotors();

  Serial.print("Left: ");   Serial.print(leftDistance);
  Serial.print(" cm | Front: "); Serial.print(frontDistance);
  Serial.print(" cm | Correction: "); Serial.println(correction);
}
