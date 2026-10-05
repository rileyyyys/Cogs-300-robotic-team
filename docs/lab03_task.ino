int enA = 9;
int in1 = 8;
int in2 = 7;

int in3 = 5;
int in4 = 4;
int enB = 3;

int normalSpeed = 200;
int gentleSpeed = 160;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  Serial.begin(9600);

  Serial.println("Robot Controls:");
  Serial.println("f = straight");
  Serial.println("q = gentle left");
  Serial.println("l = sharp left");
  Serial.println("e = gentle right");
  Serial.println("r = sharp right");
  Serial.println("b = backwards");
  Serial.println("s = stop");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'f') {
      straight();
      Serial.println("Straight");
    }

    else if (command == 'q') {
      gentleLeft();
      Serial.println("Gentle Left");
    }

    else if (command == 'l') {
      sharpLeft();
      Serial.println("Sharp Left");
    }

    else if (command == 'e') {
      gentleRight();
      Serial.println("Gentle Right");
    }

    else if (command == 'r') {
      sharpRight();
      Serial.println("Sharp Right");
    }

    else if (command == 'b') {
      backwards();
      Serial.println("Backwards");
    }

    else if (command == 's') {
      stopRobot();
      Serial.println("Stop");
    }
  }
}


// STRAIGHT
void straight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, normalSpeed);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, normalSpeed);
}


// GENTLE LEFT
void gentleLeft() {
  // Left motor slower
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, gentleSpeed);

  // Right motor normal speed
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, normalSpeed);
}


// SHARP LEFT
void sharpLeft() {
  // Left motor stopped
  analogWrite(enA, 0);

  // Right motor moves forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, normalSpeed);
}


// GENTLE RIGHT
void gentleRight() {
  // Left motor normal speed
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, normalSpeed);

  // Right motor slower
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, gentleSpeed);
}


// SHARP RIGHT
void sharpRight() {
  // Left motor moves forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, normalSpeed);

  // Right motor stopped
  analogWrite(enB, 0);
}


// BACKWARDS
void backwards() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, normalSpeed);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, normalSpeed);
}


// STOP
void stopRobot() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}