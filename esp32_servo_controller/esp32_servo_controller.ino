#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define hipLOffset 175
#define kneeLOffset 190
#define ankleLOffset 75
#define hipROffset 15
#define kneeROffset 5
#define ankleROffset 35

#define l1 5
#define l2 5.7

#define stepClearance 1
#define stepHeight 10

Servo hipL, hipR, kneeL, kneeR, ankleL, ankleR;
Adafruit_MPU6050 mpu;

const float Z_THRESHOLD_UP   =  2.5;
const float Z_THRESHOLD_DOWN = -2.5;
bool zAlreadyTriggered = false;

float ball_x   = 0.0;
float ball_y   = 0.0;
float ball_w   = 0.0;
String ball_dir = "SEARCH";
bool ball_detected    = false;
unsigned long lastMsgTime = 0;
const unsigned long BALL_TIMEOUT = 1500;

bool kickInProgress = false;

void updateServoPos(int target1, int target2, int target3, char leg) {
  if (leg == 'l') {
    hipL.write(hipLOffset - target1);
    kneeL.write(kneeLOffset - target2);
    ankleL.write(2 * ankleLOffset - target3);
  } else if (leg == 'r') {
    hipR.write(hipROffset + target1);
    kneeR.write(kneeROffset + target2);
    ankleR.write(ankleROffset + target3);
  }
}

void pos(float x, float z, char leg) {
  float hipRad2 = atan(x / z);
  float z2      = z / cos(hipRad2);
  float hipRad1 = acos((sq(l1) + sq(z2) - sq(l2)) / (2 * l1 * z2));
  float kneeRad = PI - acos((sq(l1) + sq(l2) - sq(z2)) / (2 * l1 * l2));
  float ankleRad= PI / 2 + hipRad2 - acos((sq(l2) + sq(z2) - sq(l1)) / (2 * l2 * z2));
  updateServoPos(hipRad1 * (180/PI) + hipRad2 * (180/PI),
                 kneeRad  * (180/PI),
                 ankleRad * (180/PI), leg);
}

void takeStep(float stepLength, int stepVelocity) {
  for (float i = stepLength; i >= -stepLength; i -= 0.5) {
    pos(i,  stepHeight,               'r');
    pos(-i, stepHeight - stepClearance,'l');
    delay(stepVelocity);
  }
  for (float i = stepLength; i >= -stepLength; i -= 0.5) {
    pos(-i, stepHeight - stepClearance,'r');
    pos(i,  stepHeight,               'l');
    delay(stepVelocity);
  }
}

void initialize() {
  for (float i = 10.7; i >= stepHeight; i -= 0.1) {
    pos(0, i, 'l');
    pos(0, i, 'r');
  }
}

void turn(float degrees, char direction) {
  float degreesPerStep  = 5.0;
  int   stepCount       = abs(degrees) / degreesPerStep;
  float turnStepLength  = 1.5;
  int   stepVelocity    = 100;

  for (int s = 0; s < stepCount; s++) {
    for (float i = turnStepLength; i >= -turnStepLength; i -= 0.5) {
      if (direction == 'r') {
        pos(i,  stepHeight - stepClearance, 'l');
        pos(-i * 0.2, stepHeight,           'r');
      } else {
        pos(i,  stepHeight - stepClearance, 'r');
        pos(-i * 0.2, stepHeight,           'l');
      }
      delay(stepVelocity);
    }
    for (float i = -turnStepLength; i <= turnStepLength; i += 0.5) {
      pos(0, stepHeight, 'l');
      pos(0, stepHeight, 'r');
      delay(stepVelocity / 2);
    }
  }
}

void left(float degrees)  { turn(degrees, 'l'); }
void right(float degrees) { turn(degrees, 'r'); }

void forwardFallCounter() {
  hipR.write(0); kneeR.write(80);
  ankleR.write(60); ankleL.write(60);
  Serial.println("forward fall counter");
}

void backwardFallCounter() {
  pos(-10, 20, 'r');
  Serial.println("backward fall counter");
}

void kickMotion() {
  Serial.println(">> KICK!");

  for (float i = 0; i >= -3.0; i -= 0.3) {
    pos(i, stepHeight, 'r');
    delay(30);
  }
  delay(200);

  for (float i = -3.0; i <= 4.0; i += 0.5) {
    pos(i, stepHeight, 'r');
    delay(15);
  }
  delay(150);

  initialize();
}

void readSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("JSON error: ");
    Serial.println(err.c_str());
    return;
  }

  ball_x   = doc["x"] | 0.0f;
  ball_y   = doc["y"] | 0.0f;
  ball_w   = doc["w"] | 0.0f;
  ball_dir = doc["d"] | "SEARCH";

  ball_detected = true;
  lastMsgTime   = millis();

  Serial.printf("Ball -> x:%.2f y:%.2f w:%.0f dir:%s\n",
                ball_x, ball_y, ball_w, ball_dir.c_str());
}

void executeBallCommand() {
  if (!ball_detected || kickInProgress) return;

  if (ball_dir == "KICK") {
    kickInProgress = true;
    kickMotion();
    kickInProgress = false;
    ball_detected  = false;

  } else if (ball_dir == "TURN LEFT") {
    left(10);

  } else if (ball_dir == "TURN RIGHT") {
    right(10);

  } else if (ball_dir == "MOVE FORWARD") {
    takeStep(1, 100);

  } else if (ball_dir == "SEARCH") {
    right(15);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  if (!mpu.begin()) { while (1) yield(); }

  hipL.attach(25);  hipR.attach(13);
  kneeL.attach(26); kneeR.attach(12);
  ankleL.attach(27); ankleR.attach(14);

  hipL.write(hipLOffset);   kneeL.write(kneeLOffset);   ankleL.write(ankleLOffset);
  hipR.write(hipROffset);   kneeR.write(kneeROffset);   ankleR.write(ankleROffset);

  delay(5000);
  initialize();
  Serial.println("Ready");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float z = a.acceleration.z;

  bool isOutsideThreshold = (z > Z_THRESHOLD_UP || z < Z_THRESHOLD_DOWN);
  if (isOutsideThreshold) {
    if (!zAlreadyTriggered) {
      if (z > Z_THRESHOLD_UP) forwardFallCounter();
      else                     backwardFallCounter();
      zAlreadyTriggered = true;
    }
    return;
  } else {
    if (zAlreadyTriggered) {
      Serial.println("--- Stabilized. Re-armed. ---");
      zAlreadyTriggered = false;
    }
  }

  readSerial();

  if (ball_detected && millis() - lastMsgTime > BALL_TIMEOUT) {
    ball_detected = false;
    ball_dir      = "SEARCH";
  }

  executeBallCommand();
}