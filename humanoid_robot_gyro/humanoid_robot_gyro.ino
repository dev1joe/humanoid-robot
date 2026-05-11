#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

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

#define bodyHalfWidth 3.0  // half the distance between feet in cm, tune this to your robot

Servo hipL;
Servo hipR;
Servo kneeL;
Servo kneeR;
Servo ankleL;
Servo ankleR;

Adafruit_MPU6050 mpu;

const float Z_THRESHOLD_UP = 2.5;
const float Z_THRESHOLD_DOWN = -2.5;

// STATE VARIABLE: Tracks if we are currently in an "exceeded" state
bool zAlreadyTriggered = false;

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
  float hipDeg2 = hipRad2 * (180 / PI);

  float z2 = z / cos(hipRad2);

  float hipRad1 = acos((sq(l1) + sq(z2) - sq(l2)) / (2 * l1 * z2));
  float hipDeg1 = hipRad1 * (180 / PI);

  float kneeRad = PI - acos((sq(l1) + sq(l2) - sq(z2)) / (2 * l1 * l2));

  float ankleRad = PI / 2 + hipRad2 - acos((sq(l2) + sq(z2) - sq(l1)) / (2 * l2 * z2));

  float hipDeg = hipDeg1 + hipDeg2;
  float kneeDeg = kneeRad * (180 / PI);
  float ankleDeg = ankleRad * (180 / PI);

  //  Serial.print(hipDeg);
  //  Serial.print("\t");
  //  Serial.print(kneeDeg);
  //  Serial.print("\t");
  //  Serial.println(ankleDeg);

  updateServoPos(hipDeg, kneeDeg, ankleDeg, leg);
}

/**
* @param stepLength how long the step is in cm
* @param stepVelocity delay in ms
*/
void takeStep(float stepLength, int stepVelocity) {
  for (float i = stepLength; i >= -stepLength; i -= 0.5) {
    pos(i, stepHeight, 'r');
    pos(-i, stepHeight - stepClearance, 'l');
    delay(stepVelocity);
  }

  for (float i = stepLength; i >= -stepLength; i -= 0.5) {
    pos(-i, stepHeight - stepClearance, 'r');
    pos(i, stepHeight, 'l');
    delay(stepVelocity);
  }
}

void initialize() {
  for (float i = 10.7; i >= stepHeight; i -= 0.1) {
    pos(0, i, 'l');
    pos(0, i, 'r');
  }
}

/**
 * Turns the robot by a specific angle.
 * @param degrees Total angle to turn
 * @param direction 'l' for left, 'r' for right
 */
void turn(float degrees, char direction) {
  // Calibration factor: how many degrees does one "1cm step" turn the robot?
  // Adjust this value based on your robot's physical width.
  float degreesPerStep = 5.0; 
  int stepCount = abs(degrees) / degreesPerStep;
  float turnStepLength = 1.5; // The stride length during the turn
  int stepVelocity = 100;

  for (int s = 0; s < stepCount; s++) {
    // Phase 1: Lift and move one leg forward while the other pushes
    for (float i = turnStepLength; i >= -turnStepLength; i -= 0.5) {
      if (direction == 'r') {
        // Right turn: Left leg swings wide (larger i), Right leg pivots (small or negative i)
        pos(i, stepHeight - stepClearance, 'l'); 
        pos(-i * 0.2, stepHeight, 'r'); // Right leg barely moves
      } else {
        // Left turn: Right leg swings wide, Left leg pivots
        pos(i, stepHeight - stepClearance, 'r');
        pos(-i * 0.2, stepHeight, 'l');
      }
      delay(stepVelocity);
    }

    // Phase 2: Reset legs to neutral to prepare for next step
    for (float i = -turnStepLength; i <= turnStepLength; i += 0.5) {
      pos(0, stepHeight, 'l');
      pos(0, stepHeight, 'r');
      delay(stepVelocity / 2);
    }
  }
} 

// Wrapper functions for ease of use
void left(float degrees) {
  turn(degrees, 'l');
}

void right(float degrees) {
  turn(degrees, 'r');
}

void myEmergencyFunction(float currentZ) {
  Serial.println("\n*** FIRST DETECTION: Emergency Function Executed ***");
  Serial.printf("Trigger Value: %.2f\n", currentZ);
}

void forwardFallCounter() {
  hipR.write(0);
  kneeR.write(80);
  ankleR.write(60);
  ankleL.write(60);
  Serial.println("forward fall counter");
}

void backwardFallCounter() {
  pos(-10, 20, 'r');
  Serial.println("backward fall counter");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  if (!mpu.begin()) { while (1) yield(); }

    hipL.attach(25);
  hipR.attach(13);
  kneeL.attach(26);
  kneeR.attach(12);
  ankleL.attach(27);
  ankleR.attach(14);

  hipL.write(hipLOffset);
  kneeL.write(kneeLOffset);
  ankleL.write(ankleLOffset);

  hipR.write(hipROffset);
  kneeR.write(kneeROffset);
  ankleR.write(ankleROffset);

  delay(5000);

  initialize();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float z = a.acceleration.z;

  // Check if we are outside the "Safe Zone"
  bool isOutsideThreshold = (z > Z_THRESHOLD_UP || z < Z_THRESHOLD_DOWN);

  if (isOutsideThreshold) {
    // Only call the function if this is the FIRST time we noticed it's outside
    if (!zAlreadyTriggered) {
      if (z > Z_THRESHOLD_UP) {
        forwardFallCounter();
      } else {
        backwardFallCounter();
      }
      zAlreadyTriggered = true; // "Latch" the state so it doesn't fire again
    }
    
    // We still 'return' to skip the rest of the loop while the Z-axis is crazy
    return; 
  } 
  else {
    // Z is back in the safe zone! 
    // Reset the flag so we can detect the next movement later
    if (zAlreadyTriggered) {
      Serial.println("--- Z-Axis stabilized. System re-armed. ---");
      zAlreadyTriggered = false;
    }
  }

  // REGULAR TASKS (Only runs if Z is within -5 t'o 5)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.printf("Safe Zone -> Z: %.2f\n", z);
    lastPrint = millis();
  }

  takeStep(1, 100); // Walk forward
}