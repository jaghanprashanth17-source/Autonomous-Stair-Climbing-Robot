#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az, gx, gy, gz;

const int MotorLeft = 9;
const int MotorRight = 5;

const int baseSpeed = 85;
const int boostTargetSpeed = 120;

const int boostStep = 5;
const int boostInterval = 60; 

const float climbPitchMin = 31.0;  
const float climbPitchMax = 38.0;  

const float zAccelThreshold = 2.0;

bool torqueBoostActive = false;
bool sensorsReady = false;

unsigned long lastBoostTime = 0;
unsigned long lastStairTime = 0;
const unsigned long stairTimeout = 4000; 
unsigned long sensorStartTime = 0;

int currentBoostSpeed = baseSpeed;
float prevZAccel = 0;
float filteredPitch = 0;
float filteredRoll = 0;
const float alpha = 0.1; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println(" MPU6050 connected. Warming up sensors...");
  sensorStartTime = millis();

  pinMode(MotorLeft, OUTPUT);
  pinMode(MotorRight, OUTPUT);
  analogWrite(MotorLeft, baseSpeed);
  analogWrite(MotorRight, baseSpeed);
}

void loop() {
  // Read IMU data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float rawPitch = atan2(ay, az) * 180.0 / M_PI;
  float rawRoll  = atan2(-ax, az) * 180.0 / M_PI;

  filteredPitch = filteredPitch * (1 - alpha) + rawPitch * alpha;
  filteredRoll  = filteredRoll  * (1 - alpha) + rawRoll * alpha;

  float zAccel = (float)az / 16384.0 * 9.81;
  float zAccelChange = abs(zAccel - prevZAccel);

  if (!sensorsReady && millis() - sensorStartTime > 1500) {
    sensorsReady = true;
    Serial.println("Sensors ready. Starting step detection...");
    prevZAccel = zAccel;
    return;
  }

  if (!sensorsReady) return;

  // Log filtered values
  Serial.print("Pitch: ");
  Serial.print(filteredPitch);
  Serial.print(" | Roll: ");
  Serial.print(filteredRoll);
  Serial.print(" | Z-Accel Δ: ");
  Serial.println(zAccelChange);

  if (!torqueBoostActive && zAccelChange > zAccelThreshold) {
    Serial.println(" Step detected → Starting torque boost");
    torqueBoostActive = true;
    currentBoostSpeed = baseSpeed;
    lastBoostTime = millis();
    lastStairTime = millis();
  }
  if (torqueBoostActive) {
    if (millis() - lastBoostTime >= boostInterval) {
      lastBoostTime = millis();

      currentBoostSpeed += boostStep;
      if (currentBoostSpeed > boostTargetSpeed) {
        currentBoostSpeed = boostTargetSpeed;
      }

      Serial.print(" Boosting speed: ");
      Serial.println(currentBoostSpeed);

      // Apply motor drive with roll correction
      int correction = filteredRoll * 1.0;
      driveMotors(currentBoostSpeed, correction);

      if (filteredPitch >= climbPitchMin && filteredPitch <= climbPitchMax) {
        Serial.println("Front wheels climbed → STOP motors!");
        // stopMotors();
        analogWrite(MotorLeft, 87);
        analogWrite(MotorRight, 87);

        torqueBoostActive = false;
        lastStairTime = millis();
        delay(500);
        return;
      }
    }
  }
  else {
    driveMotors(baseSpeed, 0);
  }
  if (millis() - lastStairTime > stairTimeout) {
  if (filteredPitch < 16) {
    stopMotors();
    Serial.println("⛔ No stairs for 4 seconds → Robot stopped.");
    while (1); 
  } else {
    int correction = filteredRoll * 1.0;
    driveMotors(95, correction);  
    Serial.println("Still climbing...");
  }
}
  prevZAccel = zAccel;
  delay(100);
}

void driveMotors(int base, int correction) {
  int leftPWM = constrain(base - correction, 70, 150);
  int rightPWM = constrain(base + correction, 70, 150);

  analogWrite(MotorLeft, leftPWM);
  analogWrite(MotorRight, rightPWM);

  Serial.print(" Driving → L: ");
  Serial.print(leftPWM);
  Serial.print(" | R: ");
  Serial.println(rightPWM);
}

void stopMotors() {
  analogWrite(MotorLeft, 0);
  analogWrite(MotorRight, 0);
  Serial.println(" Motors stopped.");
}
