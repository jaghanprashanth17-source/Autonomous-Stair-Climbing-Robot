#include <Wire.h>
// #include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 2;  
bool blinkState;

Quaternion q;
VectorFloat gravity;
float ypr[3]; 

bool DMPReady = false;  
uint8_t MPUIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t FIFOBuffer[64]; 

float yaw = 0;
float targetYaw = 0;
volatile bool MPUInterrupt = false;
void DMPDataReady() {
    MPUInterrupt = true;
}

const int MotorRight = 5;
const int MotorLeft = 9;
const int MotorRightBack = 12;
const int MotorLeftBack = 8;
const int MotorRightBack_1 = 11;
const int MotorLeftBack_1 = 7;

unsigned long previousTime = 0; 
double Setpoint, Input, Output;
double Kp = 3, Ki = 0, Kd = 0; 
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

float desiredSpeed = 90.0;  
float actualSpeed = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

MPU6050 mpu;


unsigned long lastTime;
double timeStep = 100;

unsigned long startTime; 
const unsigned long duration1 = 11000;  
const unsigned long duration2 = 20000;
const unsigned long duration3 = 37000;
const unsigned long duration4 = 49000;
float yawDifference=0;
void updateTargetYaw();
void controlRotation();
void PID();
void Turn90();
void Turn180();
void Turnminus90();
void stopMotors();


void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
Serial.begin(9600);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  } else {
    Serial.println("MPU6050 connection successful");
  }

  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(MotorRight, OUTPUT);
  pinMode(MotorLeft, OUTPUT);
  pinMode(MotorRightBack, OUTPUT);
  pinMode(MotorLeftBack, OUTPUT);
  pinMode(MotorRightBack_1, OUTPUT);
  pinMode(MotorLeftBack_1, OUTPUT);
  Setpoint = 33.3; 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); 
   startTime = millis();

  calibrateIMU();
  
  previousTime = millis();
  delay(1000);  
}

void calibrateIMU() 
{
  int numSamples = 1000;
  float sumY = 0;
    for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az;
    mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    float gyroz = gyroZ / 16384.0;  
    sumY += gyroz;
    delay(2);
  }
  
  
  float gyrozbias = (sumY / numSamples) * 9.81;  
  Serial.print("gyroz bias: ");
  Serial.println(gyrozbias);
}

void loop() 
{
    while (millis() - startTime <= duration1) 
    {
         PID();
        Serial.println("PID 1 running ");
    }
    while (millis() - startTime <= duration1 + 6000) 
    {
         Turn90();
    }
    while (millis() - startTime <= duration2 + 4000) {
     
        PID();
        Serial.println("PID 2 running ");
    }
    while (millis() - startTime <= duration2 + 10000) {
          Turn180();
    }
    while (millis() - startTime <= duration3) {
     PID();
    Serial.println("PID 3 running ");
  }
   while (millis() - startTime  <= duration3 + 8000) {
    Turnminus90();
  }
  while (millis() - startTime  <= duration4) {
    PID();
    Serial.println("PID 4 running ");
  }
      stopMotors();
      Serial.println("Reached home position ");
}

void PID(){
  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  Input = abs(gyroZ / 131.0); 
  Serial.print("Current Motor Speed (dps): ");
  Serial.println(Input);
  myPID.Compute();
  Serial.print("PID Output (PWM): ");
  Serial.println(Output);
  int motorSpeed = constrain(abs(Output), 0, 85);  
  analogWrite(MotorRight, motorSpeed);
  analogWrite(MotorLeft, motorSpeed);
  Serial.print("Motor Speed PWM: ");
  Serial.println(motorSpeed);
  delay(timeStep);
  
}

void Turn90() 
{
   targetYaw = 90;
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) 
    {

      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw = ypr[0] * 180/M_PI;  
      
    }
    yawDifference = targetYaw - yaw;
    Serial.print("Yaw : ");
    Serial.println(yaw);
    Serial.print("Yaw difference: ");
    Serial.println(yawDifference);

    if (abs(yawDifference) < 3) 
    {
      stopMotors();
      analogWrite(MotorLeft, 0);
      analogWrite(MotorRight, 0);
      Serial.println("Stopping Robot 90 (Target Reached)");
    } 
    else if (yawDifference > 0) 
    {
      digitalWrite(MotorLeftBack, HIGH);
      digitalWrite(MotorLeftBack_1, HIGH);
      digitalWrite(MotorRightBack, LOW);
      digitalWrite(MotorRightBack_1, LOW);

    analogWrite(MotorLeft, 80);
    analogWrite(MotorRight, 85);
    Serial.println("Rotating Right");
    } 
    else 
    {
        digitalWrite(MotorLeftBack, LOW);
        digitalWrite(MotorLeftBack_1, LOW);
        digitalWrite(MotorRightBack, HIGH);
        digitalWrite(MotorRightBack_1, HIGH);
    
      analogWrite(MotorLeft, 80);
      analogWrite(MotorRight, 85);
      Serial.println("Rotating Left");

    }
}

void Turn180()
{
   targetYaw = 270;
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) 
    {

      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw = ypr[0] * 180/M_PI;  
      
    }
  yawDifference = targetYaw - yaw;
  if (yawDifference > 180) {
    yawDifference -= 360;
  } else if (yawDifference < -180) {
    yawDifference += 360;
  }
    Serial.print("Target Yaw ");
    Serial.println(targetYaw);
    Serial.print("Yaw : ");
    Serial.println(yaw);
    Serial.print("Yaw difference: ");
    Serial.println(yawDifference);

  if (abs(yawDifference) <4) 
  {
          stopMotors();
          analogWrite(MotorLeft, 0);
          analogWrite(MotorRight, 0);
          Serial.println("Stopping Robot 180 (Target Reached)");
        
  } else if (yawDifference > 0)
   {
          digitalWrite(MotorLeftBack, HIGH);
          digitalWrite(MotorLeftBack_1, HIGH);
          digitalWrite(MotorRightBack, LOW);
          digitalWrite(MotorRightBack_1, LOW);
          analogWrite(MotorLeft, 80);
          analogWrite(MotorRight, 85);
          Serial.println("Rotating Right 180");
  }    else  {
        digitalWrite(MotorLeftBack, LOW);
        digitalWrite(MotorLeftBack_1, LOW);
        digitalWrite(MotorRightBack, HIGH);
        digitalWrite(MotorRightBack_1, HIGH);
    
      analogWrite(MotorLeft, 80);
      analogWrite(MotorRight, 85);
      Serial.println("Rotating Left");

           }
}

void Turnminus90()
{
 targetYaw = 0;
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) 
    {

      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw = ypr[0] * 180/M_PI;  
      
    }
yawDifference = targetYaw + yaw;
if (yawDifference > 180) {
    yawDifference -= 360;
  } else if (yawDifference < -180) {
    yawDifference += 360;
  }
    Serial.print("Target Yaw ");
    Serial.println(targetYaw);
    Serial.print("Yaw : ");
    Serial.println(yaw);
    Serial.print("Yaw difference: ");
    Serial.println(yawDifference);
if (abs(yawDifference) > 177 && abs(yawDifference) < 180) {
      stopMotors();
      analogWrite(MotorLeft, 0);
      analogWrite(MotorRight, 0);
    Serial.println("Stopping Robot -90 (Target Reached)");
  } else if (yawDifference > 0) {
          digitalWrite(MotorLeftBack, HIGH);
          digitalWrite(MotorLeftBack_1, HIGH);
          digitalWrite(MotorRightBack, LOW);
          digitalWrite(MotorRightBack_1, LOW);
          analogWrite(MotorLeft, 80);
          analogWrite(MotorRight, 85);
          Serial.println("Rotating Right ");
    Serial.println("Rotating Right");
  } else {
    digitalWrite(MotorLeftBack, LOW);
        digitalWrite(MotorLeftBack_1, LOW);
        digitalWrite(MotorRightBack, HIGH);
        digitalWrite(MotorRightBack_1, HIGH);
    
        analogWrite(MotorLeft, 80);
        analogWrite(MotorRight, 85);
        Serial.println("Rotating Left");
  }
}
void stopMotors()
{
  analogWrite(MotorLeft, 0);
  analogWrite(MotorRight, 0);
  digitalWrite(MotorLeftBack, LOW);
  digitalWrite(MotorLeftBack_1, LOW);
  digitalWrite(MotorRightBack, LOW);
  digitalWrite(MotorRightBack_1, LOW);
  } 
  
