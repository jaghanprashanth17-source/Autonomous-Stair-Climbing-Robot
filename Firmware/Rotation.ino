#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 2;  
bool blinkState;

bool DMPReady = false;  
uint8_t MPUIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t FIFOBuffer[64]; 

Quaternion q;
VectorFloat gravity;
float ypr[3];  
const int MotorRight = 5;
const int MotorLeft = 9;
const int MotorRightBack = 12;
const int MotorLeftBack = 8;
const int MotorRightBack_1 = 11;
const int MotorLeftBack_1 = 7;
float targetYaw = 0;
float yaw = 0;
volatile bool MPUInterrupt = false;
void DMPDataReady() {
    MPUInterrupt = true;
}

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

  pinMode(MotorLeft, OUTPUT);
  pinMode(MotorRight, OUTPUT);
  pinMode(MotorRightBack, OUTPUT);
  pinMode(MotorLeftBack, OUTPUT);
  pinMode(MotorRightBack_1, OUTPUT);
  pinMode(MotorLeftBack_1, OUTPUT);
}

void loop() {
  if (!DMPReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = ypr[0] * 180/M_PI; 
  }
  updateTargetYaw();
  controlRotation();

  delay(100);
}

void controlRotation() {
  float yawDifference = targetYaw - yaw;
  if (yawDifference > 180) {
    yawDifference -= 360;
  } else if (yawDifference < -180) {
    yawDifference += 360;
  }
  Serial.print("Yaw : ");
  Serial.println(yaw);
  Serial.print("Yaw difference: ");
    Serial.println(yawDifference);
  Serial.print("Target Yaw : ");
    Serial.println(targetYaw);
  if (abs(yawDifference) < 5) {
    analogWrite(MotorLeft, 0);
    analogWrite(MotorRight, 0);
    Serial.println("Stopping Robot (Target Reached)");
  } 
  else if (yawDifference > 0) {
    digitalWrite(MotorLeftBack, HIGH);
    digitalWrite(MotorLeftBack_1, HIGH);
    digitalWrite(MotorRightBack, LOW);
    digitalWrite(MotorRightBack_1, LOW);
    analogWrite(MotorLeft, 85);
    analogWrite(MotorRight, 85);
    Serial.println("Rotating Right");
    }
  else {
    digitalWrite(MotorLeftBack, LOW);
    digitalWrite(MotorLeftBack_1, LOW);
    digitalWrite(MotorRightBack, HIGH);
    digitalWrite(MotorRightBack_1, HIGH); 
    analogWrite(MotorLeft, 85);
    analogWrite(MotorRight, 85);
    Serial.println("Rotating Left");
  }
}

void updateTargetYaw() {
  if (Serial.available()) {
    char input = Serial.read(); 
    if (input == '0') {
      targetYaw = 0;  
      Serial.println("Target Yaw set to 0");
    } else if (input == '9') {
      targetYaw = yaw + 90;  
      Serial.println("Target Yaw set to 90");
    } else if (input == '-') {
      input = Serial.read();  
      if (input == '9') {
        targetYaw = yaw - 90;  
        Serial.println("Target Yaw set to -90");
      } else if (input == '4') {
        targetYaw = yaw - 45;  
        Serial.println("Target Yaw set to -45");
      }
    } else if (input == '1') {
      targetYaw = yaw + 180;  
      Serial.println("Target Yaw set to 180");
    }
    
  }
}
