#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;

#define MOTOR_LEFT_PWM 9
#define MOTOR_RIGHT_PWM 5
#define MOTOR_LEFT_DIR 7
#define MOTOR_RIGHT_DIR 8


double setpoint = 0, input, output;
double Kp = 2.2, Ki = 0.09, Kd = 0.05;  
PID yawPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
    Serial.println("MPU6050 connected");
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    
    yawPID.SetMode(AUTOMATIC);
    yawPID.SetOutputLimits(-50, 50); 
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    input = gz / 131.0;
    yawPID.Compute();
    int baseSpeed = 83;  
    int leftSpeed = baseSpeed + output;
    int rightSpeed = baseSpeed - output;
    leftSpeed = constrain(leftSpeed, 90, 120);
    rightSpeed = constrain(rightSpeed, 90, 120);
    analogWrite(MOTOR_LEFT_PWM, leftSpeed);
    analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
  
    Serial.print("Yaw Rate: "); Serial.print(input);
    Serial.print(" | Correction: "); Serial.println(output);
    Serial.print(" left speed "); Serial.println(leftSpeed);
    Serial.print(" right speed "); Serial.println(rightSpeed);
    
    delay(10);
}

