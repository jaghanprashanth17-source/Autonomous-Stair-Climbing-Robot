# Autonomous-Stair-Climbing-Robot

# 📌 Project Overview
This project presents the design, implementation, and testing of a mobile robot capable of autonomous stair detection and navigation. By leveraging real-time feedback from an MPU6050 Inertial Measurement Unit (IMU), the system identifies stair encounters through Z-axis acceleration spikes and maintains stability using dynamic pitch and roll compensation.

Key Engineering Achievements:
Autonomous Step Detection: Developed a real-time algorithm that identifies stair edges by analyzing vertical acceleration impulses, achieving a torque boost phase for climbing.

Control Optimization: Reduced Yaw Rate error by 60% using a tuned PID controller, ensuring precise straight-line trajectory even on uneven surfaces.

Hybrid Motion Control: Integrated user-defined rotation logic with path-planning algorithms to allow for autonomous "return-to-home" functionality.

## 🛠 Tech Stack:
Microcontroller: Arduino Mega (ATMega2560).
Sensors: MPU6050 (3-Axis Accelerometer & Gyroscope).

Actuators: BY1016Z High-Torque DC Gear Motors (250W, 24V).

Control Theory: PID (Proportional-Integral-Derivative) for speed and deviation correction.

Programming: Embedded C++ (Arduino IDE).

## ⚙️Core Logic & Implementation
1. Stair Detection Mechanism
The robot monitors the Z-axis acceleration in real-time. When a wheel impacts a stair edge, the sudden spike (exceeding a threshold of 2.5) triggers a Torque Boost phase.

Torque Boost: PWM increased to 115-130 to overcome the step height.

Leveling: The system monitors the Pitch Angle; once it returns to a level threshold, the robot resumes base power to conserve energy.

2. PID Speed & Yaw Correction
To prevent drifting, a dual-PID loop was implemented:

Speed PID: Uses tilt angle feedback to maintain a constant 83 PWM base speed regardless of load.

Yaw PID: Measures angular velocity around the vertical axis to correct for mechanical misalignments, resulting in a low 0.24°/s RMSE.

## 📊 Performance Evaluation
Rotation Accuracy: Achieved a Mean Absolute Error (MAE) of 2.23° across multiple 90° and 180° test runs.

Navigation Stability: Achieved a 60% reduction in yaw rate deviation compared to non-PID control systems.

## 🚀 Future Enhancements
Encoder Integration: Transitioning to physical wheel encoders for higher speed estimation precision.

Adaptive PID: Implementing real-time auto-tuning for PID parameters to adapt to varying terrain materials.
