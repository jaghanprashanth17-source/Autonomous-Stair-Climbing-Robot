# Autonomous-Stair-Climbing-Robot

📌 Project Overview
This project presents the design, implementation, and testing of a mobile robot capable of autonomous stair detection and navigation. By leveraging real-time feedback from an MPU6050 Inertial Measurement Unit (IMU), the system identifies stair encounters through Z-axis acceleration spikes and maintains stability using dynamic pitch and roll compensation.

Key Engineering Achievements:
Autonomous Step Detection: Developed a real-time algorithm that identifies stair edges by analyzing vertical acceleration impulses, achieving a torque boost phase for climbing.

Control Optimization: Reduced Yaw Rate error by 60% using a tuned PID controller, ensuring precise straight-line trajectory even on uneven surfaces.

Hybrid Motion Control: Integrated user-defined rotation logic with path-planning algorithms to allow for autonomous "return-to-home" functionality.

🛠 Tech Stack
Microcontroller: Arduino Mega (ATMega2560).

Sensors: MPU6050 (3-Axis Accelerometer & Gyroscope).

Actuators: BY1016Z High-Torque DC Gear Motors (250W, 24V).

Control Theory: PID (Proportional-Integral-Derivative) for speed and deviation correction.

Programming: Embedded C++ (Arduino IDE).
