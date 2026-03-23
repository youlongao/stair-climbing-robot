#  Three-Stage Stair-Climbing Robot

A modular C++ embedded system for controlling a **three-stage stair-climbing robot**, designed for complex terrain traversal including stairs, obstacles, and uneven surfaces.

---

##  Overview

This project implements a **layered control architecture** for a stair-climbing robot, separating hardware abstraction from high-level motion logic. The system is designed with scalability, safety, and maintainability in mind.

The repository currently focuses on the **Low-Level Driver Layer**, which provides direct control over actuators and feedback sensors.

---

##  Features

- ⚙️ Modular C++ design (hardware abstraction layers)
- 🔄 Closed-loop control support (via encoder feedback)
- 🛡️ Safety mechanisms (limit switches, safe positions, braking)
- 🧩 Easy integration with higher-level control (FSM, planning)
- 🐧 Linux-based embedded system (libgpiod + PWM driver)

---

##  System Architecture

```text
+-----------------------------+
|  Behavior / State Machine   |
+-----------------------------+
|   Motion Control Layer      |
+-----------------------------+
|   Mechanism Control Layer   |
+-----------------------------+
|   Sensor Feedback Layer     |
+-----------------------------+
|   Low-Level Driver Layer    |
+-----------------------------+
|   Hardware (Motors, etc.)   |
+-----------------------------+

climbing/
├── include/
│   ├── motor_driver.h
│   ├── servo_driver.h
│   ├── linear_actuator.h
│   └── encoder.h
│
├── src/
│   ├── motor_driver.cpp
│   ├── servo_driver.cpp
│   ├── linear_actuator.cpp
│   └── encoder.cpp
│
├── docs/
│   └── README-A.md
│

ow-Level Driver Layer
Motor Driver

Files
motor_driver.h
motor_driver.cpp
Description
Controls the main drive motors using PWM and GPIO direction signals.

Servo Driver
Files
servo_driver.h
servo_driver.cpp
Description
Controls servo motors for joints, locking mechanisms, and posture adjustment.

Linear Actuator
Files
linear_actuator.h
linear_actuator.cpp
Description
Controls linear motion components such as lead screws and lifting mechanisms.

Encoder
Files
encoder.h
encoder.cpp
Description
Provides feedback for position, speed, and distance using tick counting.

Author YI LIU
