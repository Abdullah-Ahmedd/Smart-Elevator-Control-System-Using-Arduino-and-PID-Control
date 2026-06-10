# Smart Elevator Control System Using Arduino and PID Control

A closed-loop elevator control system built on Arduino UNO, featuring PID-based motor control, real-time position feedback, and safety monitoring.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [System Architecture](#system-architecture)
- [Working Principle](#working-principle)
- [PID Control](#pid-control)
- [Applications](#applications)
- [Team Members](#team-members)

---

## Overview

This project implements a smart elevator prototype that uses a PID (Proportional-Integral-Derivative) controller to achieve accurate floor positioning, smooth acceleration/deceleration, and safe operation. The system reads user input from a potentiometer, tracks position via a rotary encoder, and uses an ultrasonic sensor as a safety mechanism.

---

## Features

- Accurate floor positioning using closed-loop PID control
- Smooth acceleration and deceleration profiles
- Real-time position tracking via rotary encoder
- Collision prevention using ultrasonic sensor
- Low-cost, simple hardware architecture
- Reduced overshoot and oscillations

---

## Hardware Components

| Component | Role |
|---|---|
| Arduino UNO | Main controller — runs PID algorithm and manages all I/O |
| Potentiometer | Floor selection unit — user input interface |
| Rotary Encoder | Position feedback sensor — tracks motor shaft rotation |
| Ultrasonic Sensor | Safety system — detects shaft limit proximity |
| L298N Motor Driver | Power interface between Arduino and DC motor |
| DC Gear Motor | Actuator — raises and lowers the elevator cabin |

---

## System Architecture

```
User Input (Potentiometer)
        │
        ▼
  Arduino UNO  ◄──── Rotary Encoder (Position Feedback)
        │        ◄──── Ultrasonic Sensor (Safety)
        ▼
  L298N Motor Driver
        │
        ▼
   DC Gear Motor
        │
        ▼
  Elevator Cabin
```

---

## Working Principle

1. **Floor Selection** — User rotates the potentiometer; Arduino reads the analog input and maps it to a target floor position.
2. **Motion Profiling** — Motor speed is gradually increased to create a smooth acceleration profile.
3. **Position Monitoring** — Rotary encoder pulses are counted continuously to determine the cabin's current position.
4. **Safety Monitoring** — Ultrasonic sensor checks proximity to shaft limits; triggers an emergency stop if a boundary is reached.
5. **PID Control** — The controller computes the error and adjusts motor speed and direction accordingly.

---

## PID Control

The PID controller minimizes the position error using three terms:

```
Error = Target Position − Current Position
```

| Term | Function |
|---|---|
| **P** (Proportional) | Corrective action proportional to current error |
| **I** (Integral) | Eliminates steady-state error for accurate floor alignment |
| **D** (Derivative) | Predicts future error trends to reduce overshoot |

---

## Applications

- Residential elevators
- Industrial lifting systems
- Automated storage systems
- Educational control-system labs
- Smart building automation

---

## Team Members

| Name | Section | ID |
|---|---|---|
| Amira Salah Eldin Mohamed | 1 | 9230247 |
| Aya Medhat Essam Eldein | 1 | 9230269 |
| Hussien Mohamed Hesham | 2 | 9230346 |
| Zaynab Mohamed Sayed Darwish | 2 | 9230414 |
| Sheirifa Alai Koreny Alai | 2 | 9230475 |
| Abdullah Ahmed Mohamed Sayed Aly | 2 | 9230555 |

> Submitted to: **Dr. Mahmoud Gilany**
