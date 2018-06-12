# Quadcopter PID implementation
## 1. Introduction

This Arduino sketch provides a flight controller for an X quadcopter based on an Arduino Uno board and the [MPU6050 sensor](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).

Basically, this automation routine is an implementation of a digital PID.
The method used to calculate PID coefficients is Ziegler Nichols method.
The frame of the quadcopter is based on the F450.

You can use [this](https://github.com/lobodol/ESC-calibration) to calibrate your ESCs.

Currently under development.

## 2. Requirements
Arduino libraries:
* Wire

## 3. Pin connection:
```
       +-------------------------+
       |        MPU-6050         |
       |                         |
       | 3V3  SDA  SCL  GND  INT |
       +--+----+----+----+----+--+
          |    |    |    |
          |    |    |    |
+---------+----+----+----+----------------+
|        3.3V  A4   A5  GND               |
|                                         |
|                                         |
|                 Arduino Uno             |
|                                         |
| #4   #5   #6   #7   #8   #9  #10   #11  |
+--+----+----+----+----+----+----+----+---+
   |    |    |    |    |    |    |    |
  (M1) (M2) (M3) (M4)  |    |    |    |
                       |    |    |    |  
                       |    |    |    |
                    +--+----+----+----+---+
                    | C1   C2   C3   C4   |
                    |                     |
                    |     RF Receiver     |
                    +---------------------+
  
Legend:
Mx : Motor X
Cx : Receiver channel x
```

## 4. Quadcopter orientation

```
 Front
(1) (2)     x
  \ /     z ↑
   X       \|
  / \       +----→ y
(3) (4)
```

* Motor 1 : front left  - clockwise
* Motor 2 : front right - counter-clockwise
* Motor 3 : rear left   - clockwise
* Motor 4 : rear left   - counter-clockwise

![Paper plane](https://www.firediy.fr/images/articles/drone-1/ypr.jpg)
* Left wing **up** implies a positive roll
* Nose **up** implies a positive pitch
* Nose **right** implies a positive yaw
