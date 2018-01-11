# Quadcopter PID implementation
## 1. Introduction

The goal of this project is to realize automation routine of an X quadricopter based on an Arduino Uno and the MPU-6050 sensor.

Basicaly, this automation routine is an implementation of a digital PID.
The method used to calculate PID coefficients is Ziegler Nichols method.

This project uses I2Cdev library that can be found here : https://github.com/jrowberg/i2cdevlib

Currently in beta version.

## 2. Requirements
Arduino libraries:
* Wire
* I2C
* Servo
* SimpleTimer
* MPU6050_6Axis_MotionApps20

## 3. Pin connection:
```
       +-------------------------+
       |        MPU-6050         |
       |                         |
       | 3V3  SDA  SCL  GND  INT |
       +--+----+----+----+----+--+
          |    |    |    |    |
          |    |    |    |    |
+---------+----+----+----+----+-----------+
|        3.3V  A4   A5  GND   #2          |
|                                         |
|                                         |
|                 Arduino Uno             |
|                                         |
| #4   #5   #6   #7   #8   #9  #10   #11  |
+--+----+----+----+----+----+----+----+---+
   |    |    |    |    |    |    |    |
  (M1) (M2) (M3) (M4) (C1) (C2) (C3) (C4)
  
Legend:
Mx : Motor X
Cx : Receiver channel x
```

Quadcopter orientation:
```
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
