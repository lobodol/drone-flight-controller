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
   |    |    |    |    |    (M1) (M2) (M3) (M4)  
   |    |    |    |    |     |    |    |    |
+--+----+----+----+----+-----+----+----+----+---+
| 3.3V  A4   A5  GND   #2    #5   #6   #7   #10 |
|                                               |
|                                               |
|                 Arduino Uno                   |
+-----------------------------------------------+
```