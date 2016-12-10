# Quadcopter PID implementation

The goal of this project is to realize automation routine of an X quadricopter based on an Arduino Uno and the MPU-6050 sensor.

Basicaly, this automation routine is an implementation of a digital PID.
The method used to calculate PID coefficients is Ziegler Nichols method.

This project uses I2Cdev library that can be found here : https://github.com/jrowberg/i2cdevlib

Currently in beta version.

# Pin connection:
```
+-------------------------+
|      MPU-6050           |
|                         |
| 3V3  SDA  SCL  GND  INT |
+--+----+----+----+----+--+
   |    |    |    |    |
   |    |    |    |    |
+--+----+----+----+----+--+
| 3.3V  A4   A5  GND   #2 |
|                         |
|       Arduino Uno       |
+-------------------------+
```
