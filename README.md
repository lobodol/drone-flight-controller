The goal of this project is to realize automation routine of an X quadricopter based on an Arduino Uno and the MPU-6050 sensor.

This project uses I2Cdev library that can be found here : https://github.com/jrowberg/i2cdevlib

And VirtualWire library for wireless communication.

Currently in beta version.

```
Pin connection :
+--------------------+
|      MPU-6050      |
|                    |
| 3V3  SDA  SCL  GND |
+--+----+----+----+--+
   |    |    |    |
   |    |    |    |
+--+----+----+----+--+
| 3.3V  A4   A5  GND |
|                    |
|    Arduino Uno     |
+--------------------+
```
