#include <Wire.h>
#include <I2Cdev.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "asservissement.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
Servo motA;
Servo motB;
Servo motC;
Servo motD;

#define LED_PIN 13

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int x = 0;
int* cmd;
float* errors;
float mesures[3] = {0,0,0};
int* cmdMot;



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    //Serial.begin(9600);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    motA.attach(3);
    motB.attach(4);
    motC.attach(5);
    motD.attach(6);

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) {
      return;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }

        //--- 1. Lecture des commandes moteur ---
        cmd = getCommandes();
        /*Serial.print("Commandes : ");
        Serial.print(cmd[0]);
        Serial.print(";");
        Serial.print(cmd[1]);
        Serial.print(";");
        Serial.print(cmd[2]);
        Serial.print(";");
        Serial.print(cmd[3]);
        Serial.print("\n");*/

        //--- 2. Lecture des mesures du capteur ---
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        mesures[0] = ypr[0] * 180/M_PI; // Yaw   : lacet
        mesures[1] = ypr[1] * 180/M_PI; // Pitch : tangage
        mesures[2] = ypr[2] * 180/M_PI; // Roll  : rouli

        
        /*Serial.print(mesures[0]);
        Serial.print(";");
        Serial.print(mesures[1]);
        Serial.print(";");
        Serial.print(mesures[2]);
        Serial.print(";");
        Serial.print(x);
        Serial.print("\n");*/

        //--- 3. Calcul des erreurs ---
        errors = calcErrors(mesures, cmd);
        /*
        Serial.print(errors[0]);
        Serial.print(";");
        Serial.print(errors[1]);
        Serial.print(";");
        Serial.print(errors[2]);
        Serial.print(";");
        Serial.print(x);
        Serial.print("\n");*/

        //--- 4. Calcul de l'asservissement des moteurs ---
        cmdMot = asservissementP(errors, cmd[3]);

        //--- 5. Affectation des commandes ---
        motA.write(cmdMot[0]);
        motB.write(cmdMot[1]);
        motC.write(cmdMot[2]);
        motD.write(cmdMot[3]);

        
        Serial.print("Mot A : ");
        Serial.print(cmdMot[0]);
        Serial.print("\tMot B : ");        
        Serial.print(cmdMot[1]);
        Serial.print("\nMot C : ");
        Serial.print(cmdMot[2]);
        Serial.print("\tMot D : ");
        Serial.print(cmdMot[3]);
        Serial.print("\n\n");
        
        
              
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

    x++;
}
