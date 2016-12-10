// ---------------------------------------------------------------------------
#include <Wire.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <SimpleTimer.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "asservissement.h"
#include "instructions.h"
#include "debug.h"
#include "SoftwareSerial.h"
#include "utils.h"
// ------------------ Declaration of each motor's servo ----------------------
Servo motA;
Servo motB;
Servo motC;
Servo motD;
// ------------- Global variables used for PID automation -----------------------------
float* cmd;                     // Received instructions : [Yaw, Pitch, Roll, Throttle]
float* errors;                  // Measured errors (used for proportional component) : [Yaw, Pitch, Roll]
float  sErr[3]     = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float  lastErr[3]  = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
float  measures[3] = {0, 0, 0}; // Angle measures : [Yaw, Pitch, Roll]
SimpleTimer timer;
// --------------------- MPU650 variables ------------------------------------
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU control/status vars
bool     dmpReady = false; // set true if DMP init was successful
uint8_t  mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t  devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;        // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64];   // FIFO storage buffer

// Orientation/motion vars
Quaternion  q;       // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // Indicates whether MPU interrupt pin has gone high
// ---------------------------------------------------------------------------



/**
 * Interrup détection routine
 */
void dmpDataReady() {
    mpuInterrupt = true;
}



/**
 * Setup configuration
 */
void setup() {
    init_imu();

    // Call automation routine every ms (sampling frequency = 1kHz)
    timer.setInterval(1, automation);
        
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)    

    Serial.begin(57600); // Causes a lot of FIFO Overflows under 38400 bauds
    mySerial.begin(9600);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // MPU calibration
    mpu.setXAccelOffset(377);
    mpu.setYAccelOffset(-4033);
    mpu.setZAccelOffset(1591);
    mpu.setXGyroOffset(115);
    mpu.setYGyroOffset(101);
    mpu.setZGyroOffset(32);
    
    // Returns 0 if it worked
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0 : #pin2)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Get expected DMP packet size for later comparison
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

    // Define each motor's pin
    motA.attach(10, 1000, 2000);
    motB.attach(5, 1000, 2000);
    motC.attach(6, 1000, 2000);
    motD.attach(7, 1000, 2000);

    initialize_motor();
}


/**
 * Main program loop
 */
void loop() {
    timer.run();
    
    // If programming failed, don't try to do anything
    if (!dmpReady) {
      return;
    }

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // Do nothing...
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // Wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        //--- 1. Read instructions from virtual wire ---
        cmd = getInstructions();

        //--- 2. Read measures from sensor ---
        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // Convert Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        measures[YAW]   = ypr[YAW]   * (180 / M_PI) * 0;
        measures[PITCH] = ypr[PITCH] * (180 / M_PI);
        measures[ROLL]  = ypr[ROLL]  * (180 / M_PI);

        dumpMeasures(measures);
        
        //--- 3. Calculate errors compared to instructions ---
        errors = calcErrors(measures, cmd);
        //dumpErrors(errors);
    }
}

/**
 * ESCs configuration
 */
void initialize_motor()
{
    Serial.print("Arming the motors! \n");

    motA.write(0);
    motB.write(0);
    motC.write(0);
    motD.write(0);
    delay(2000);
        
    Serial.print("MOTORS ARE READY! \n");
    delay(2000);
}

/**
 * Init the MPU
 */
void init_imu()
{
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(-9);
    mpu.setYGyroOffset(-3);
    mpu.setZGyroOffset(61);
    mpu.setXAccelOffset(-449);
    mpu.setYAccelOffset(2580);
    mpu.setZAccelOffset(1259);
  
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)…"));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 * 
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Actually, Pulse Position Modulation (PPM) is used to control motors speeds.
 * As a result, value range is about 0 to 180°.
 */
void automation()
{
    float  Kp[3]       = {0.0, 0.0, 0.06}; // P coefficients in that order : Yaw, Pitch, Roll
    float  Ki[3]       = {0.0, 0.0, 0.01}; // I coefficients in that order : Yaw, Pitch, Roll
    float  Kd[3]       = {0, 0, 25};       // D coefficients in that order : Yaw, Pitch, Roll
    float  deltaErr[3] = {0, 0, 0};        // Error deltas in that order :  Yaw, Pitch, Roll
    // Initialize motor commands with throttle
    float cmd_motA = cmd[THROTTLE];
    float cmd_motB = cmd[THROTTLE];
    float cmd_motC = cmd[THROTTLE];
    float cmd_motD = cmd[THROTTLE];

    // Do not calculate anything if throttle is 0
    if (cmd[THROTTLE] != 0) { 
        // Calculate sum of errors : Integral coefficients
        sErr[YAW]   += errors[YAW];
        sErr[PITCH] += errors[PITCH];
        sErr[ROLL]  += errors[ROLL];
        
        // Calculate error delta : Derivative coefficients
        deltaErr[YAW]   = errors[YAW]   - lastErr[YAW];
        deltaErr[PITCH] = errors[PITCH] - lastErr[PITCH];
        deltaErr[ROLL]  = errors[ROLL]  - lastErr[ROLL];
        
        // Save current error as lastErr for next time
        lastErr[YAW]   = errors[YAW];
        lastErr[PITCH] = errors[PITCH];
        lastErr[ROLL]  = errors[ROLL];
        
        // Yaw - Lacet (Z)
        cmd_motA -= (errors[YAW] * Kp[YAW] + sErr[YAW] * Ki[YAW] + deltaErr[YAW] * Kd[YAW]);
        cmd_motD -= (errors[YAW] * Kp[YAW] + sErr[YAW] * Ki[YAW] + deltaErr[YAW] * Kd[YAW]);
        cmd_motC += (errors[YAW] * Kp[YAW] + sErr[YAW] * Ki[YAW] + deltaErr[YAW] * Kd[YAW]);
        cmd_motB += (errors[YAW] * Kp[YAW] + sErr[YAW] * Ki[YAW] + deltaErr[YAW] * Kd[YAW]);
        
        // Pitch - Tangage (Y)
        cmd_motA -= (errors[PITCH] * Kp[PITCH] + sErr[PITCH] * Ki[PITCH] + deltaErr[PITCH] * Kd[PITCH]);
        cmd_motB -= (errors[PITCH] * Kp[PITCH] + sErr[PITCH] * Ki[PITCH] + deltaErr[PITCH] * Kd[PITCH]);
        cmd_motC += (errors[PITCH] * Kp[PITCH] + sErr[PITCH] * Ki[PITCH] + deltaErr[PITCH] * Kd[PITCH]);
        cmd_motD += (errors[PITCH] * Kp[PITCH] + sErr[PITCH] * Ki[PITCH] + deltaErr[PITCH] * Kd[PITCH]);
        
        // Roll - Roulis (X)
        cmd_motA -= (errors[ROLL] * Kp[ROLL] + sErr[ROLL] * Ki[ROLL] + deltaErr[ROLL] * Kd[ROLL]);
        cmd_motC -= (errors[ROLL] * Kp[ROLL] + sErr[ROLL] * Ki[ROLL] + deltaErr[ROLL] * Kd[ROLL]);
        cmd_motB += (errors[ROLL] * Kp[ROLL] + sErr[ROLL] * Ki[ROLL] + deltaErr[ROLL] * Kd[ROLL]);
        cmd_motD += (errors[ROLL] * Kp[ROLL] + sErr[ROLL] * Ki[ROLL] + deltaErr[ROLL] * Kd[ROLL]);
    }

    // Write speed for each motor
    motA.write(normalize(cmd_motA));
    motB.write(normalize(cmd_motB));
    motC.write(normalize(cmd_motC));
    motD.write(normalize(cmd_motD));
}


