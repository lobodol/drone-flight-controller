/**
 * @author lobodol <grobodol@gmail.com>
 */

// ---------------------------------------------------------------------------
#include <Wire.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <SimpleTimer.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "receiver.h"
#include "utils.h"
#include "debug.h"

// ------------------ Declaration of each motor's servo ----------------------
Servo motA;
Servo motB;
Servo motC;
Servo motD;
// ------------- Global variables used for PID automation -----------------------------
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float error_sum[3] = {0, 0, 0};      // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
float measures[3] = {0, 0, 0};       // Angular measures : [Yaw, Pitch, Roll]
SimpleTimer pid_timer;               // Timer used to run PID automation at desired frequency.
// --------------------- MPU650 variables ------------------------------------
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU control/status vars
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // Indicates whether MPU interrupt pin has gone high
// ---------------------------------------------------------------------------


/**
 * Interrupt detection routine.
 */
void dmpDataReady() {
    mpuInterrupt = true;
}

/**
 * Setup configuration
 */
void setup() {
    // Turn LED on during setup.
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    configureChannelMapping();

    // Call automation routine every 10ms (sampling frequency = 100Hz)
    pid_timer.setInterval(10, automation);

    // Start I2C communication
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    Serial.begin(57600);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Sensor calibration
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
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));

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

        // Make LED blink fast and wait for restart.
        while (true) {
            digitalWrite(13, LOW);
            delay(250);
            digitalWrite(13, HIGH);
            delay(250);
        }
    }

    // Configure interrupts for receiver.
    PCICR  |= (1 << PCIE0);  //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0); //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1); //Set PCINT1 (digital input 9) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2); //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3); //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

    // Define each motor's pin
    motA.attach(3, 1000, 2000);
    motB.attach(5, 1000, 2000);
    motC.attach(6, 1000, 2000);
    motD.attach(7, 1000, 2000);

    initialize_esc();

    // Turn LED off: setup is done.
    digitalWrite(13, LOW);
}


/**
 * Main program loop
 */
void loop() {
    pid_timer.run();

    if (mpuInterrupt || fifoCount >= packetSize) {
        // Reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // Get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // Check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // Reset so we can continue cleanly
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));

            // Otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            readSensor();
//          dumpMeasures(measures);

            getFlightInstruction();
            dumpCommands(instruction);

            // Calculate errors compared to instructions
            calculateErrors();
            //dumpErrors(errors);
        }
    }
}

/**
 * ESCs configuration.
 * Send low throttle to each ESC to set lowest point.
 *
 * Takes about 4sec.
 */
void initialize_esc() {
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
 * Each motor output is considered as a servomotor. As a result, value range is about 0 to 180° (full speed).
 */
void automation() {
    float Kp[3] = {0.0, 0.0, 0.32};    // P coefficients in that order : Yaw, Pitch, Roll //ku = 0.21
    float Ki[3] = {0.0, 0.0, 0.008};  // I coefficients in that order : Yaw, Pitch, Roll
    float Kd[3] = {0, 0, 10};           // D coefficients in that order : Yaw, Pitch, Roll
    float deltaErr[3] = {0, 0, 0};     // Error deltas in that order :  Yaw, Pitch, Roll
    // Initialize motor commands with throttle
    float cmd_motA = instruction[THROTTLE];
    float cmd_motB = instruction[THROTTLE];
    float cmd_motC = instruction[THROTTLE];
    float cmd_motD = instruction[THROTTLE];
    float yaw   = 0;
    float pitch = 0;
    float roll  = 0;

    // Do not calculate anything if throttle is 0
    if (instruction[THROTTLE] >= 5) {
        // Calculate sum of errors : Integral coefficients
        error_sum[YAW]   += errors[YAW];
        error_sum[PITCH] += errors[PITCH];
        error_sum[ROLL]  += errors[ROLL];

        // Calculate error delta : Derivative coefficients
        deltaErr[YAW]   = errors[YAW]   - previous_error[YAW];
        deltaErr[PITCH] = errors[PITCH] - previous_error[PITCH];
        deltaErr[ROLL]  = errors[ROLL]  - previous_error[ROLL];

        // Save current error as previous_error for next time
        previous_error[YAW]   = errors[YAW];
        previous_error[PITCH] = errors[PITCH];
        previous_error[ROLL]  = errors[ROLL];

        yaw   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (deltaErr[YAW]   * Kd[YAW]);
        pitch = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (deltaErr[PITCH] * Kd[PITCH]);
        roll  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (deltaErr[ROLL]  * Kd[ROLL]);

        // Yaw - Lacet (Z axis)
        cmd_motA -= yaw;
        cmd_motD -= yaw;
        cmd_motC += yaw;
        cmd_motB += yaw;

        // Pitch - Tangage (Y axis)
        cmd_motA -= pitch;
        cmd_motB -= pitch;
        cmd_motC += pitch;
        cmd_motD += pitch;

        // Roll - Roulis (X axis)
        cmd_motA -= roll;
        cmd_motC -= roll;
        cmd_motB += roll;
        cmd_motD += roll;
    }

    // Apply speed for each motor
    motA.write(normalize(cmd_motA));
    motB.write(normalize(cmd_motB));
    motC.write(normalize(cmd_motC));
    motD.write(normalize(cmd_motD));
}

/**
 * Get angular values from MPU sensor.
 *
 * @return void
 */
void readSensor()
{
    // Wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
    }

    // Read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Convert Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    measures[YAW]   = ypr[YAW]   * (180 / M_PI) * 0; // Not ready yet : force to 0 for now.
    measures[PITCH] = ypr[PITCH] * (180 / M_PI) * 0; // Force to 0 during PID tuning
    measures[ROLL]  = ypr[ROLL]  * (180 / M_PI);
}

/**
 * Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.
 *
 * @return void
 */
void calculateErrors()
{
    errors[YAW]   = measures[YAW]   - instruction[YAW];
    errors[PITCH] = measures[PITCH] - instruction[PITCH];
    errors[ROLL]  = measures[ROLL]  - instruction[ROLL];
}
