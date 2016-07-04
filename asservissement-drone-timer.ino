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
// ------------------ Déclaration des commandes moteurs ----------------------
Servo motA;
Servo motB;
Servo motC;
Servo motD;
// ------------- Variables pour l'asservissement -----------------------------
int*   cmd;                       // Commandes reçues : [Yaw, Pitch, Roll, Throttle]
int*   cmdMot;                    // Commandes à appliquer aux moteurs : [MotA, MotB, MotC, MotD]
float* errors;                    // Erreurs mesurées : [Yaw, Pitch, Roll]
float  mesures[3]    = {0, 0, 0}; // Mesures des angles [Yaw, Pitch, Roll]
float  lastErr[3]    = {0, 0, 0}; // Valeur de l'erreur précédente : composante Dérivée [Yaw, Pitch, Roll]
float  sErr[3];                      // Sommes des erreurs : composante Intégrale [Yaw, Pitch, Roll]
SimpleTimer timer;
// -------------- Variables pour la communication bluetooth ------------------
SoftwareSerial mySerial(8,9);             // BlueTooth module: pin#8=TX pin#9=RX                     /!\ TODO : inverser les pins pour le nouveau shield
// ---------------- Variables du MPU650 --------------------------------------
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
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

    timer.setInterval(1, asservissement);
        
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)    

    Serial.begin(57600); // Provoque beaucoup de FIFO Overflow en dessous de 38400 bauds
    mySerial.begin(9600);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Calibration du MPU
    mpu.setXAccelOffset(377);
    mpu.setYAccelOffset(-4033);
    mpu.setZAccelOffset(1591);
    mpu.setXGyroOffset(115);
    mpu.setYGyroOffset(101);
    mpu.setZGyroOffset(32);
    
    // Returns 0 if it worked
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0 : #pin2)..."));
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

    // Init the error sum
    sErr[0] = 0.; // Yaw
    sErr[1] = 0.; // Pitch
    sErr[2] = 0.; // Roll

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
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        //--- 1. Read instructions from serial port ---
        cmd = getSerialCommands();
        // dumpCommands(cmd);

        //--- 2. Read mesures from sensor ---
        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // Display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // TODO : set the resolution number
        mesures[0] = 0*ypr[0] * 180/M_PI; // Yaw   : lacet
        mesures[1] = ypr[1] * 180/M_PI; // Pitch : tangage
        mesures[2] = ypr[2] * 180/M_PI; // Roll  : rouli

        dumpMesures(mesures);
        
        //--- 3. Calculate errors compared to instructions ---
        errors = calcErrors(mesures, cmd);
        //dumpErrors(errors);
    }
}

/**
 * Configure les ESC
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
 * Rounds a float value 
 * 
 * @param float value
 * @param unsigned int precision : number of decimals
 */
float myRound(float value, unsigned int precision)
{
	  int coeff = pow(10.0, (float)precision);

	  value = (int)(value * coeff);

	  return (value / coeff);
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
 * Fonction d'asservissement principale
 */
void asservissement()
{
    static int commandes[4] = {0,0,0,0};
    float Kp[3]       = {0.0, 0.0, 0.06};       // Coefficient P dans l'ordre : Yaw, Pitch, Roll
    float Ki[3]       = {0.0, 0.0, 0.01};    // Coefficient I dans l'ordre : Yaw, Pitch, Roll
    float Kd[3]       = {0, 0, 25};//{0.65, 0.65, 0.65};             // Coefficients D dans l'ordre : Yaw, Pitch, Roll
    float deltaErr[3] = {0, 0, 0};             // Yaw, Pitch, Roll
  
    // Initialisation des commandes moteur
    float cmd_motA = cmd[3];
    float cmd_motB = cmd[3];
    float cmd_motC = cmd[3];
    float cmd_motD = cmd[3];

    // If throttle instruction > 0
    if (cmd[3] != 0) { 
        // Calcul la somme des erreurs : composante Intégrale
        sErr[0] += errors[0]; // Yaw
        sErr[1] += errors[1]; // Pitch
        sErr[2] += errors[2]; // Roll
        
        // Calcul du delta erreur : composante Dérivée
        deltaErr[0] = errors[0] - lastErr[0]; // Yaw
        deltaErr[1] = errors[1] - lastErr[1]; // Pitch
        deltaErr[2] = errors[2] - lastErr[2]; // Roll
        
        // Sauvegarde de la dernière erreur : composante Dérivée
        lastErr[0] = errors[0]; // Yaw
        lastErr[1] = errors[1]; // Pitch
        lastErr[2] = errors[2]; // Roll
        
        // Yaw - Lacet (Z)
        cmd_motA -= (errors[0] * Kp[0] + sErr[0] * Ki[0] + deltaErr[0] * Kd[0]);
        cmd_motD -= (errors[0] * Kp[0] + sErr[0] * Ki[0] + deltaErr[0] * Kd[0]);
        cmd_motC += (errors[0] * Kp[0] + sErr[0] * Ki[0] + deltaErr[0] * Kd[0]);
        cmd_motB += (errors[0] * Kp[0] + sErr[0] * Ki[0] + deltaErr[0] * Kd[0]);
        
        // Pitch - Tangage (Y)
        cmd_motA -= (errors[1] * Kp[1] + sErr[1] * Ki[1] + deltaErr[1] * Kd[1]);
        cmd_motB -= (errors[1] * Kp[1] + sErr[1] * Ki[1] + deltaErr[1] * Kd[1]);
        cmd_motC += (errors[1] * Kp[1] + sErr[1] * Ki[1] + deltaErr[1] * Kd[1]);
        cmd_motD += (errors[1] * Kp[1] + sErr[1] * Ki[1] + deltaErr[1] * Kd[1]);
        
        // Roll - Roulis (X)
        cmd_motA -= (errors[2] * Kp[2] + sErr[2] * Ki[2] + deltaErr[2] * Kd[2]);
        cmd_motC -= (errors[2] * Kp[2] + sErr[2] * Ki[2] + deltaErr[2] * Kd[2]);
        cmd_motB += (errors[2] * Kp[2] + sErr[2] * Ki[2] + deltaErr[2] * Kd[2]);
        cmd_motD += (errors[2] * Kp[2] + sErr[2] * Ki[2] + deltaErr[2] * Kd[2]);
    }

    // Cas limites [0, 180]
    commandes[0] = normaliser(cmd_motA);
    commandes[1] = normaliser(cmd_motB);
    commandes[2] = normaliser(cmd_motC);
    commandes[3] = normaliser(cmd_motD);
      
    //--- 5. Affectation des commandes ---
    motA.write(commandes[0]);
    motB.write(commandes[1]);
    motC.write(commandes[2]);
    motD.write(commandes[3]);

    //dumpSErrors(sErr);
}


