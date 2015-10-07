// ---------------------------------------------------------------------------
#include <Wire.h>
#include <I2Cdev.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "asservissement.h"
#include "SoftwareSerial.h"
// ------------------- Déclaration des constantes ----------------------------
#define MAX_THROTTLE 180
#define STX          0x02
#define ETX          0x03
// ------------------ Déclaration des commandes moteurs ----------------------
Servo motA;
Servo motB;
Servo motC;
Servo motD;
// ------------- Variables pour l'asservissement -----------------------------
int*   cmd;                       // Commandes reçues : [Yaw, Pitch, Roll, Throttle]
int*   cmdMot;                    // Commandes à appliquer aux moteurs : [MotA, MotB, MotC, MotD]
int    lastThrottle  = 0;         // Dernière valeur lue
float* errors;                    // Erreurs mesurées : [Yaw, Pitch, Roll]
float  mesures[3]    = {0,0,0};   // Mesures des angles [Yaw, Pitch, Roll]
float  lastErr[3]    = {0, 0, 0}; // Valeur de l'erreur précédente : composante Dérivée [Yaw, Pitch, Roll]
//float* sErr[3]       = {0, 0, 0}; // Sommes des erreurs : composante Intégrale [Yaw, Pitch, Roll]
float* sErr;
// -------------- Variables pour la communication bluetooth ------------------
int etat             = 0;                        // 0 : OFF - 1 : ON
SoftwareSerial mySerial(8,9);                    // BlueTooth module: pin#8=TX pin#9=RX
byte rec[8]          = {0, 0, 0, 0, 0, 0, 0, 0}; // bytes received
byte buttonStatus    = 0;                        // first Byte sent to Android device
long previousMillis  = 0;                        // will store last time Buttons status was updated
String displayStatus = "xxxx";                   // message to Android device
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int x = 0;

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
        
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)    

    Serial.begin(57600); // Provoque beaucoup de FIFO Overflow en dessous de 38400 bauds
    mySerial.begin(9600); // TODO vérifier si on peut augmenter le bitrate

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
    sErr[0] = 0; // Yaw
    sErr[1] = 0; // Pitch
    sErr[2] = 0; // Roll

    motA.attach(4, 1000, 2000);
    motB.attach(5, 1000, 2000);
    motC.attach(6, 1000, 2000);
    motD.attach(7, 1000, 2000);

    initialize_motor();
}


/**
 * Main program loop
 */
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
        getState();
        cmd = getCommandes(etat);
        //cmd = getCommands();

        /*
        Serial.print("Commandes : ");
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

        // TODO : set the resolution number
        mesures[0] = 0*ypr[0] * 180/M_PI; // Yaw   : lacet
        mesures[1] = ypr[1] * 180/M_PI; // Pitch : tangage
        mesures[2] = ypr[2] * 180/M_PI; // Roll  : rouli

        /*
        Serial.print(mesures[0]);
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
        cmdMot[0] = 0;
        cmdMot[1] = 0;
        cmdMot[2] = 0;
        cmdMot[3] = 0;
        cmdMot = asservissementP(errors, cmd[3]);

        //--- 5. Affectation des commandes ---
        motA.write(cmdMot[0]);
        motB.write(cmdMot[1]);
        motC.write(cmdMot[2]);
        motD.write(cmdMot[3]);


        /*             
        Serial.print("Mot A : ");
        Serial.print(cmdMot[0]);
        Serial.print("\tMot B : ");        
        Serial.print(cmdMot[1]);
        Serial.print("\nMot C : ");
        Serial.print(cmdMot[2]);
        Serial.print("\tMot D : ");
        Serial.print(cmdMot[3]);
        Serial.print("\n\n");*/
    }

    x++;
}

/**
 * Configure les ESC
 */
void initialize_motor()
{
    Serial.print("Arming the motor! \n");
    delay(3000);
    /*Serial.print("Setting low speed! \n");

    motA.write(0);
    motB.write(0);
    motC.write(0);
    motD.write(0);
    delay(4000);
    
    Serial.print("Setting high speed! \n");
    motA.write(180);
    motB.write(180);
    motC.write(180);
    motD.write(180);
    delay(4000);*/
        
    motA.write(10);
    motB.write(10);
    motC.write(10);
    motD.write(10);
    Serial.print("MOTOR IS READY! \n");
    delay(2000);
}

/**
 * Returns the throttle joystick value 
 *
 * @param byte[] : received data from serial port
 * @return int
 */
int getJoystickState(byte data[8])
{
    int joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
    int joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
    joyX = joyX - 200;                                                  // Offset to avoid
    joyY = joyY - 200;                                                  // transmitting negative numbers

    if (joyX<-100 || joyX>100 || joyY<-100 || joyY>100) {
   		  return 0;      // commmunication error
    }

    return map(joyY, -100, 100, 0, 180);
}


/**
 * Read commands from the serial port
 * @return int* : array of commands[yaw, pitch, roll, throttle]
 */
int* getCommands()
{
    static int cmdMot[4] = {0, 0, 0, 0};
    if (mySerial.available()) {                           // data received from smartphone
        delay(2);
        rec[0] =  mySerial.read();  
    
        if (rec[0] == STX) { // START bit
            int i=1;
      
            while (mySerial.available()) {
                delay(1);
                rec[i] = mySerial.read();
        
                if (rec[i]>127 || i>7) {
                    break;     // Communication error
                }
        
                if ((rec[i]==ETX) && (i==2 || i==7)) {
                    break;     // Button or Joystick data
                }
        
                i++;
            }
      
            if (i==7) {
                cmdMot[3] = getJoystickState(rec);     // 6 Bytes  ex: < STX "200" "180" ETX >
                lastThrottle = cmdMot[3];
            }
        }
    }

    return cmdMot;
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
 * Get the state from the bluetooth module : 0 = off; 1 = on
 */
void getState()
{
    if (mySerial.available()) {                           // data received from smartphone
        delay(2);
        rec[0] =  mySerial.read();

        //Serial.print(rec[0]);

        if (rec[0] == 48) {
            etat = 0;
        } else if (rec[0] == 49) {
            etat = 1;
        }

        //Serial.println(etat);
    }
}
