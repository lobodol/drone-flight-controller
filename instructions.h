/**
 * Contains all usual functions to read instructions from differents supports.
 * USB wire
 * Bluetooth module
 * ...
 */

// ---------------------------------------------------------------------------
#include "SoftwareSerial.h"
// ---------------------------- Local constants ------------------------------
#define STX          0x02
#define ETX          0x03
// --------------------------- Local variables -------------------------------
byte rec[8]      = {0, 0, 0, 0, 0, 0, 0, 0}; // Bytes received
int lostTimes    = 0;
int lastThrottle = 0;                        // Last read value
// ---------------------------------------------------------------------------


/**
 * Converts ascii numbers to integer
 * 
 * @param byte data[8] : array of read instructions
 */
int asciiToInt(byte data[8]) {
  int result = 0;

    result += (data[0]-48)*100 + (data[1]-48)*10 + (data[2]-48);

    if (result > 100) {
      result = 100;
    } else if (result < 0) {
      result = 0;
    }
  
  return result;
}

/**
 * Reads instructions from USB port.
 * Throttle instruction goes from 0 to 100.
 * 
 * @return int* : an array of instrcutions : yaw, pitch, roll, throttle
 */
int* getSerialCommands()
{
    // Yaw, Pitch, Roll, throttle
    static int cmdMot[4] = {0, 0, 0, 0};
    int i = 0;
    
    while (Serial.available()) {
      rec[i] = Serial.read();
      
      if (i >= 2) {
        break;
      }

      i++;
    }

    cmdMot[3] = asciiToInt(rec);

    return cmdMot;
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
 * Read commands from the bluetooth module
 * 
 * @return int* : array of commands[yaw, pitch, roll, throttle]
 */
int* getCommands(SoftwareSerial mySerial)
{    
    static int cmdMot[4] = {0, 0, 0, 0};
    if (mySerial.available()) {                           // data received from smartphone
        //Serial.println("serial available");
        delay(2);
        rec[0] =  mySerial.read();  
    
        if (rec[0] == STX) { // START bit
            //Serial.println("START bit");
            int i=1;
      
            while (mySerial.available()) {
                delay(1);
                rec[i] = mySerial.read();
        
                if (rec[i]>127 || i>7) {
                    break;     // Communication error
                    //Serial.println("Communication error");
                }
        
                if ((rec[i]==ETX) && (i==2 || i==7)) {
                    break;     // Button or Joystick data
                    //Serial.println("Data");
                }
        
                i++;
            }
      
            if (i==7) {
                cmdMot[3] = getJoystickState(rec);     // 6 Bytes  ex: < STX "200" "180" ETX >
                lastThrottle = cmdMot[3];
            }

            lostTimes = 0;
        } else {
            lostTimes++;
            //Serial.println("Signal lost");
        }
    } else {
        lostTimes++;
        //Serial.println("serial not available");
    }

    if (lostTimes > 3) {
        //Serial.println("Signal seems lost...");
    }

    return cmdMot;
}
