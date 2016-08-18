/**
 * Contains all usual functions to read instructions from virtual wire.
 */

// ---------------------------------------------------------------------------
#include <VirtualWire.h>
// ----------------------------Local constants ------------------------------
#define CHECKSUM_INDEX 8
#define PRECISION      1
#define STX_INDEX      0
#define ETX_INDEX      9
#define STX            0x02
#define ETX            0x03
#define MESSAGE_LENGTH 10
#define YAW            0
#define PITCH          1
#define ROLL           2
#define THROTTLE       3
// --------------------------- Local variables -------------------------------
byte   rec[MESSAGE_LENGTH];
int    lostTimes = 0;
float* lastInstructions;
// ---------------------------------------------------------------------------

/**
 * Return TRUE if message is well formed, FALSE otherwise.
 * Save receveided datas in "rec" variable & increment "lostTime" each time message isn't valid.
 * A message is 10 bytes length and must be composed as following :
 * << STX (1B) | YAW (2B) | PITCH (2B) | ROLL (2B) | THROTTLE (1B) | CHECKSUM (1B) | ETX (1B) >>
 *
 * @return bool
 */
bool isMessageValid()
{
    bool    isValid = false;
    uint8_t buflen  = MESSAGE_LENGTH;

    // Wait for incoming message
    if (vw_wait_rx_max(133) && vw_get_message(rec, &buflen)) {
        // Start byte
        if (rec[STX_INDEX] == STX) {
            // Init checksum
            byte checksum = rec[STX_INDEX];

            for (byte i = STX_INDEX+1; i < buflen; i++) {
                // Calculate checksum
                if (i < CHECKSUM_INDEX) {
                    // Bitwise XOR
                    checksum ^= rec[i];
                } else if (i == CHECKSUM_INDEX && checksum != rec[CHECKSUM_INDEX]) {
                    // Checksum error
                    Serial.println("Checksum error");
                    break;
                } else if (i == ETX_INDEX && rec[i] == ETX) {
                    // Everything went good
                    isValid = true;
                    break;
                } else if (i >= ETX_INDEX) {
                    // Message is too long, something went wrong
                    Serial.println("Message too long");
                    break;
                }
            }
        } else {
            // Never encountered the STX symbol
            Serial.println("--- NO STX ---");
        }
    } else {
        Serial.println("NO DATA RECEIVED");
    }

    if (!isValid) {
        lostTimes++;
        Serial.println("Message is NOT valid");
    } else {
        lostTimes = 0;
    }

    return isValid;
}

/**
 * Extract yaw value from received datas
 *
 * @return float : yaw value in [0, 360]°
 */
float getYaw()
{
    //                 MSB            LSB
    uint16_t yaw = (rec[1] << 8) | rec[2];

    // Limit cases
    if (yaw > 360) {
        yaw = 360;
    } else if (yaw < 0) {
        yaw = 0;
    }

    return float(yaw);
}

/**
 * Extract pitch value from received datas.
 *
 * @return float : pitch value in [-45.0, +45.0]°
 */
float getPitch()
{
    //                 MSB            LSB
    uint16_t pitch = (rec[3] << 8) | rec[4];

    // Limit cases
    if (pitch > 900) {
        pitch = 900;
    } else if (pitch < 0) {
        pitch = 0;
    }

    float pitch_float = pitch / pow(10, PRECISION);
    pitch_float -= 45;

    return pitch_float;
}

/**
 * Extract roll value from received datas.
 *
 * @return float : pitch value in [-45.0, +45.0]°
 */
float getRoll()
{
    //                MSB            LSB
    uint16_t roll = (rec[5] << 8) | rec[6];

    // Limit cases
    if (roll > 900) {
        roll = 900;
    } else if (roll < 0) {
        roll = 0;
    }

    float roll_float = roll / pow(10, PRECISION);
    roll_float -= 45;

    return roll_float;
}

/**
 * Extract throttle value from received datas.
 *
 * @return float
 */
float getThrottle()
{
    float throttle = float(rec[7]);

    if (throttle < 0) {
        throttle = 0;
    } else if (throttle > 100) {
        throttle = 100;
    }

    return throttle;
}

/**
 * Read instructions from virtual wire
 *
 * @return float[4] : [Yaw, Pitch, Roll, Throttle]
 */
float* getInstructions()
{
    static float commands[4] = {0,0,0,0};

    if (isMessageValid()) {
        commands[YAW]      = getYaw();
        commands[PITCH]    = getPitch();
        commands[ROLL]     = getRoll();
        commands[THROTTLE] = getThrottle();

        // Save insructions for next time
        lastInstructions[YAW]      = commands[YAW];
        lastInstructions[PITCH]    = commands[PITCH];
        lastInstructions[ROLL]     = commands[ROLL];
        lastInstructions[THROTTLE] = commands[THROTTLE];
    } else if (lostTimes > 255) {
        // Communication lost for too long, start landing process
        //Serial.println("Landing...");
    } else if (lostTimes > 3) {
        // Communication seems lost, stabilize quadcopter using default values and wait
        //Serial.println("Stabilize...");
        commands[YAW]      = 0;
        commands[PITCH]    = 0;
        commands[ROLL]     = 0;

        if (lastInstructions[THROTTLE] > 50) {
            commands[THROTTLE] = 50;
        } else {
            commands[THROTTLE] = lastInstructions[THROTTLE];
        }
    } else {
        // Communication error, but not for too long yet : use last instructions
        commands[YAW]      = lastInstructions[YAW];
        commands[PITCH]    = lastInstructions[PITCH];
        commands[ROLL]     = lastInstructions[ROLL];
        commands[THROTTLE] = lastInstructions[THROTTLE];
        //Serial.println("Use last instructions");
    }

    return commands;
}