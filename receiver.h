/**
 * Contains all usual functions to read instructions from receiver.
 *
 * @author lobodol <grobodol@gmail.com>
 */

// ---------------------------------------------------------------------------
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

// ---------------------------------------------------------------------------
// Received instructions formatted with good units : [Yaw, Pitch, Roll, Throttle]
float instruction[4];

// Previous state of each channel (HIGH or LOW)
byte previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel.
unsigned long current_time;
unsigned long timer[4];

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel.
int mode_mapping[4];

// ---------------------------------------------------------------------------
/**
 * Calculate real value of flight instructions from pulses length of each channel.
 *
 * - Roll     : from -33° to 33°
 * - Pitch    : from -33° to 33°
 * - Yaw      : from -180° to 180°
 * - Throttle : from 0 to 144 (no unit)
 *
 * @return void
 */
void getFlightInstruction()
{
    instruction[YAW]      = map(pulse_length[mode_mapping[YAW]], 1000, 2000, -180, 180);
    instruction[PITCH]    = map(pulse_length[mode_mapping[PITCH]], 1000, 2000, -33, 33);
    instruction[ROLL]     = map(pulse_length[mode_mapping[ROLL]], 1000, 2000, -33, 33);
    instruction[THROTTLE] = map(pulse_length[mode_mapping[THROTTLE]], 1000, 2000, 0, 144);
}

/**
 * Customize mapping of controls: set here which command is on which channel.
 *
 * @return void
 */
void configureChannelMapping()
{
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

/**
 * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
 * Read the receiver signals in order to get flight instructions.
 *
 * This routine must be as fast as possible to prevent main program to be messed up.
 * The trick here is to use port registers to read pin state.
 * Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
 * It is less convenient but more efficient, which is the most important here.
 *
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 */
ISR(PCINT0_vect) {
        current_time = micros();

        // Channel 1 -------------------------------------------------
        if (PINB & B00000001) {                                        // Is input 8 high ?
            if (previous_state[CHANNEL1] == LOW) {                     // Input 8 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL1] = HIGH;                       // Save current state
                timer[CHANNEL1] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL1] == HIGH) {                 // Input 8 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL1] = LOW;                            // Save current state
            pulse_length[CHANNEL1]   = current_time - timer[CHANNEL1]; // Calculate pulse duration & save it
        }

        // Channel 2 -------------------------------------------------
        if (PINB & B00000010) {                                        // Is input 9 high ?
            if (previous_state[CHANNEL2] == LOW) {                     // Input 9 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL2] = HIGH;                       // Save current state
                timer[CHANNEL2] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL2] == HIGH) {                 // Input 9 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL2] = LOW;                            // Save current state
            pulse_length[CHANNEL2]   = current_time - timer[CHANNEL2]; // Calculate pulse duration & save it
        }

        // Channel 3 -------------------------------------------------
        if (PINB & B00000100) {                                        // Is input 10 high ?
            if (previous_state[CHANNEL3] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL3] = HIGH;                       // Save current state
                timer[CHANNEL3] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL3] == HIGH) {                 // Input 10 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL3] = LOW;                            // Save current state
            pulse_length[CHANNEL3]   = current_time - timer[CHANNEL3]; // Calculate pulse duration & save it
        }

        // Channel 4 -------------------------------------------------
        if (PINB & B00001000) {                                        // Is input 11 high ?
            if (previous_state[CHANNEL4] == LOW) {                     // Input 11 changed from 0 to 1 (rising edge)
                previous_state[CHANNEL4] = HIGH;                       // Save current state
                timer[CHANNEL4] = current_time;                        // Save current time
            }
        } else if (previous_state[CHANNEL4] == HIGH) {                 // Input 11 changed from 1 to 0 (falling edge)
            previous_state[CHANNEL4] = LOW;                            // Save current state
            pulse_length[CHANNEL4]   = current_time - timer[CHANNEL4]; // Calculate pulse duration & save it
        }
}
