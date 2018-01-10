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