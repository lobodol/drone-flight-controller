/**
 * Contains all usual functions to read instructions from desired transmission channel.
 */

// ---------------------------- Local constants ------------------------------
#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3
// ----------------------------- Local variables -----------------------------
int throttle_pin = 0;
int roll_pin = 2;
// ---------------------------------------------------------------------------

/**
 * Returns hard-coded instructions for Yaw, Pitch, Roll & Throttle.
 * Values for yaw, pitch & roll are in degrees.
 *
 * @return float[4] : [Yaw, Pitch, Roll, Throttle] (values in degrees).
 */
float* getInstructions()
{
    static float commands[4];
    int roll     = analogRead(roll_pin);
    int throttle = analogRead(throttle_pin);

    // A positive value makes the drone rotate clockwise
    // A negative value makes the drone rotate counter-clockwise
    // Value range : [-180, 180]°
    commands[YAW]      = 0;  // Not implemented yet in the PID automation.

    // A positive value makes the drone lean backwards
    // A negative value makes the drone lean forwards
    // Value range : [-45, 45]°
    commands[PITCH]    = 0;

    // A positive value makes the drone lean to the right
    // A negative value makes the drone lean to the left
    // Value range : [-45, 45]°
    commands[ROLL]     = map(roll, 0, 1023, -45, 45); // Value in degrees.

    // A 100 value makes the motors run at full speed
    // A 0 value stops the motors.
    // Value range : [0, 100]
    commands[THROTTLE] = map(throttle, 0, 1023, 0, 100);

    return commands;
}
