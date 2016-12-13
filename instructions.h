/**
 * Contains all usual functions to read instructions from desired transmission channel.
 */

// ----------------------------Local constants ------------------------------
#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3
// ---------------------------------------------------------------------------

/**
 * Returns hard-coded instrcutions for Yaw, Pitch, Roll & Trhottle.
 * Values for yaw, pitch & roll are in degrees.
 *
 * TODO: Use a simple potentiometer to give dynamic commands.
 *
 * @return float[4] : [Yaw, Pitch, Roll, Throttle] (values in degrees).
 */
float* getInstructions()
{
    static float commands[4];

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
    commands[ROLL]     = 0; // Value in degrees.

    // A 100 value makes the motors run at full speed
    // A 0 value stops the motors.
    // Value range : [0, 100]
    commands[THROTTLE] = 45;

    return commands;
}
