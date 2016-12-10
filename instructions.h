/**
 * Contains all usual functions to read instructions from desired transmission channel.
 */

// ----------------------------Local constants ------------------------------
#define YAW            0
#define PITCH          1
#define ROLL           2
#define THROTTLE       3
// ---------------------------------------------------------------------------

/**
 * Returns hard-coded instrcutions for Yaw, Pitch, Roll & Trhottle.
 * Values for yaw, pitch & roll are in degrees.
 *
 * TODO: Use a simple potentiometer to give dynamic commands.
 *
 * @return float[4] : [Yaw, Pitch, Roll, Throttle]
 */
float* getInstructions()
{
    static float commands[4];

    commands[YAW]      = 0; // Value in degrees.
    commands[PITCH]    = 0; // Value in degrees.
    commands[ROLL]     = 0; // Value in degrees.
    commands[THROTTLE] = 45; // Value over 100.

    return commands;
}
