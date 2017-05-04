/**
 * Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the mesure and the command.
 *
 * @param float[3] measures : array of the measures in that order : Yaw, Pitch, Roll.
 * @param float[3] cmd      : array of commands in that order : Yaw, Pitch Roll.
 * @return float[3]         : array of calculated errors in that order : Yaw, Pitch, Roll.
 */
float *calcErrors(float measures[3], float cmd[3])
{
    static float errors[3];

    errors[YAW]   = measures[YAW]   - cmd[YAW];
    errors[PITCH] = measures[PITCH] - cmd[PITCH];
    errors[ROLL]  = measures[ROLL]  - cmd[ROLL];

    return errors;
}

/**
 * Cast float into integer and rectify command : if is greater than 180 or lower than 0.
 * The returned value is a integer between 0 and 180 (both included).
 * This is directly linked to the PPM command that must be an angle between 0 and 180 degrees.
 *
 * @param float value : the value to format.
 * @return int : [0, 180]
 */
int normalize(float value)
{
    int maxVal = 180;
    value = (int) value;

    if (value > maxVal) {
        value = maxVal;
    } else if (value < 0) {
        value = 0;
    }

    return value;
}
