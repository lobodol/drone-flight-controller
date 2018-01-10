
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
