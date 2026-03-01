#pragma once

/**
 * Clamp a value within a [min_value, max_value] range.
 *
 * @param float value     : The value to clamp
 * @param float min_value : The lower bound of the range
 * @param float max_value : The upper bound of the range
 *
 * @return float
 */
inline float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        return max_value;
    } else if (value < min_value) {
        return min_value;
    }
    return value;
}

/**
 * Calculate the PID set point in °/s for a given axis.
 *
 * A dead band of 16µs around the stick center (1492–1508) prevents tiny
 * stick movements from producing a non-zero set point. Outside the dead band,
 * the stick deviation is reduced by a level-adjust term proportional to the
 * current angle, which limits the maximum tilt to ±32.8°.
 *
 * @param float angle         Measured angle on the axis (in °)
 * @param int   channel_pulse Receiver pulse length for the axis (in µs, nominally 1000–2000)
 *
 * @return float Set point in °/s
 */
inline float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
    float set_point    = 0;

    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

/**
 * Calculate the PID set point of the YAW axis in °/s.
 *
 * Yaw has no notion of absolute angle (the drone can spin freely), so the
 * level-adjust term used for pitch and roll does not apply. The set point is
 * zeroed when the throttle is at or below the disarm threshold (≤ 1050µs) to
 * prevent unwanted yaw rotation while the motors are spinning down.
 *
 * @param int yaw_pulse      Receiver pulse length of the yaw channel (in µs)
 * @param int throttle_pulse Receiver pulse length of the throttle channel (in µs)
 *
 * @return float Set point in °/s
 */
inline float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
    // Do not yaw when turning off the motors
    if (throttle_pulse > 1050) {
        // There is no notion of angle on this axis as the quadcopter can turn on itself
        return calculateSetPoint(0, yaw_pulse);
    }

    return 0;
}
