/**
 * Calcul les erreurs Yaw, Pitch, Roll par rapport aux commandes
 *
 * @param float* mesures : tableau des mesures Yaw, Pitch, Roll
 * @param float* cmd     : tableau des commandes Yaw, Pitch Roll
 * @return float*        : tableau des erreurs Yaw, Pitch, Roll
 */
float* calcErrors(float mesures[3], float cmd[3])
{
  static float errors[3];

  errors[YAW]   = mesures[YAW]   - cmd[YAW];
  errors[PITCH] = mesures[PITCH] - cmd[PITCH];
  errors[ROLL]  = mesures[ROLL]  - cmd[ROLL];

  return errors;
}

/**
 * Rectifie une commande si elle est supérieure à 180 ou inférieure à 0
 *
 * @param float value
 * @return int : [0, 180]
 */
int normalize(float value)
{
  int maxVal = 180;
  value      = (int)value;

  if (value > maxVal) {
    value = maxVal;
  } else if (value < 0) {
    value = 0;
  }

  return value;
}
