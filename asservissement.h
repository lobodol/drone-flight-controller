/**
 * Calcul les erreurs Yaw, Pitch, Roll par rapport aux commandes
 * @param float* mesures : tableau des mesures Yaw, Pitch, Roll
 * @param int* cmd       : tableau des commandes Yaw, Pitch Roll
 * @return float*        : tableau des erreurs Yaw, Pitch, Roll
 */
float* calcErrors(float mesures[3], int cmd[3])
{
  static float errors[3];

  errors[0] = mesures[0] - cmd[0]; // Yaw
  errors[1] = mesures[1] - cmd[1]; // Pitch
  errors[2] = mesures[2] - cmd[2]; // Roll

  return errors;
}

/**
 * Rectifie une commande si elle est supérieure à 180 ou inférieure à 0
 * @param float value
 * @return int : [0, 180]
 */
int normaliser(float value)
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

/**
 * Calcul les rapports cycliques de signaux de contrôle de chacun des moteurs en fonctions des erreurs par rapport à la consigne
 * pour un quadricoptère de type X : 
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 * 
 * Les moteurs A et D tournent dans le sens horaire
 * Les moteurs B et C tournent dans le sens anti-horaire
 *
 * Asservissement de type Proportionnel
 *
 * @param float[3] errors : tableau des erreurs Yaw, Pitch, Roll
 * @param float cmd_h     : commande des gaz
 */
int* asservissementP(float errors[3], int cmd_h)
{
  //int Kp = 1; // Coefficient de proportionnalité
  float Kp[3] = {1.5, 1.5, 1}; // Coefficient P dans l'ordre : Yaw, Pitch, Roll
  static int commandes[4] = {0,0,0,0};

  if (cmd_h == 0) {
      return commandes;
  }

  // Initialisation des commandes moteur
  int cmd_motA = cmd_h;
  int cmd_motB = cmd_h;
  int cmd_motC = cmd_h;
  int cmd_motD = cmd_h;

  // Yaw - Lacet (Z)
  cmd_motA -= errors[0] * Kp[0];
  cmd_motD -= errors[0] * Kp[0];
  cmd_motC += errors[0] * Kp[0];
  cmd_motB += errors[0] * Kp[0];

  // Pitch - Tangage (Y)
  cmd_motA -= errors[1] * Kp[1];
  cmd_motB -= errors[1] * Kp[1];
  cmd_motC += errors[1] * Kp[1];
  cmd_motD += errors[1] * Kp[1];

  // Roll - Roulis (X)
  cmd_motA -= errors[2] * Kp[2];
  cmd_motC -= errors[2] * Kp[2];
  cmd_motB += errors[2] * Kp[2];
  cmd_motD += errors[2] * Kp[2];

  // Cas limites [0, 180]
  commandes[0] = normaliser(cmd_motA);
  commandes[1] = normaliser(cmd_motB);
  commandes[2] = normaliser(cmd_motC);
  commandes[3] = normaliser(cmd_motD);

  return commandes;
}

/**
 * Asservissement de type Proportionnel Intégral
 * @param float[3] errors :
 * @param int cmd_h : commande des gaz (throttle)
 */
int* asservissementPI(float errors[3], int cmd_h, float* sErr)
{
  static int commandes[4];
  float Kp[3] = {1.5, 1.5, 1}; // Coefficient P dans l'ordre : Yaw, Pitch, Roll
  float Ki[3] = {1, 1, 1}; // Coefficient I dans l'ordre : Yaw, Pitch, Roll

  // Initialisation des commandes moteur
  int cmd_motA = cmd_h;
  int cmd_motB = cmd_h;
  int cmd_motC = cmd_h;
  int cmd_motD = cmd_h;

  // Calcul la somme des erreurs : composante Intégrale
  sErr[0] += errors[0]; // Yaw
  sErr[1] += errors[1]; // Pitch
  sErr[2] += errors[2]; // Roll

  // Yaw
  cmd_motA -= (errors[0] * Kp[0] + sErr[0] * Ki[0]);
  cmd_motD -= (errors[0] * Kp[0] + sErr[0] * Ki[0]);
  cmd_motC += (errors[0] * Kp[0] + sErr[0] * Ki[0]);
  cmd_motB += (errors[0] * Kp[0] + sErr[0] * Ki[0]);

  // Pitch - Tangage (Y)
  cmd_motA -= (errors[1] * Kp[1] + sErr[1] * Ki[1]);
  cmd_motB -= (errors[1] * Kp[1] + sErr[1] * Ki[1]);
  cmd_motC += (errors[1] * Kp[1] + sErr[1] * Ki[1]);
  cmd_motD += (errors[1] * Kp[1] + sErr[1] * Ki[1]);
  
  // Roll - Roulis (X)
  cmd_motA -= (errors[2] * Kp[2] + sErr[2] * Ki[2]);
  cmd_motC -= (errors[2] * Kp[2] + sErr[2] * Ki[2]);
  cmd_motB += (errors[2] * Kp[2] + sErr[2] * Ki[2]);
  cmd_motD += (errors[2] * Kp[2] + sErr[2] * Ki[2]);

  // Cas limites [0, 180]
  commandes[0] = normaliser(cmd_motA);
  commandes[1] = normaliser(cmd_motB);
  commandes[2] = normaliser(cmd_motC);
  commandes[3] = normaliser(cmd_motD);

  return commandes;
}

/**
 * Asservissement PID
 */
int* asservissementPID(float errors[3], int cmd_h, float* sErr, float* lastErr)
{
    static int commandes[4];
    int Kp[3] = {1, 1, 1}; // Coefficients P dans l'ordre : Yaw, Pitch, Roll
    int Ki[3] = {1, 1, 1}; // Coefficients I dans l'ordre : Yaw, Pitch, Roll
    int Kd[3] = {1, 1, 1}; // Coefficients D dans l'ordre : Yaw, Pitch, Roll
  
    // Initialisation des commandes moteur
    int cmd_motA = cmd_h;
    int cmd_motB = cmd_h;
    int cmd_motC = cmd_h;
    int cmd_motD = cmd_h;
  
    // Calcul la somme des erreurs : composante Intégrale
    sErr[0] += errors[0]; // Yaw
    sErr[1] += errors[1]; // Pitch
    sErr[2] += errors[2]; // Roll
  
    // Yaw - Lacet (Z)
    cmd_motA -= (errors[0] * Kp[0] + sErr[0] * Ki[0] + lastErr[0] * Kd[0]);
    cmd_motD -= (errors[0] * Kp[0] + sErr[0] * Ki[0] + lastErr[0] * Kd[0]);
    cmd_motC += (errors[0] * Kp[0] + sErr[0] * Ki[0] + lastErr[0] * Kd[0]);
    cmd_motB += (errors[0] * Kp[0] + sErr[0] * Ki[0] + lastErr[0] * Kd[0]);
  
    // Pitch - Tangage (Y)
    cmd_motA -= (errors[1] * Kp[1] + sErr[1] * Ki[1] + lastErr[1] * Kd[1]);
    cmd_motB -= (errors[1] * Kp[1] + sErr[1] * Ki[1] + lastErr[1] * Kd[1]);
    cmd_motC += (errors[1] * Kp[1] + sErr[1] * Ki[1] + lastErr[1] * Kd[1]);
    cmd_motD += (errors[1] * Kp[1] + sErr[1] * Ki[1] + lastErr[1] * Kd[1]);
  
    // Roll - Roulis (X)
    cmd_motA -= (errors[2] * Kp[2] + sErr[2] * Ki[2] + lastErr[2] * Kd[2]);
    cmd_motC -= (errors[2] * Kp[2] + sErr[2] * Ki[2] + lastErr[2] * Kd[2]);
    cmd_motB += (errors[2] * Kp[2] + sErr[2] * Ki[2] + lastErr[2] * Kd[2]);
    cmd_motD += (errors[2] * Kp[2] + sErr[2] * Ki[2] + lastErr[2] * Kd[2]);
  
    // Sauvegarde de la dernière erreur : composante Dérivée
    lastErr[0] = errors[0]; // Yaw
    lastErr[1] = errors[1]; // Pitch
    lastErr[2] = errors[2]; // Roll
  
    // Cas limites [0, 180]
    commandes[0] = normaliser(cmd_motA);
    commandes[1] = normaliser(cmd_motB);
    commandes[2] = normaliser(cmd_motC);
    commandes[3] = normaliser(cmd_motD);
  
    return commandes;
}
