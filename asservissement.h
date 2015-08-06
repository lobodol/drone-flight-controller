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
 * Lecture des commandes : Yaw, Pitch, Roll, Gaz
 * @return int[4] : tableau des commandes
 */
int* getCommandes()
{
  static int cmd[4];
  
  cmd[0] = 0;  // Lacet
  cmd[1] = 0;  // Tantgage
  cmd[2] = 0;  // Rouli
  cmd[3] = 50; // Gaz

  return cmd;
}

/**
 * Rectifie une commande si elle est supérieure à 1024 ou inférieure à 0
 * @param float value
 * @return int : [0, 1024]
 */
int normaliser(float value)
{
  value = (int)value;
  
	if (value > 1024) {
		value = 1024;
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
  int Kp = 1; // Coefficient de proportionnalité
  int aKp[3] = {1, 1, 1};
  static int commandes[4];

  // Initialisation des commandes moteur
  int cmd_motA = cmd_h;
  int cmd_motB = cmd_h;
  int cmd_motC = cmd_h;
  int cmd_motD = cmd_h;

  // Lacet (Z)
  cmd_motA -= errors[0] * Kp;
  cmd_motD -= errors[0] * Kp;
  cmd_motC += errors[0] * Kp;
  cmd_motB += errors[0] * Kp;

  // Tangage (Y)
  cmd_motA -= errors[1] * Kp;
  cmd_motB -= errors[1] * Kp;
  cmd_motC += errors[1] * Kp;
  cmd_motD += errors[1] * Kp;
  
  // Roulis (X)
  cmd_motA -= errors[2] * Kp;
  cmd_motC -= errors[2] * Kp;
  cmd_motB += errors[2] * Kp;
  cmd_motD += errors[2] * Kp;

  // Cas limites [0, 255]
  commandes[0] = normaliser(cmd_motA);
  commandes[1] = normaliser(cmd_motB);
  commandes[2] = normaliser(cmd_motC);
  commandes[3] = normaliser(cmd_motD);

  return commandes;
}
