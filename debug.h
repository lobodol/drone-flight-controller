/**
 * Dump input commands on serial port
 *
 * @param float cmd[4] : array of commands (yaw, pitch, roll, throttle)
 */
void dumpCommands(float* cmd) {
    Serial.print("Commandes : ");

    Serial.print(cmd[YAW]);
    Serial.print(";");
    Serial.print(cmd[PITCH]);
    Serial.print(";");
    Serial.print(cmd[ROLL]);
    Serial.print(";");
    Serial.print(cmd[THROTTLE]);
    Serial.print("\n");
}

/**
 * Dump sensor's mesures on serial port
 *
 * @param float mesures[3] : array of mesures
 */
void dumpMeasures(float *mesures) {
    Serial.print("Mesures : ");

    Serial.print(mesures[YAW]);
    Serial.print(";");
    Serial.print(mesures[PITCH]);
    Serial.print(";");
    Serial.println(mesures[ROLL]);
}

/**
 * Dump calculated errors on serial port
 *
 * @param float errors[3] : array of errors (yaw, pitch, roll)
 */
void dumpErrors(float errors[3]) {
    Serial.print("Erreurs : ");

    Serial.print(errors[YAW]);
    Serial.print(";");
    Serial.print(errors[PITCH]);
    Serial.print(";");
    Serial.println(errors[ROLL]);
}


/**
 * Dump calculated errors on serial port
 *
 * @param float errors[3] : array of errors (yaw, pitch, roll)
 */
void dumpSErrors(float errors[3]) {
    Serial.print("Somme erreurs : ");

    Serial.print(errors[YAW]);
    Serial.print(";");
    Serial.print(errors[PITCH]);
    Serial.print(";");
    Serial.println(errors[ROLL]);
}

/**
 * Dump motor commands. Each command must be in [0, 180]°.
 *
 * @param float cmdA : command motor A.
 * @param float cmdB : command motor B.
 * @param float cmdC : command motor C.
 * @param float cmdD : command motor D.
 */
void dumpCmdMot(float cmdA, float cmdB, float cmdC, float cmdD) {
    Serial.println("Commandes moteurs:");

    Serial.print(normalize(cmdA));
    Serial.print(";");
    Serial.println(normalize(cmdB));
    Serial.print(normalize(cmdC));
    Serial.print(";");
    Serial.println(normalize(cmdD));
}

