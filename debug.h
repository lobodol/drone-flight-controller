/**
 * Dump input commands on serial port
 *
 * @param int cmd[4] : array of commands (yaw, pitch, roll, throttle)
 */
void dumpCommands(int cmd[4]) {
    Serial.print("Commandes : ");

    Serial.print(cmd[0]);
    Serial.print(";");
    Serial.print(cmd[1]);
    Serial.print(";");
    Serial.print(cmd[2]);
    Serial.print(";");
    Serial.print(cmd[3]);
    Serial.print("\n");
}

/**
 * Dump sensor's mesures on serial port
 *
 * @param float mesures[3] : array of mesures
 */
void dumpMeasures(float *mesures) {
    Serial.print("Mesures : ");

    Serial.print(mesures[0]);
    Serial.print(";");
    Serial.print(mesures[1]);
    Serial.print(";");
    Serial.println(mesures[2]);
}

/**
 * Dump calculated errors on serial port
 *
 * @param float errors[3] : array of errors (yaw, pitch, roll)
 */
void dumpErrors(float errors[3]) {
    Serial.print("Erreurs : ");

    Serial.print(errors[0]);
    Serial.print(";");
    Serial.print(errors[1]);
    Serial.print(";");
    Serial.println(errors[2]);
}


/**
 * Dump calculated errors on serial port
 *
 * @param float errors[3] : array of errors (yaw, pitch, roll)
 */
void dumpSErrors(float errors[3]) {
    Serial.print("Somme erreurs : ");

    Serial.print(errors[0]);
    Serial.print(";");
    Serial.print(errors[1]);
    Serial.print(";");
    Serial.println(errors[2]);
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

