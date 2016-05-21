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
void dumpMesures(float mesures[3]) {
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

