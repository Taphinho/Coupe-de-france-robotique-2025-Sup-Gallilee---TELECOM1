#include "Motor.h"
#include "Mouvement.h"
#include <Arduino.h>

void initMD25() {
  Wire.begin();
  Wire.setClock(100000);
}
bool md25Ready() {
  Wire.beginTransmission(0x58); // Adresse I2C par défaut du MD25
  return Wire.endTransmission() == 0; // Si 0, le MD25 est détecté
}

void resetEncodeurs() {
  enc1_avant=0 , enc2_avant=0 ;
  Wire.beginTransmission(MD25);
  Wire.write(REG_COMMAND);
  Wire.write(RESET_ENCODER);
  Wire.endTransmission();
  delay(10);
}

long lireEncodeur(int registre) {
  // 1. Réinitialise la communication I2C pour éviter les erreurs
  Wire.beginTransmission(MD25);
  Wire.write(registre);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("Erreur I2C: ");
    Serial.println(error);
    return 0; // Retourne 0 en cas d'échec
  }

  // 2. Demande 4 octets (32 bits)
  Wire.requestFrom(MD25, 4);
  unsigned long timeout = millis() + 100; // Timeout de 100ms
  while (Wire.available() < 4 && millis() < timeout);

  if (Wire.available() < 4) {
    Serial.println("Timeout: Lecture encodeur incomplete");
    return 0;
  }

  // 3. Lecture des 4 octets et conversion en 'long' signé
  uint32_t raw_value = 0;
  for (int i = 0; i < 4; i++) {
    raw_value = (raw_value << 8) | Wire.read();
  }

  // 4. Conversion en valeur signée (complément à 2)
  long valeur = (long)raw_value; // Conversion directe (gère le signe automatiquement)
  
  return valeur;
}
void definirVitesse(float vitesse1, float vitesse2) {
  vitesse1 = constrain(vitesse1, -100, 100);
  vitesse2 = constrain(vitesse2, -100, 100);
  
  Wire.beginTransmission(MD25);
  Wire.write(SPEED1);
  Wire.write(128 + (int8_t)vitesse1);
  Wire.write(128 + (int8_t)vitesse2);
  Wire.endTransmission();
  delay(10);
}
void stopMotors() {
  // Méthode 1 : Si vous utilisez le MD25 avec la librairie standard
  definirVitesse(0, 0);  // Met les 2 moteurs à vitesse 0
  
  Serial.println("Moteurs arrêtés");  // Debug
}