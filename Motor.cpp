#include "Motor.h"
#include <Arduino.h>

void initMD25() {
  Wire.begin();
  Wire.setClock(200000);
}

void resetEncodeurs() {
  Wire.beginTransmission(MD25);
  Wire.write(REG_COMMAND);
  Wire.write(RESET_ENCODER);
  Wire.endTransmission();
}

long lireEncodeur(int registre) {
  Wire.beginTransmission(MD25);
  Wire.write(registre);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25, 4);
  while(Wire.available() < 4);
  
  long valeur = 0;
  for(int i=0; i<4; i++) {
    valeur = (valeur << 8) | Wire.read();
  }
  
  return (valeur > 0x7FFFFFFF) ? valeur - 0x100000000 : valeur;
}

void definirVitesse(float vitesse1, float vitesse2) {
  vitesse1 = constrain(vitesse1, -100, 100);
  vitesse2 = constrain(vitesse2, -100, 100);
  
  Wire.beginTransmission(MD25);
  Wire.write(SPEED1);
  Wire.write(128 + (int8_t)vitesse1);
  Wire.write(128 + (int8_t)vitesse2);
  Wire.endTransmission();
}