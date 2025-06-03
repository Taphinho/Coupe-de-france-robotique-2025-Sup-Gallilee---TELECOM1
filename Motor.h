#ifndef MOTOR_H
#define MOTOR_H

#include <Wire.h>

#define MD25 0x58
#define ENCODER1 0x02 
#define ENCODER2 0x06
#define SPEED1 0x00
#define SPEED2 0x01
#define REG_COMMAND 0x10
#define RESET_ENCODER 0x20

void initMD25();
bool md25Ready();
void resetEncodeurs();
long lireEncodeur(int registre);
void definirVitesse(float vitesse1, float vitesse2);

#endif