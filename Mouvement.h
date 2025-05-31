#include "Odometry.h"

#ifndef MOUVEMENT_H
#define MOUVEMENT_H

extern float distanceParcourue;
extern float distancex;
extern float distancey;
extern int etape;
extern const float margeArret;
extern int TICKS_90_DEGRES;
typedef enum {
  None,
  Gauche,
  Droite
}Actionprec;

typedef enum{
  Rien,
  X,
  Y
}Deplacementprec;

extern Actionprec actionprec;
extern Deplacementprec deplacementprec;

void avancer(float vitesseInitiale, float vitesseMinimale, float distanceCible);
void reculer(float vitesseInitiale, float vitesseMinimale, float distanceCible);
void tournerDroite(Position pos);
void tournerGauche(Position pos);
void ajustementangle(float angle, int vitesse);
void reglerPosition(Position PosElement);
//void reglerPosition1(Position PosElement);
void calibrer90Degres();

#endif