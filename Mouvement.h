#include "Odometry.h"

#ifndef MOUVEMENT_H
#define MOUVEMENT_H

extern float distanceParcourue;
extern float distancex;
extern float distancey;
extern int etape;
extern bool objectifAtteint;
extern long enc1_avant , enc2_avant ;
extern const float margeArret;
extern int TICKS_90_DEGRES;
extern int essais;
extern AouR aour;
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

void initVar();
bool obstacleDetected();
void avancer(float vitesseInitiale, float vitesseMinimale, float distanceCible);
void reculer(float vitesseInitiale, float vitesseMinimale, float distanceCible);
void tournerDroite(Position pos);
void tournerGauche(Position pos);
void tournerGauches(Position pos);
void tournerDroites(Position pos);
void ajustementangle(float angle, int vitesse);
void reglerPosition(Position PosElement);
//void reglerPosition1(Position PosElement);
float normaliserAngle(float angle);
float radToDeg(float radians);
float degToRad(float degrees);
void tourner180(Position pos);
void calibrer90Degres();

#endif