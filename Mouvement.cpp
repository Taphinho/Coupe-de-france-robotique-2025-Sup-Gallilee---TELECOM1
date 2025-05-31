#include "Mouvement.h"
#include "Motor.h"
#include "Odometry.h"
#include <Arduino.h>

float distanceParcourue = 0.0;
float distancex=0;
float distancey=0;
int etape = 1;
Actionprec actionprec = None;
Deplacementprec deplacementprec = Rien;
const float margeArret = 0;
int TICKS_90_DEGRES = 160;
long enc1_avant=0 , enc2_avant=0 ;



void avancer(float vitesseInitiale, float vitesseMinimale, float distanceCible) {
  static const float decelerationStartRatio = 0.6f; // Ratio pour commencer la décélération
  const float decelerationStart = decelerationStartRatio * distanceCible;
  //const float margeArret = 0.005f; // 5mm de marge d'arrêt (ajustable)
  
  // Lecture des encodeurs
  const long enc1 = lireEncodeur(ENCODER1);
  const long enc2 = lireEncodeur(ENCODER2);
  
  // Calcul du déplacement depuis la dernière lecture
  const float delta1 = (enc1 - enc1_avant) / static_cast<float>(ticksParTour) * circonferenceRoue;
  const float delta2 = (enc2 - enc2_avant) / static_cast<float>(ticksParTour) * circonferenceRoue;
  const float deltaMoy = (delta1 + delta2) * 0.5f;
  
  // Mise à jour de la position et de la distance parcourue
  updatePosition(delta1, delta2);
  Serial.print(enc1-enc1_avant);
  distanceParcourue += fabs(deltaMoy);
  enc1_avant = enc1;
  enc2_avant = enc2;
  
  // Calcul de la distance restante avec sécurité
  const float distanceRestante = max(0.0f, distanceCible - distanceParcourue);
  
  // Gestion de la vitesse avec anticipation
  if (distanceRestante <= margeArret) {
    definirVitesse(0.0f, 0.0f);
    etape++;
    Serial.print(F("ARRET - Distance finale: ")); // F() pour économiser la RAM
    Serial.print(distanceParcourue * 100.0f, 2); // 2 décimales
    Serial.println(F(" cm"));
    //distanceParcourue = 0.0f; // Réinitialisation pour le prochain mouvement
    return;
  }
  
  // Contrôle de la vitesse
  float vitesse = vitesseInitiale;
  if (distanceRestante <= decelerationStart) {
    // Décélération quadratique pour plus de précision
    const float ratio = distanceRestante / decelerationStart;
    vitesse = vitesseMinimale + (vitesseInitiale - vitesseMinimale) * ratio * ratio;
  }
  
  // Application des vitesses
  definirVitesse(vitesse, vitesse);
  
  // Affichage toutes les 200ms (sans bloquer)
  static unsigned long dernierAffichage = 0;
  const unsigned long maintenant = millis();
  if (maintenant - dernierAffichage >= 200) {
    dernierAffichage = maintenant;
    
    Serial.print(F("Parcouru: "));
    Serial.print(distanceParcourue * 100.0f, 1);
    Serial.print(F("cm | Restant: "));
    Serial.print(distanceRestante * 100.0f, 1);
    Serial.print(F("cm | Vitesse: "));
    Serial.println((distanceRestante <= decelerationStart) ? F("DECEL") : F("CRUISE"));
  }
}

void reculer(float vitesseInitiale, float vitesseMinimale, float distanceCible) {
  static const float decelerationStartRatio = 0.6f; // Ratio pour commencer la décélération
  const float decelerationStart = decelerationStartRatio * distanceCible;
  //const float margeArret = 0.005f; // 5mm de marge d'arrêt (ajustable)
  
  // Lecture des encodeurs
  const long enc1 = lireEncodeur(ENCODER1);
  const long enc2 = lireEncodeur(ENCODER2);
  
  // Calcul du déplacement depuis la dernière lecture
  const float delta1 = (enc1 - enc1_avant) / static_cast<float>(ticksParTour) * circonferenceRoue;
  const float delta2 = (enc2 - enc2_avant) / static_cast<float>(ticksParTour) * circonferenceRoue;
  const float deltaMoy = (delta1 + delta2) * 0.5f;
  
  // Mise à jour de la position et de la distance parcourue
  updatePosition(delta1, delta2);
  Serial.print(enc1-enc1_avant);
  distanceParcourue += fabs(deltaMoy);
  enc1_avant = enc1;
  enc2_avant = enc2;
  
  // Calcul de la distance restante avec sécurité
  const float distanceRestante = max(0.0f, distanceCible - distanceParcourue);
  
  // Gestion de la vitesse avec anticipation
  if (distanceRestante <= margeArret) {
    definirVitesse(0.0f, 0.0f);
    etape++;
    Serial.print(F("ARRET - Distance finale: ")); // F() pour économiser la RAM
    Serial.print(distanceParcourue * 100.0f, 2); // 2 décimales
    Serial.println(F(" cm"));
    //distanceParcourue = 0.0f; // Réinitialisation pour le prochain mouvement
    return;
  }
  
  // Contrôle de la vitesse
  float vitesse = -vitesseInitiale; // Vitesse négative pour reculer
  if (distanceRestante <= decelerationStart) {
    // Décélération quadratique pour plus de précision
    const float ratio = distanceRestante / decelerationStart;
    vitesse = - (vitesseMinimale + (vitesseInitiale - vitesseMinimale) * ratio * ratio);
  }
  
  // Application des vitesses (négatives pour reculer)
  definirVitesse(vitesse, vitesse);
  
  // Affichage toutes les 200ms (sans bloquer)
  static unsigned long dernierAffichage = 0;
  const unsigned long maintenant = millis();
  if (maintenant - dernierAffichage >= 200) {
    dernierAffichage = maintenant;
    
    Serial.print(F("Parcouru: "));
    Serial.print(distanceParcourue * 100.0f, 1);
    Serial.print(F("cm | Restant: "));
    Serial.print(distanceRestante * 100.0f, 1);
    Serial.print(F("cm | Vitesse: "));
    Serial.println((distanceRestante <= decelerationStart) ? F("DECEL") : F("CRUISE"));
  }
}

void tournerDroite(Position pos) {
  resetEncodeurs();
  definirVitesse(30, -30);
  
  long ticks = 0;
  while(abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks =(enc1 - enc2 )/2;
    enc1_avant= enc1;
    enc2_avant=enc2;
    delay(10);
}
definirVitesse(0, 0);
ajustementangle(pos.theta,1);
}
void tournerGauche(Position pos) {
  resetEncodeurs();
  definirVitesse(-30, 30);
  
  long ticks = 0;
  while(abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1 -enc2 )/2;
    enc1_avant= enc1;
    enc2_avant= enc2;
    delay(10);
  }
  definirVitesse(0, 0);
  ajustementangle(pos.theta,1);
  
  }
void tournerDroites(Position pos) {
  resetEncodeurs();
  definirVitesse(30, -30);
  
  long ticks = 0;
  while(abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1  - enc2) /2;
    enc1_avant= enc1;
    enc2_avant=enc2;
    delay(10);
}
definirVitesse(0, 0);
//ajustementangle(pos.theta,1);
}
void tournerGauches(Position pos) {
  resetEncodeurs();
  definirVitesse(-30, 30);
  
  long ticks = 0;
  while(abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1 - enc2 )/2;
    enc1_avant=enc1;
    enc2_avant=enc2;
    delay(10);
  }
  definirVitesse(0, 0);
  //ajustementangle(pos.theta,1);
  
  }
void ajustementangle(float angle, int vitesse){
  while(abs(angle - robotPosition.theta) >= 0.5 ){
    if (angle > robotPosition.theta ){
    definirVitesse(0,vitesse);
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    float delta1 = (enc1 - enc1_avant) / (float)ticksParTour * circonferenceRoue;
    float delta2 = (enc2 - enc2_avant) / (float)ticksParTour * circonferenceRoue;
    float deltaMoy = (delta1 + delta2) / 2.0;
    updatePosition(delta1,delta2);
    enc1_avant = enc1;
    enc2_avant = enc2;
    Serial.print("Angle: ");
    Serial.println(robotPosition.theta);
    Serial.print(angle);
    }
    else{
      definirVitesse(vitesse,0);
      long enc1 = lireEncodeur(ENCODER1);
      long enc2 = lireEncodeur(ENCODER2);
      float delta1 = (enc1 - enc1_avant) / (float)ticksParTour * circonferenceRoue;
      float delta2 = (enc2 - enc2_avant) / (float)ticksParTour * circonferenceRoue;
      float deltaMoy = (delta1 + delta2) / 2.0;
      updatePosition(delta1,delta2);
      enc1_avant = enc1;
      enc2_avant = enc2;
      Serial.print("Angle: ");
      Serial.println(robotPosition.theta);
    }
    

  }
}

void reglerPosition(Position PosElement) {
    if(etape == 3){
        switch(actionprec){
            case Gauche:
                tournerDroites(PosElement);
                actionprec = Droite;
                break;
            case Droite: 
                tournerGauches(PosElement);
                actionprec = Gauche;
                break;
        }
        etape++;
        return;
    }

    // Calcul des centres du robot et de l'élément
    float robot_centre_x = robotPosition.x + robotPosition.largeur / 2;
    float robot_centre_y = robotPosition.y + robotPosition.longueur / 2;
    float objet_centre_x = PosElement.x + PosElement.largeur / 2;
    float objet_centre_y = PosElement.y + PosElement.longueur / 2;

    float dx = robot_centre_x - objet_centre_x;
    float dy = robot_centre_y - objet_centre_y;

    switch(int(PosElement.theta)) {
        case 0: // Objet regarde vers le haut (Y+), robot doit être derrière => y > objet
            deplacementprec = Y;
            if (robot_centre_y <= objet_centre_y) {
                if (robot_centre_x < objet_centre_x) {
                    tournerDroite(PosElement);
                    actionprec = Droite;
                } else {
                    tournerGauche(PosElement);
                    actionprec = Gauche;
                }
            } else {
                if (fabs(dx) < (PosElement.largeur + robotPosition.largeur) / 2) {
                    if (robot_centre_x < objet_centre_x) {
                        tournerGauche(PosElement);
                    } else {
                        tournerDroite(PosElement);
                    }
                    
                    while (distanceParcourue < PosElement.largeur / 2){
                      avancer(-10,-10,PosElement.largeur / 2);
                    };
                    distanceParcourue=0.0f;
                    ajustementangle(PosElement.theta,5);
                }
                
                while (distanceParcourue < fabs(dy) + 2 * PosElement.longueur + robotPosition.longueur){
                  reculer(-10,-10,fabs(dy) + 2 * PosElement.longueur + robotPosition.longueur);
                }
                distanceParcourue=0.0f;
                return;
            }
            break;

        case 90: // Objet regarde vers la droite (X+), robot doit être derrière => x > objet
            deplacementprec = X;
            if (robot_centre_x <= objet_centre_x) {
                if (robot_centre_y < objet_centre_y) {
                    tournerDroite(PosElement);
                    actionprec = Droite;
                } else {
                    tournerGauche(PosElement);
                    actionprec = Gauche;
                }
            } else {
                if (fabs(dy) < (PosElement.longueur + robotPosition.longueur) / 2) {
                    if (robot_centre_y < objet_centre_y) {
                        tournerDroite(PosElement);
                    } else {
                        tournerGauche(PosElement);
                    }
                    
                    while (distanceParcourue < PosElement.longueur / 2){
                      avancer(-10,-10,PosElement.longueur / 2);
                    }
                    distanceParcourue=0.0f;
                    ajustementangle(PosElement.theta,5);
                }
               
                while (distanceParcourue < fabs(dx) + 2 * PosElement.largeur + robotPosition.largeur){
                   reculer(-10,-10,fabs(dx) + 2 * PosElement.largeur + robotPosition.largeur);
                }
                distanceParcourue=0.0f;
                return;
            }
            break;

        case 180: // Objet regarde vers le bas (Y−), robot doit être derrière => y < objet
            deplacementprec = Y;
            if (robot_centre_y >= objet_centre_y) {
                if (robot_centre_x < objet_centre_x) {
                    tournerGauche(PosElement);
                    actionprec = Gauche;
                } else {
                    tournerDroite(PosElement);
                    actionprec = Droite;
                }
            } else {
                if (fabs(dx) < (PosElement.largeur + robotPosition.largeur) / 2) {
                    if (robot_centre_x < objet_centre_x) {
                        tournerDroite(PosElement);
                    } else {
                        tournerGauche(PosElement);
                    }
                    
                    while (distanceParcourue < PosElement.largeur / 2){
                      avancer(-10,-10,PosElement.largeur / 2);
                    }
                    distanceParcourue=0.0f;
                    ajustementangle(PosElement.theta,5);
                }
                
                while (distanceParcourue < fabs(dy) + 2 * PosElement.longueur + robotPosition.longueur){
                  reculer(-10,-10,fabs(dy) + 2 * PosElement.longueur + robotPosition.longueur);
                }
                distanceParcourue=0.0f;
                return;
            }
            break;

        case 270: // Objet regarde vers la gauche (X−), robot doit être derrière => x < objet
            deplacementprec = X;
            if (robot_centre_x >= objet_centre_x) {
                if (robot_centre_y < objet_centre_y) {
                    tournerDroite(PosElement);
                    actionprec = Droite;
                } else {
                    tournerGauche(PosElement);
                    actionprec = Gauche;
                }
            } else {
                if (fabs(dy) < (PosElement.longueur + robotPosition.longueur) / 2) {
                    if (robot_centre_y < objet_centre_y) {
                        tournerGauche(PosElement);
                    } else {
                        tournerDroite(PosElement);
                    }
                
                    while (distanceParcourue < PosElement.longueur / 2){
                      avancer(-10,-10,PosElement.longueur / 2);
                    }
                    distanceParcourue=0.0f;
                    ajustementangle(PosElement.theta,5);
                }
                
                while (distanceParcourue < fabs(dx) + 2 * PosElement.largeur + robotPosition.largeur){
                  reculer(-10,-10,fabs(dx) + 2 * PosElement.largeur + robotPosition.largeur);
                }
                distanceParcourue=0.0f;
                return;
            }
            break;

        default:
            return; // Orientation inconnue
    }

    etape++;
}




void calibrer90Degres() {
  Serial.println("Calibration en cours...");
  resetEncodeurs();
  definirVitesse(-30, 30);
  delay(1000); // Tourner pendant 1s
  definirVitesse(0, 0);
  
  long enc1 = lireEncodeur(ENCODER1);
  long enc2 = lireEncodeur(ENCODER2);
  TICKS_90_DEGRES = abs(enc2 - enc1)/2;
  
  Serial.print("Nouvelle valeur calibrée: ");
  Serial.println(TICKS_90_DEGRES);
}