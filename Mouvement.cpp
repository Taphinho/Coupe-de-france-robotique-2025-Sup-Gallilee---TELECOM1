#include "Mouvement.h"
#include "Motor.h"
#include "Odometry.h"
#include <Arduino.h>


float distanceParcourue;
float distancex;
float distancey;
int etape;
bool objectifAtteint;
Actionprec actionprec;
Deplacementprec deplacementprec;
AouR aour;
const float margeArret = 0;
int TICKS_90_DEGRES;
int essais;
long enc1_avant;
long enc2_avant;

void initVar() {
  distanceParcourue = 0.0;
  distancex = 0;
  distancey = 0;
  etape = 1;
  objectifAtteint = 0;
  actionprec = None;
  deplacementprec = Rien;
  aour = R;
  TICKS_90_DEGRES = 169;
  enc1_avant = 0;
  enc2_avant = 0;
  essais = 0;

}


void avancer(float vitesseInitiale, float vitesseMinimale, float distanceCible) {
  static const float decelerationStartRatio = 0.6f;  // Ratio pour commencer la décélération
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
  updatePosition(delta1, delta2, aour);
  //Serial.print(enc1-enc1_avant);
  distanceParcourue += fabs(deltaMoy);
  enc1_avant = enc1;
  enc2_avant = enc2;

  // Calcul de la distance restante avec sécurité
  const float distanceRestante = max(0.0f, distanceCible - distanceParcourue);

  // Gestion de la vitesse avec anticipation
  if (distanceRestante <= margeArret) {
    definirVitesse(0.0f, 0.0f);
    etape++;
    objectifAtteint=1;
    Serial.print(F("ARRET - Distance finale: "));  // F() pour économiser la RAM
    Serial.print(distanceParcourue * 100.0f, 2);   // 2 décimales
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
  const float decelerationStart = 0.6 * distanceCible;
  long enc1 = lireEncodeur(ENCODER1);
  long enc2 = lireEncodeur(ENCODER2);


if (enc1 == enc1_avant && enc2 == enc2_avant) {
    essais++;
    if (essais > 5) { // Après 5 essais sans mouvement
        Serial.println("Robot bloqué, arrêt");
        definirVitesse(0, 0);
        return;
    }
} else {
    essais = 0;
}
  // Calcul différentiel pour éviter l'accumulation d'erreurs
  float delta1 = (enc1 - enc1_avant) / (float)ticksParTour * circonferenceRoue;
  float delta2 = (enc2 - enc2_avant) / (float)ticksParTour * circonferenceRoue;
  float deltaMoy = (delta1 + delta2) / 2.0;

  updatePosition(delta1, delta2, aour);
  //Serial.println(enc1-enc1_avant);

  distanceParcourue += abs(deltaMoy);
  Serial.println(distanceParcourue);
  enc1_avant = enc1;
  enc2_avant = enc2;

  float distanceRestante = distanceCible - distanceParcourue;

  if (distanceParcourue >= distanceCible) {
    definirVitesse(0, 0);
    objectifAtteint = 1;
    Serial.print("RETOU*********************R terminé - Distance totale: ");
    updatePosition(delta1, delta2, aour);

    Serial.print(distanceParcourue * 100);
    Serial.println(" cm");
    //while(true); // Arrêt définitif
    return;
  }

  if (distanceRestante <= decelerationStart) {
    float ratio = distanceRestante / (distanceCible - decelerationStart);
    float vitesse = vitesseMinimale + (vitesseInitiale - vitesseMinimale) * ratio * ratio;
    definirVitesse(vitesse, vitesse);
  } else {
    definirVitesse(vitesseInitiale, vitesseInitiale);
    Serial.println("vitesse");
  }

  // Affichage toutes les 200ms
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage > 200) {
    dernierAffichage = millis();
    Serial.print("Parcouru: ");
    Serial.print(distanceParcourue * 100, 1);
    Serial.print("cm");
    Serial.print(" | Restant: ");
    Serial.print(distanceRestante * 100, 1);
    Serial.print("cm");
    Serial.print(" | Vitesse: ");
    Serial.println((distanceRestante <= decelerationStart) ? "DECEL" : "CRUISE");
    printPosition();
  }
}

void tournerDroite(Position pos) {
  resetEncodeurs();
  definirVitesse(30, -30);

  long ticks = 0;
  while (abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1 - enc2) / 2;
    enc1_avant = enc1;
    enc2_avant = enc2;
    delay(10);
  }
  definirVitesse(0, 0);
  ajustementangle(pos.theta, 1);
}
void tournerGauche(Position pos) {
  resetEncodeurs();
  definirVitesse(-30, 30);

  long ticks = 0;
  while (abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1 - enc2) / 2;
    enc1_avant = enc1;
    enc2_avant = enc2;
    delay(10);
  }
  definirVitesse(0, 0);
  ajustementangle(pos.theta, 1);
}
void tournerDroites(Position pos) {
  resetEncodeurs();
  definirVitesse(30, -30);

  long ticks = 0;
  while (abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1 - enc2) / 2;
    enc1_avant = enc1;
    enc2_avant = enc2;
    delay(10);
  }
  definirVitesse(0, 0);
  //ajustementangle(pos.theta,1);
}
void tournerGauches(Position pos) {
  resetEncodeurs();
  definirVitesse(-30, 30);

  long ticks = 0;
  while (abs(ticks) < TICKS_90_DEGRES) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1 - enc2) / 2;
    enc1_avant = enc1;
    enc2_avant = enc2;
    delay(10);
  }
  definirVitesse(0, 0);
  //ajustementangle(pos.theta,1);
}
void tourner180(Position pos) {
  resetEncodeurs();
  definirVitesse(20, -20);

  long ticks = 0;
  while (abs(ticks) < 2 * 170) {
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);
    ticks = (enc1 - enc2) / 2;
    enc1_avant = enc1;
    enc2_avant = enc2;
    delay(10);
  }
}
// Conversion degrés -> radians
float degToRad(float degrees) {
  return degrees * PI / 180.0f;
}

// Conversion radians -> degrés
float radToDeg(float radians) {
  return radians * 180.0f / PI;
}
void ajustementangle(float angleCibleDeg, int vitesse) {
  float angleCible = angleCibleDeg * PI / 180.0f;

  // Paramètres du robot
  const float entraxe = 0.23f;       // Distance entre les roues en mètres
  const float seuilAngle = 0.0087f;  // ≈ 0.5 degré en radians
  //const int ticksParTour = 2000; // Ajuster selon votre encodeur
  const float ticksParRadian = ticksParTour * entraxe / (2 * PI * 0.05);

  // Réinitialiser les compteurs
  enc1_avant = lireEncodeur(ENCODER1);
  enc2_avant = lireEncodeur(ENCODER2);

  // Calcul du nombre de ticks nécessaires
  float erreurAngle = normaliserAngle(angleCible - robotPosition.theta);
  long ticksNecessaires = abs(erreurAngle * ticksParRadian);

  Serial.print("Rotation de ");
  Serial.print(angleCibleDeg);
  Serial.print("° (");
  Serial.print(ticksNecessaires);
  Serial.println(" ticks)");

  // Déterminer la direction
  int dir = (erreurAngle > 0) ? 1 : -1;

  // Boucle de contrôle
  long ticksCumules = 0;
  while (ticksCumules < ticksNecessaires) {
    // Appliquer la vitesse (moteurs en opposition pour tourner sur place)
    definirVitesse(-dir * vitesse, dir * vitesse);

    // Lire les encodeurs
    long enc1 = lireEncodeur(ENCODER1);
    long enc2 = lireEncodeur(ENCODER2);

    // Calculer le déplacement angulaire en ticks
    long delta1 = enc1 - enc1_avant;
    long delta2 = enc2 - enc2_avant;
    ticksCumules += abs(delta1 + delta2) / 2;

    // Mettre à jour la position
    updatePosition(delta1, delta2, aour);  // À implémenter

    // Sauvegarder les valeurs
    enc1_avant = enc1;
    enc2_avant = enc2;

    // Debug
    Serial.print("Ticks cumulés: ");
    Serial.print(ticksCumules);
    Serial.print("/");
    Serial.print(ticksNecessaires);
    Serial.print(" - Angle actuel: ");
    Serial.println(robotPosition.theta * 180.0f / PI);

    delay(10);  // Délai de contrôle
  }

  // Arrêt des moteurs
  definirVitesse(0, 0);
}

// Fonction pour normaliser un angle entre -PI et PI (ou -180 et 180)
float normaliserAngle(float angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

void reglerPosition(Position PosElement) {
  if (etape == 3) {
    switch (actionprec) {
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

  //tourner180(PosElement);
  // ajustementangle(180, 5);
  // Calcul des centres du robot et de l'élément
  float robot_centre_x = robotPosition.x;
  float robot_centre_y = robotPosition.y;
  float objet_centre_x = PosElement.x;
  float objet_centre_y = PosElement.y;

  float dx = robot_centre_x - objet_centre_x;
  float dy = robot_centre_y - objet_centre_y;

  switch (int(PosElement.theta)) {
    case 0:  // Objet regarde vers le haut (Y+), robot doit être derrière => y > objet
      deplacementprec = Y;
      if (robot_centre_y <= objet_centre_y) {
        if (robot_centre_x < objet_centre_x) {
          tournerDroites(PosElement);
          actionprec = Droite;
        } else {
          tournerGauches(PosElement);
          actionprec = Gauche;
        }
      } else {
        if (fabs(dx) < (PosElement.largeur + robotPosition.largeur) / 2) {
          if (robot_centre_x < objet_centre_x) {
            tournerGauches(PosElement);
          } else {
            tournerDroites(PosElement);
          }

          while (distanceParcourue < PosElement.largeur / 2) {
            avancer(-10, -10, PosElement.largeur / 2);
          };
          distanceParcourue = 0.0f;
          ajustementangle(PosElement.theta, 5);
        }
        definirVitesse(5,5);
        while (distanceParcourue < fabs(dy) + 2 * PosElement.longueur + robotPosition.longueur) {
          printf("ici 0");
          reculer(-10, -10, fabs(dy) + 2 * PosElement.longueur + robotPosition.longueur);
        }
        distanceParcourue = 0.0f;
        return;
      }
      break;

    case 90:  // Objet regarde vers la droite (X+), robot doit être derrière => x > objet
      deplacementprec = X;

      if (robot_centre_x <= objet_centre_x) {
        if (robot_centre_y < objet_centre_y) {
          tournerDroites(PosElement);
          actionprec = Droite;
        } else {
          tournerGauches(PosElement);
          actionprec = Gauche;
        }
      } else {
        if (fabs(dy) < (PosElement.longueur + robotPosition.longueur) / 2) {
          if (robot_centre_y < objet_centre_y) {
            tournerDroites(PosElement);
          } else {
            tournerGauches(PosElement);
          }

          while (distanceParcourue < PosElement.longueur / 2) {
            avancer(-10, -10, PosElement.longueur / 2);
          }
          distanceParcourue = 0.0f;
          ajustementangle(PosElement.theta, 5);
        }

        while (distanceParcourue < fabs(dx) + 1.2 * PosElement.largeur + robotPosition.largeur) {
          printf("ici 90");
          reculer(-10, -10, fabs(dx) + 1.2 * PosElement.largeur + robotPosition.largeur);
        }
        distanceParcourue = 0.0f;
        return;
      }
      break;

    case 180:  // Objet regarde vers le bas (Y−), robot doit être derrière => y < objet
      deplacementprec = Y;
      //Serial.println(robot_centre_y);
      //Serial.println(objet_centre_y);
      if (robot_centre_y >= objet_centre_y) {
        aour = Avancer;
        resetEncodeurs();

        if (robot_centre_x < objet_centre_x) {
          tournerGauches(PosElement);
          actionprec = Gauche;
        } else {
          tournerDroites(PosElement);
          actionprec = Droite ;
          Serial.println("dis");
        }
      } else {
        if (fabs(dx) < (PosElement.largeur + robotPosition.largeur) / 2) {
          if (robot_centre_x < objet_centre_x) {
            tournerDroites(PosElement);
          } else {
            tournerGauches(PosElement);
          }

          while (distanceParcourue < PosElement.largeur / 2) {
            avancer(-10, -10, PosElement.largeur / 2);
          }
          distanceParcourue = 0.0f;
          //ajustementangle(PosElement.theta,5);
        } else {
          float cmp = fabs(dy) + PosElement.longueur + robotPosition.longueur;
          aour = Reculer;
          resetEncodeurs();
          while (!objectifAtteint) {
            if(essais>5){
              return;
            }
            
            reculer(10, 10, cmp);
          
            //cmp = fabs(dy) + PosElement.longueur + robotPosition.longueur;
          }
          distanceParcourue = 0.0f;
        }
        return;
      }
      break;

    case 270:  // Objet regarde vers la gauche (X−), robot doit être derrière => x < objet
      deplacementprec = X;
      if (robot_centre_x >= objet_centre_x) {
        if (robot_centre_y < objet_centre_y) {
          tournerDroites(PosElement);
          actionprec = Droite;
        } else {
          tournerGauches(PosElement);
          actionprec = Gauche;
        }
      } else {
        if (fabs(dy) < (PosElement.longueur + robotPosition.longueur) / 2) {
          if (robot_centre_y < objet_centre_y) {
            tournerGauches(PosElement);
          } else {
            tournerDroites(PosElement);
          }

          while (distanceParcourue < PosElement.longueur / 2) {
            avancer(-10, -10, PosElement.longueur / 2);
          }
          distanceParcourue = 0.0f;
          ajustementangle(PosElement.theta, 5);
        }

        while (distanceParcourue < fabs(dx) + 2 * PosElement.largeur + robotPosition.largeur) {
          reculer(-10, -10, fabs(dx) + 2 * PosElement.largeur + robotPosition.largeur);
        }
        distanceParcourue = 0.0f;
        return;
      }
      break;

    default:
      return;  // Orientation inconnue
  }
  
  etape++;
  return;
}




void calibrer90Degres() {
  Serial.println("Calibration en cours...");
  resetEncodeurs();
  definirVitesse(-30, 30);
  delay(1000);  // Tourner pendant 1s
  definirVitesse(0, 0);

  long enc1 = lireEncodeur(ENCODER1);
  long enc2 = lireEncodeur(ENCODER2);
  TICKS_90_DEGRES = abs(enc2 - enc1) / 2;

  Serial.print("Nouvelle valeur calibrée: ");
  Serial.println(TICKS_90_DEGRES);
}