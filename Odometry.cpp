#include "Odometry.h"
#include <Arduino.h>
#include <math.h>

#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

Position robotPosition ;
const int ticksParTour = 358;
const float diametreRoue = 0.1;
const float circonferenceRoue = PI * diametreRoue;

void initOdometry() {
  robotPosition = {1.25, 0, 180, millis(), 0.23, 0.26};
}

void updatePosition(float distanceRoue1, float distanceRoue2, AouR aour) {
  
    const float largeur = robotPosition.largeur;
    float deltaThetaRad = (distanceRoue1 - distanceRoue2) / largeur;
    float deplacement = 0.5f * (distanceRoue1 + distanceRoue2);
    
    if(aour == Reculer){
        deplacement = -deplacement;
        deltaThetaRad = -deltaThetaRad;
    }
    float thetaDeg = robotPosition.theta;
    float thetaRad = thetaDeg * DEG_TO_RAD;
    //float thetaRadNew = thetaRad + deltaThetaRad;

    if (fabs(deltaThetaRad) < 0.3f) {
        // Déplacement linéaire
     robotPosition.x += deplacement * sin(thetaRad);
     robotPosition.y += deplacement * cos(thetaRad);
    } else {
        // Mouvement circulaire
        float rayon = deplacement / deltaThetaRad;
        float thetaRadNew = thetaRad + deltaThetaRad;

        robotPosition.x += rayon * (sin(thetaRadNew) - sin(thetaRad));
        robotPosition.y -= rayon * (cos(thetaRadNew) - cos(thetaRad));

        thetaRad = thetaRadNew;
    }

    // Conversion et normalisation en degrés
    thetaDeg = fmod(thetaRad * RAD_TO_DEG, 360.0f);
    if (thetaDeg < 0.0f) thetaDeg += 360.0f;

    robotPosition.theta = thetaDeg;
    robotPosition.lastUpdate = millis();
}

Position getCurrentPosition() {
  return robotPosition;
}

void printPosition() {
  Position pos = getCurrentPosition();
  Serial.print("Position: X=");
  Serial.print(pos.x, 3);
  Serial.print("m Y=");
  Serial.print(pos.y, 3);
  Serial.print("m Theta=");
  Serial.print(pos.theta, 1);
  Serial.println("°");
}