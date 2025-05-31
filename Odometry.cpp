#include "Odometry.h"
#include <Arduino.h>
#include <math.h>

Position robotPosition = {1.25, 0, 0, 0, 0.23, 0.26};
const int ticksParTour = 358;
const float diametreRoue = 0.1;
const float circonferenceRoue = PI * diametreRoue;

void initOdometry() {
  robotPosition = {1.25, 0, 0, millis(), 0.23, 0.26};
}

void updatePosition(float distanceRoue1, float distanceRoue2) {
  float deplacement = (distanceRoue1 + distanceRoue2) / 2.0f;
  float pente = (distanceRoue1 - distanceRoue2) / robotPosition.largeur;
  float deltaTheta = atan(pente)*(180/PI);
  
  robotPosition.x += deplacement * sin(deltaTheta);
  robotPosition.y += deplacement * cos(deltaTheta);
  robotPosition.theta += deltaTheta;
  while(robotPosition.theta >= 360.0f)robotPosition.theta -= 360.0f;
  while(robotPosition.theta < 0.0f)robotPosition.theta +=360.0f;
  
  //robotPosition.theta = atan2(sin(robotPosition.theta), cos(robotPosition.theta));
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