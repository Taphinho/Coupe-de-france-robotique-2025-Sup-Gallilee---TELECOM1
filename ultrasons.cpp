#include "ultrasons.h"

#define TRIG_PIN 7


float lireDistanceUltrason(int echoPin) {
  // Génération de l'impulsion trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Mesure du temps de vol
  unsigned long duree = pulseIn(echoPin, HIGH);
   if(duree == 0){
   float distance=11;
    return distance;
    }
  
  // Conversion en distance (cm)
  float distance = duree * 0.0343/2; // 0.0343/2 pré-calculé
  delay(75);
  // Validation de la plage
  return distance;
}

void printDistance(const char* caption, float distance) {
  Serial.print(caption);
  if (distance < 0) {
    Serial.println("hors-plage");
  } else {
    Serial.print(distance, 1);
    Serial.println(" cm");
  }
}