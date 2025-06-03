#include <Wire.h>
#include "Motor.h"
#include "Odometry.h"
#include "Mouvement.h"



Position Plat1 ={0.78,0.25,180,millis(),0.4,0.1 };

Position Plat2 = {1.10, 0.95,180,millis(),0.4,0.1};


void setup() {
  initVar();
  Serial.begin(115200);
  Wire.begin(); // Nécessaire pour la communication I2C

  // Attente du démarrage du MD25
  Serial.println("Vérification du MD25...");
  while (!md25Ready()) {
    Serial.println("MD25 non détecté, attente...");
    delay(500);
  }
  
  Serial.println("MD25 détecté !");
  initOdometry();
  initMD25();
  //definirVitesse(2, 2);
  resetEncodeurs();
  delay(1000);
  //pos = Plat1;
  Serial.println("Système prêt 1");
}
void loop() {
  resetEncodeurs();
  switch(etape) {
    case 1:
      //Serial.println("here");
      if(essais>5){
        initMD25();
        resetEncodeurs();
        delay(500);
      }
      essais=0;
      distanceParcourue = 0.0;
      
      reglerPosition(Plat1);
      Serial.println(etape);
      break;
    case 2:
    distancex=fabs(robotPosition.x - Plat1.x);
    distancey= fabs(robotPosition.y - Plat1.y);
    objectifAtteint=0;
    resetEncodeurs();
     delay(1000);
      if(deplacementprec==X){
        avancer(-30, -10, distancey);
      }else{
        avancer(-30, -10, distancex);
      
        Serial.println("hi thera");
      } 
      
      
      break;
    case 3:
       printf("here 3");
      distanceParcourue = 0.0;
      reglerPosition(Plat1);
      break;
    case 4:
     printf("here 3");
        delay(1000);
      if(deplacementprec==X){
        avancer(-30, -10, distancey);
      }else{
        avancer(-30, -10, distancex);
      }
      
      
      resetEncodeurs();
      break;
  }
  delay(10);
}