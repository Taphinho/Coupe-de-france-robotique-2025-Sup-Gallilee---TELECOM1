#include <Wire.h>
#include "Motor.h"
#include "Odometry.h"
#include "Mouvement.h"



Position Plat1 ={0.78,0.25,180,millis(),0.1,0.4 };

Position Plat2 = {1.10, 0.95,180,millis(),0.1,0.4};

Position pos;

void setup() {
  Serial.begin(115200);
  initOdometry();
  initMD25();
  resetEncodeurs();
  delay(100);
  pos = Plat1;
  Serial.println("Système prêt");
}

void loop() {
  delay(1000);
  switch(etape) {
    case 1:
      distanceParcourue = 0.0;
      reglerPosition(pos);
      break;
    case 2:
      delay(1000);
      if(deplacementprec==X){
        avancer(-30, -10, distancey);
      }else{
        avancer(-30, -10, distancex);
      } 
      
      resetEncodeurs();
      break;
    case 3:
      distanceParcourue = 0.0;
      reglerPosition(pos);
      break;
    case 4:
        delay(1000);
      if(deplacementprec==X){
        avancer(-30, -10, distancey);
      }else{
        avancer(-30, -10, distancex);
      }
      
      
      resetEncodeurs();
      if(pos.x==Plat2.x && pos.y==Plat2.y){
        definirVitesse(0,0);
        exit(0);
      }
      pos = Plat2;
      break;
  }
  delay(10);
}