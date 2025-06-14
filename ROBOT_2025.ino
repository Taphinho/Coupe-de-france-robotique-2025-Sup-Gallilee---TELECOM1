#include <Wire.h>
#include <Servo.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "Motor.h"
#include "Odometry.h"
#include "Mouvement.h"
#include "ultrasons.h"

#define TRIG_PIN 7
#define ECHO_PIN_AVANT_1 4
#define ECHO_PIN_AVANT_2 5
#define ECHO_PIN_ARRIERE_1 2
#define ECHO_PIN_ARRIERE_2 3
#define OBSTACLE_DISTANCE 5.0

unsigned long previousMillis = 0;
const unsigned long interval = 30;

Servo servomoteur;
bool depart =1;

//int c=1;
int c=2;//bleu
//Position Plat1 ={0.77,0.25,180,millis(),0.4,0.1 };
Position Plat1 = {2.22,0.25,180,millis(),0.4,0.1};//bleu


//Position Plat2 = {1.10, 0.95,180,millis(),0.4,0.1};
Position Plat2 = {1.90,0.95,180,millis(),0.4,0.1};//bleu

//Position zone1 = {0.80, 0.05,180, 0,0.55,0.15 };
Position zone1={2.25,0.05,180,0,0.55,0.15};//bleu

//Position zone2 = {1.24, 0.13,180,0,0.55,0.15};
Position zone2={1.77,0.13,0,0.55,0.15};//bleu

//Position zone3 ={0.35,2.3,0,180,0.55,0.15};
Position zone3={2.6,2.3,0,180,0.55,0.15};//bleu

Position Pos = Plat1;

Position zone = zone1;


void stopArduino() {
  // Désactive les périphériques pour économiser plus d’énergie
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();

  // Met en veille profonde
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  noInterrupts(); // désactive les interruptions
  sleep_cpu();    // entre en veille - seul un reset ou une interruption externe peut en sortir
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN_AVANT_1, INPUT);
  pinMode(ECHO_PIN_AVANT_2, INPUT);
  pinMode(ECHO_PIN_ARRIERE_1, INPUT);
  pinMode(ECHO_PIN_ARRIERE_2, INPUT);
  pinMode(10, INPUT);
  servomoteur.attach(9);
  servomoteur.write(50);
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
  
  if (digitalRead(10) == HIGH) {
    return; // Sort de la loop() sans rien faire
  }
  

  if(depart){
    delay(1000);
    servomoteur.write(180); // Position à 90 degrés (peut être milieu ou autre selon calibration)
    delay(1000);
    servomoteur.write(50);
    depart=0;
  }
  unsigned long currentMillis = millis();

  // Exécute maFonctionParallel() toutes les 100 ms sans bloquer loop()

  resetEncodeurs();
  Serial.println(etape);
  
  if(Pos.x == zone3.x && c==1){
   distanceParcourue=0.0f;
   resetEncodeurs();
   while(distanceParcourue <= fabs(zone2.y - Pos.y)/4){
    reculer(20,20,fabs(zone2.y - Pos.y)/4);
   }
   resetEncodeurs();
   tournerDroites(Pos);
   resetEncodeurs();
   distanceParcourue=0.0f;
   while(distanceParcourue <= fabs(zone2.x - Pos.x)-0.15){
    avancer(-20,-20,fabs(zone2.x - Pos.x)-0.15);
   }
   resetEncodeurs();
   distanceParcourue=0.0f;
   tournerDroites(Pos);
   resetEncodeurs();
   while(distanceParcourue <= 3*fabs(zone2.y - Pos.y)/4){
      avancer(-20,-20,3*fabs(zone2.y - Pos.y)/4-0.2);
   }
   resetEncodeurs();
   stopArduino();

  }else if(Pos.x == zone3.x && c==2){
    distanceParcourue=0.0f;
   resetEncodeurs();
   while(distanceParcourue <= fabs(zone2.y - Pos.y)/4){
    reculer(20,20,fabs(zone2.y - Pos.y)/4);
   }
   resetEncodeurs();
   tournerGauches(Pos);
   resetEncodeurs();
   distanceParcourue=0.0f;
   while(distanceParcourue <= fabs(zone2.x - Pos.x)){
    avancer(-20,-20,fabs(zone2.x - Pos.x));
   }
   resetEncodeurs();
   distanceParcourue=0.0f;
   tournerGauches(Pos);
   resetEncodeurs();
   while(distanceParcourue <= 3*fabs(zone2.y - Pos.y)/4){
      avancer(-20,-20,3*fabs(zone2.y - Pos.y)/4);
   }
   resetEncodeurs();
   stopArduino();
  }

  
  switch(etape) {
          
    case 1:
    objectifAtteint=0;
    Serial.println(etape);
      //Serial.println("here");
      if(essais>5){
        initMD25();
        resetEncodeurs();
        delay(500);
      }
      essais=0;
      distanceParcourue = 0.0;
     
      reglerPosition(Pos);
      if(zone.x == zone1.x){
      //distancex=fabs(robotPosition.x - Pos.x)-0.1;
      distancex=fabs(robotPosition.x - Pos.x)-0.1;//bleu
      }else{
      //distancey= fabs(robotPosition.y - Pos.y)-0.1;
      distancex=fabs(robotPosition.x - Pos.x)-0.18;//bleu
      distancey= fabs(robotPosition.y - Pos.y);//bleu
      }
      break; 
    case 2:
    objectifAtteint=0;
    resetEncodeurs();
    printPosition();
     delay(1000);
      if(deplacementprec==X){
        aour = Avancer;
        avancer(-30, -10, distancey);
      }else{
        aour = Avancer;
        avancer(-30, -10, distancex);
        
      } 
      
      
      break;
    case 3:
    
      distanceParcourue = 0.0;
      resetEncodeurs();
      reglerPosition(Pos);
      distancex=fabs(robotPosition.x - Pos.x);
      if(zone.x == zone1.x){
      distancey= fabs(robotPosition.y - Pos.y)+0.1;//+fabs(Pos.y-zone.y)-0.2;
      }else{
      distancey=fabs(robotPosition.y - Pos.y)+0.07;
      }
    
      break;
    case 4:
    objectifAtteint=0;
    resetEncodeurs();
    printPosition();
    Serial.println(distancey);
    delay(1000);
      if(deplacementprec==X){
        aour = Avancer;
        avancer(-30, -10, distancey);
      }else{
        aour = Avancer;
        avancer(-30, -10, distancex);
        

      }
     
      if(zone.x == zone2.x){
         resetEncodeurs();
        distanceParcourue = 0.0;
        //tournerGauches(zone);
        resetEncodeurs();
        //while(distanceParcourue <= fabs(zone.x - Plat2.x)){
            
          //avancer(-20,-20,fabs(zone.x - Plat2.x));
        //}

        //resetEncodeurs();
        //distanceParcourue = 0.0;
        //tournerDroites(zone);
        resetEncodeurs();
        while(distanceParcourue <= fabs(zone.y-Plat2.y)+0.2){
                        
          avancer(-20,-20,fabs(zone.y-Plat2.y)+0.2);
        }
        delay(5000);
        Pos=zone3;
        zone=zone3;
        etape=0;
        break;
      }
     
       
    
      resetEncodeurs();
      if(objectifAtteint){
      
      printPosition();
      delay(1000);
      
      Pos= Plat2;
      zone=zone2;
      etape=1;
      Serial.println("hi thera");
      }
      break;
  }
  delay(10);
}
