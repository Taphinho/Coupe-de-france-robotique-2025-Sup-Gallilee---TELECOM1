#ifndef ODOMETRY_H
#define ODOMETRY_H

struct Position {
  float x;
  float y;
  float theta;
  unsigned long lastUpdate;
  float largeur;
  float longueur;
};

extern Position robotPosition;
extern const int ticksParTour;
extern const float circonferenceRoue;

void initOdometry();
void updatePosition(float distanceRoue1, float distanceRoue2);
Position getCurrentPosition();
void printPosition();

#endif