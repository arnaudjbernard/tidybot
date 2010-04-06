#ifndef DEF_MAIN_PROGRAM
#define DEF_MAIN_PROGRAM

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <string>
	
#include "args.h"

#define RAYS			32
#define SRAND_NB		5	// parameter for wander potential field
#define PI				3.141592653	// parameter for wander potential field

using namespace PlayerCc;



// functions definition
void init();

Vect avoidObstacles(LaserProxy &lp);
Vect wander(Position2dProxy &pp);
Vect goToBeacon(Position2dProxy &pp);
Vect vectCombine(Vect avoidObstaclesV, Vect wanderV, Vect goToBeaconV);
Vect move(Vect combinedVect, Position2dProxy &pp);

// Global variables for convenient programming
int nbBeacons, beacons[100], origin[2];//Error if more than 100 beacons
int mapSize[2], beaconRange;
double *wanderField = NULL;
bool reachedBeacon;

//PID
double error;
double integral;



#endif
