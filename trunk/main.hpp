/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#ifndef DEF_MAIN_PROGRAM
#define DEF_MAIN_PROGRAM

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <string>
#include <utility>

#include <cv.h>
#include <cxcore.h>
#include <cvaux.h>
#include <highgui.h>


//VERBOSITY level
//0:	no output
//1:	*****required output
//2:	****greetings
//4:	***main functions
//8:	**functions process
//16:	*process data
//32:	non-denominated ones
//64:	debug
int VERBOSITY = 1 + 2 + 4 + 8;



#include "args.h"
#include "Vect.hpp"
#include "Obstacle.hpp"
#include "Node.hpp"
#include "PathPlanner.hpp"

#define RAYS			32
#define SRAND_NB		5	// parameter for wander potential field
#define PI				3.141592653	// parameter for wander potential field

#define TR_SWITCH_MODE_3	30 //define how close the robot needs to be from the can to switch to mode 3

using namespace PlayerCc;


// functions definition
void init();

Vect avoidObstacles(LaserProxy &lp);
Vect wander(Position2dProxy &pp);
Vect goToBeacon(player_pose2d position);
Vect vectCombine(Vect avoidObstaclesV, Vect wanderV, Vect goToBeaconV);
Vect move(Vect combinedVect, Position2dProxy &pp);

void computePosition(LaserProxy &lp, Position2dProxy &pp, Position2dProxy &pMCLp);

Vect searchCan(LaserProxy &lp, CameraProxy &cp, Position2dProxy &pp);
Vect followPath(LaserProxy &lp, Position2dProxy &pp, PathPlanner &pathPlanner);
Vect grabCan(PlayerClient &robot, LaserProxy &sp, Position2dProxy &pp,
		Position2dProxy &pMCLp, ActArrayProxy &aa);
Vect putDownCan(PlayerClient &robot, ActArrayProxy &aa);
player_pose2d locateCan(const cv::Mat &imgClean);
player_pose2d locateCan(LaserProxy &sp);


// Global variables for convenient programming




//modes:
//1 - wander
//2 - goto path
//3 - grab can
//4 - put down can
int mode = 1;

//can type:
//0: CocaCola
//1: Sprite
int canType;

int cam_width = 1;
int cam_height = 1;
int cam_depth = 3;



//int origin[2];
int mapSize[2];
double *wanderField = NULL;
bool canSeen;

std::vector<std::pair<int, int> > path;

//PID
double error;
double integral;

int Split(std::vector<std::string>& vecteur, std::string chaine, char separateur);

#endif
