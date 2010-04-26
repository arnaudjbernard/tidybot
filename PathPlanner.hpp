/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

//Vector

#ifndef DEF_CLASS_PATH_PLANNER
#define DEF_CLASS_PATH_PLANNER


#define SCALE 2

//comment this line to disable OpenCv display
//#define OPENCV

#include <stdio.h>
#include <fstream>
#include <string.h>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <utility>


#ifdef OPENCV
#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>
#endif





#include "Obstacle.hpp"
#include "Node.hpp"

class PathPlanner
{
public:

	PathPlanner(int VERBOSITY);
	std::vector<std::pair<int, int> > getWayPoints(std::pair<int, int> origin, std::pair<int, int> goal);

protected:
#ifdef OPENCV
	cv::Mat imgMap, imgVis, imgSol, imgOri, imgOriSol;
#endif

	//VERBOSITY level
	//0:	no output
	//1:	*****required output
	//2:	****greetings
	//4:	***main functions
	//8:	**functions process
	//16:	*process data
	//32:	non-denominated ones
	//64:	debug
	int VERBOSITY;

	void readRoomFile();
	void growObstacles();
	void createVisibilityGraph();
	void applyAStar();
	std::vector<std::pair<int, int> > displayResult();
	std::pair<int, int> findInbound(std::pair<int, int> point);


	bool * map;
	bool pathFound;

	int mapH, mapW;
	int robotSize;
	int originX, originY;
	int goalX, goalY;


	std::vector<Obstacle> Obstacles;

	std::vector<Node *> Nodes;
	std::vector<Node *> openList;
	std::vector<Node *> closedList;
	Node * currentNode;
	Node * goalNode;

	//Bresenham's line algorithm inspired by:
	//http://rooparam.blogspot.com/2009/09/bresenhams-line-drawing-algorithm.html
	//the algorithm is detailed at:
	//http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	int INVERSE;    // 0 : false        1 : true
	int SIGN;       // 0 : y = x        1 : y = -x
	bool BHM(int x0, int y0, int x1, int y1);
	bool isVisible(int x0, int y0, int x1, int y1);
	bool isVisible(Node n1, Node n2);
	inline bool checkpixel(int x, int y);

	int Split(std::vector<std::string>& vecteur, std::string chaine, char separateur);
	int evalDistance(Node * n1, Node * n2);
	Node * bestNode(std::vector<Node *>& list);
	bool removeFromList(std::vector<Node *>& list, Node * n);
	bool isIn(std::vector<Node *>& list, Node * n);


};

#endif
