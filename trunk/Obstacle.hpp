/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#ifndef DEF_CLASS_OBSTACLE
#define DEF_CLASS_OBSTACLE

#include <iostream>

class Obstacle
{
public:
	Obstacle();
	Obstacle(int x, int y, int w, int h);
	void grow(int d, int mapW, int mapH);
	
	int x;
	int y;
	int w;
	int h;
};

#endif
